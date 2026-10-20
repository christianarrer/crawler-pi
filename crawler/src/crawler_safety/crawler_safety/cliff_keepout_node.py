#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import OccupancyGrid

from tf2_ros import Buffer, TransformListener, TransformException


class CliffKeepoutNode(Node):

    CLIFF_STATE = 1

    def __init__(self):
        super().__init__('cliff_keepout_node')

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------

        self.declare_parameter('map_frame', 'map')

        # Radius around each detected cliff point.
        self.declare_parameter('cliff_radius', 0.12)

        # Connect cliff points if they are sufficiently close.
        self.declare_parameter('connect_points', True)

        # Maximum distance between two points which may be connected.
        self.declare_parameter('connection_distance', 0.35)

        # Radius/thickness of the connecting line.
        # Normally use the same width as the cliff circles.
        self.declare_parameter('connection_radius', 0.12)

        self.map_frame = (
            self.get_parameter('map_frame')
            .get_parameter_value()
            .string_value
        )

        self.cliff_radius = (
            self.get_parameter('cliff_radius')
            .get_parameter_value()
            .double_value
        )

        self.connect_points = (
            self.get_parameter('connect_points')
            .get_parameter_value()
            .bool_value
        )

        self.connection_distance = (
            self.get_parameter('connection_distance')
            .get_parameter_value()
            .double_value
        )

        self.connection_radius = (
            self.get_parameter('connection_radius')
            .get_parameter_value()
            .double_value
        )

        # ------------------------------------------------------------
        # Sensor frames
        # ------------------------------------------------------------

        self.sensor_frames = {
            4: 'cliff_sensor_04',
            5: 'cliff_sensor_05',
            6: 'cliff_sensor_06',
            7: 'cliff_sensor_07',
        }

        # Positions of state fields in /crawler/cliff_sensors
        self.sensor_state_indices = {
            4: 3,
            5: 6,
            6: 9,
            7: 12,
        }

        # ------------------------------------------------------------
        # Stored cliff positions
        # ------------------------------------------------------------

        # Stored in world/map coordinates.
        #
        # This allows the OccupancyGrid to be recreated if SLAM
        # changes the map size or origin.
        self.cliff_points = []

        self.previous_states = {
            4: 0,
            5: 0,
            6: 0,
            7: 0,
        }

        # ------------------------------------------------------------
        # TF
        # ------------------------------------------------------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ------------------------------------------------------------
        # Map / keepout mask
        # ------------------------------------------------------------

        self.map_info = None
        self.keepout_data = None

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )

        self.cliff_sub = self.create_subscription(
            Int64MultiArray,
            '/crawler/cliff_sensors',
            self.cliff_callback,
            10,
        )

        # Keepout mask is latched / transient-local.
        # Nav2 may therefore start after this node.
        mask_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.keepout_pub = self.create_publisher(
            OccupancyGrid,
            '/crawler/cliff_keepout_mask',
            mask_qos,
        )

        self.get_logger().info(
            'Cliff keepout node started. '
            'Waiting for /map and /crawler/cliff_sensors.'
        )

    # ================================================================
    # Map handling
    # ================================================================

    def map_callback(self, msg: OccupancyGrid):

        recreate = False

        if self.map_info is None:
            recreate = True

        else:
            if (
                self.map_info.width != msg.info.width
                or self.map_info.height != msg.info.height
                or self.map_info.resolution != msg.info.resolution
                or self.map_info.origin.position.x
                != msg.info.origin.position.x
                or self.map_info.origin.position.y
                != msg.info.origin.position.y
            ):
                recreate = True

        self.map_info = msg.info

        if recreate:

            size = msg.info.width * msg.info.height

            # IMPORTANT:
            #
            # A keepout MASK is different from our previous
            # cliff costmap layer.
            #
            # 0   = allowed
            # 100 = keepout
            #
            # Therefore the default value must be 0, not -1.
            self.keepout_data = [0] * size

            self.get_logger().info(
                f'Initialized/recreated keepout mask: '
                f'{msg.info.width} x {msg.info.height}, '
                f'resolution={msg.info.resolution:.3f} m, '
                f'origin=({msg.info.origin.position.x:.3f}, '
                f'{msg.info.origin.position.y:.3f})'
            )

            # Rebuild all previously learned cliff areas.
            self.rebuild_keepout_mask()

        self.publish_keepout_mask()

    # ================================================================
    # Cliff sensor handling
    # ================================================================

    def cliff_callback(self, msg: Int64MultiArray):

        if len(msg.data) < 13:
            self.get_logger().warn(
                f'Invalid cliff sensor message: '
                f'expected at least 13 values, got {len(msg.data)}'
            )
            return

        current_states = {
            4: int(msg.data[3]),
            5: int(msg.data[6]),
            6: int(msg.data[9]),
            7: int(msg.data[12]),
        }

        # Without /map we currently cannot construct the mask.
        #
        # The node itself can still run without Nav2.
        if self.map_info is None or self.keepout_data is None:
            self.previous_states.update(current_states)
            return

        for sensor_number, state in current_states.items():

            old_state = self.previous_states[sensor_number]

            # Only insert once on transition:
            #
            # SAFE -> CLIFF
            #
            # This prevents continuously inserting the same location.
            if (
                state == self.CLIFF_STATE
                and old_state != self.CLIFF_STATE
            ):
                self.add_cliff(sensor_number)

            self.previous_states[sensor_number] = state

    # ================================================================
    # Add one detected cliff
    # ================================================================

    def add_cliff(self, sensor_number: int):

        sensor_frame = self.sensor_frames[sensor_number]

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                sensor_frame,
                rclpy.time.Time(),
            )

        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {sensor_frame} -> '
                f'{self.map_frame}: {exc}'
            )
            return

        map_x = transform.transform.translation.x
        map_y = transform.transform.translation.y

        self.get_logger().warn(
            f'CLIFF sensor {sensor_number:02d}: '
            f'x={map_x:.3f}, y={map_y:.3f}'
        )

        new_point = (map_x, map_y)

        # Find nearest previously detected cliff point.
        nearest_point = None
        nearest_distance = None

        for old_point in self.cliff_points:

            dx = map_x - old_point[0]
            dy = map_y - old_point[1]

            distance = math.hypot(dx, dy)

            if (
                nearest_distance is None
                or distance < nearest_distance
            ):
                nearest_distance = distance
                nearest_point = old_point

        # Store new point.
        self.cliff_points.append(new_point)

        # Mark circle around actual detection.
        self.mark_circle(
            map_x,
            map_y,
            self.cliff_radius,
        )

        # Connect to nearest existing cliff point if sufficiently close.
        #
        # This gradually turns multiple cliff detections along the same
        # stair edge into a continuous keepout barrier.
        if (
            self.connect_points
            and nearest_point is not None
            and nearest_distance <= self.connection_distance
        ):

            self.get_logger().info(
                f'Connecting cliff points, '
                f'distance={nearest_distance:.3f} m'
            )

            self.mark_line(
                nearest_point[0],
                nearest_point[1],
                map_x,
                map_y,
                self.connection_radius,
            )

        self.publish_keepout_mask()

    # ================================================================
    # Rebuild complete mask
    # ================================================================

    def rebuild_keepout_mask(self):

        if self.map_info is None:
            return

        size = self.map_info.width * self.map_info.height

        self.keepout_data = [0] * size

        # Draw all circles.
        for cliff_x, cliff_y in self.cliff_points:
            self.mark_circle(
                cliff_x,
                cliff_y,
                self.cliff_radius,
            )

        # Recreate connections.
        #
        # For each point, connect it to its nearest earlier point
        # if it lies within connection_distance.
        if self.connect_points:

            for index in range(1, len(self.cliff_points)):

                point = self.cliff_points[index]

                nearest_point = None
                nearest_distance = None

                for old_point in self.cliff_points[:index]:

                    distance = math.hypot(
                        point[0] - old_point[0],
                        point[1] - old_point[1],
                    )

                    if (
                        nearest_distance is None
                        or distance < nearest_distance
                    ):
                        nearest_distance = distance
                        nearest_point = old_point

                if (
                    nearest_point is not None
                    and nearest_distance <= self.connection_distance
                ):
                    self.mark_line(
                        nearest_point[0],
                        nearest_point[1],
                        point[0],
                        point[1],
                        self.connection_radius,
                    )

        if self.cliff_points:
            self.get_logger().info(
                f'Restored {len(self.cliff_points)} '
                f'cliff point(s) into keepout mask.'
            )

    # ================================================================
    # Drawing functions
    # ================================================================

    def mark_circle(
        self,
        world_x: float,
        world_y: float,
        radius: float,
    ):

        resolution = self.map_info.resolution

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        width = self.map_info.width
        height = self.map_info.height

        center_x = int(
            math.floor(
                (world_x - origin_x) / resolution
            )
        )

        center_y = int(
            math.floor(
                (world_y - origin_y) / resolution
            )
        )

        radius_cells = int(
            math.ceil(radius / resolution)
        )

        radius_squared = (
            radius / resolution
        ) ** 2

        for dy in range(
            -radius_cells,
            radius_cells + 1,
        ):
            for dx in range(
                -radius_cells,
                radius_cells + 1,
            ):

                if (
                    dx * dx + dy * dy
                    > radius_squared
                ):
                    continue

                cell_x = center_x + dx
                cell_y = center_y + dy

                if (
                    cell_x < 0
                    or cell_y < 0
                    or cell_x >= width
                    or cell_y >= height
                ):
                    continue

                index = (
                    cell_y * width
                    + cell_x
                )

                self.keepout_data[index] = 100

    def mark_line(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        radius: float,
    ):

        distance = math.hypot(
            x2 - x1,
            y2 - y1,
        )

        # Sample more densely than the OccupancyGrid resolution
        # so that no gaps can appear in the barrier.
        step_size = max(
            self.map_info.resolution * 0.5,
            0.005,
        )

        steps = max(
            1,
            int(math.ceil(distance / step_size)),
        )

        for i in range(steps + 1):

            t = i / steps

            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t

            self.mark_circle(
                x,
                y,
                radius,
            )

    # ================================================================
    # Publish
    # ================================================================

    def publish_keepout_mask(self):

        if (
            self.map_info is None
            or self.keepout_data is None
        ):
            return

        msg = OccupancyGrid()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = self.map_frame

        msg.info = self.map_info

        msg.data = self.keepout_data

        self.keepout_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = CliffKeepoutNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()