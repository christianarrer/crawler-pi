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
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class CliffCostmapNode(Node):

    CLIFF_STATE = 1

    def __init__(self):
        super().__init__('cliff_costmap_node')

        # Frames
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # Radius of one cliff exclusion area.
        self.declare_parameter('cliff_radius', 0.12)

        self.map_frame = self.get_parameter(
            'map_frame').get_parameter_value().string_value

        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value

        self.cliff_radius = self.get_parameter(
            'cliff_radius').get_parameter_value().double_value
        self.sensor_frames = {
            4: 'cliff_sensor_04',
            5: 'cliff_sensor_05',
            6: 'cliff_sensor_06',
            7: 'cliff_sensor_07',
        }
        self.cliff_points = []


        # State positions in Int64MultiArray:
        #
        # timestamp,
        # sensor04_value, sensor04_state, sensor04_extra,
        # sensor05_value, sensor05_state, sensor05_extra,
        # ...
        self.sensor_state_indices = {
            4: 3,
            5: 6,
            6: 9,
            7: 12,
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # We copy geometry from /map.
        self.map_info = None

        # Cliff map data:
        # 0   = free / no additional cost
        # 100 = cliff / lethal obstacle
        self.cliff_data = None

        # Used to avoid inserting the same cliff on every sensor message.
        self.previous_states = {
            4: 0,
            5: 0,
            6: 0,
            7: 0,
        }

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

        # Transient local makes the latest cliff map available to a
        # StaticLayer even if the layer subscribes later.
        cliff_map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cliff_map_pub = self.create_publisher(
            OccupancyGrid,
            '/crawler/cliff_map',
            cliff_map_qos,
        )

        self.get_logger().info(
            'Cliff costmap node started. '
            'Waiting for /map and /crawler/cliff_sensors.'
        )

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

            # -1 = NO_INFORMATION.
            # The cliff layer must make no statement about cells
            # where no cliff has been detected.
            self.cliff_data = [-1] * size

            self.get_logger().info(
                f'Initialized/recreated cliff map: '
                f'{msg.info.width} x {msg.info.height}, '
                f'resolution={msg.info.resolution:.3f} m, '
                f'origin=({msg.info.origin.position.x:.3f}, '
                f'{msg.info.origin.position.y:.3f})'
            )

            # SLAM may change map dimensions and/or origin.
            # Reproject all previously detected cliff locations
            # into the new OccupancyGrid.
            for cliff_x, cliff_y in self.cliff_points:
                self.mark_circle(
                    cliff_x,
                    cliff_y,
                    self.cliff_radius,
                )

            if self.cliff_points:
                self.get_logger().info(
                    f'Restored {len(self.cliff_points)} '
                    f'previous cliff point(s) after map resize.'
                )

        self.publish_cliff_map()

    def cliff_callback(self, msg: Int64MultiArray):

        if len(msg.data) < 13:
            self.get_logger().warn(
                f'Invalid cliff sensor message: '
                f'expected at least 13 values, got {len(msg.data)}'
            )
            return

        # Read current cliff states first.
        current_states = {
            4: int(msg.data[3]),
            5: int(msg.data[6]),
            6: int(msg.data[9]),
            7: int(msg.data[12]),
        }

        # No map yet is perfectly normal if Nav2/SLAM is not running.
        if self.map_info is None or self.cliff_data is None:
            self.previous_states.update(current_states)
            return

        for sensor_number, state in current_states.items():

            old_state = self.previous_states[sensor_number]

            if (
                state == self.CLIFF_STATE
                and old_state != self.CLIFF_STATE
            ):
                self.add_cliff(sensor_number)

            self.previous_states[sensor_number] = state

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

        # Store the actual world position.
        # This survives SLAM map resizing/origin changes.
        self.cliff_points.append(
            (map_x, map_y)
        )

        self.mark_circle(
            map_x,
            map_y,
            self.cliff_radius,
        )

        self.publish_cliff_map()

    def mark_circle(self, world_x: float, world_y: float, radius: float):

        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        width = self.map_info.width
        height = self.map_info.height

        center_x = int(
            math.floor((world_x - origin_x) / resolution)
        )

        center_y = int(
            math.floor((world_y - origin_y) / resolution)
        )

        radius_cells = int(
            math.ceil(radius / resolution)
        )

        radius_squared = (
            radius / resolution
        ) ** 2

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):

                if dx * dx + dy * dy > radius_squared:
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

                index = cell_y * width + cell_x

                self.cliff_data[index] = 100

    def publish_cliff_map(self):

        if self.map_info is None or self.cliff_data is None:
            return

        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info = self.map_info
        msg.data = self.cliff_data

        self.cliff_map_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = CliffCostmapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
