#!/usr/bin/env python3

import math
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class CrawlerOdomNode(Node):
    def __init__(self):
        super().__init__("crawler_odom_node")
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # Geometry / conversion parameters
        self.declare_parameter("track_width_m", 0.245)
        self.declare_parameter("chain_pitch_mm", 11.0)
        self.declare_parameter("sprocket_teeth", 12)
        self.declare_parameter("calibration_factor", 1.17)
        self.declare_parameter("calibration_factor_angular", 0.75)


        # Fusion / odometry parameters
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("imu_weight", 1.0)
        self.declare_parameter("stale_timeout_s", 2.0)

        # Twist smoothing for pose_source="revolutions".
        # The revolution counter is quantized by the magnets. Position is still
        # integrated from every delta, but twist is estimated over this time
        # window so /odom.twist does not jump between zero and short spikes.
        self.declare_parameter("twist_history_window_s", 0.8)

        # Pose integration source:
        #   "rpm"         -> integrate speed estimated from /odometer/*/rpm
        #   "revolutions" -> integrate distance increments from /odometer/*/revolutions
        self.declare_parameter("pose_source", "revolutions")

        # Frame names
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        # Optional sign corrections
        self.declare_parameter("left_dir_sign", 1)
        self.declare_parameter("right_dir_sign", 1)
        self.declare_parameter("imu_angular_z_sign", 1)
        self.declare_parameter("odom_angular_z_sign", 1)

        self.track_width_m = float(self.get_parameter("track_width_m").value)
        self.chain_pitch_mm = float(self.get_parameter("chain_pitch_mm").value)
        self.sprocket_teeth = int(self.get_parameter("sprocket_teeth").value)
        self.calibration_factor = float(self.get_parameter("calibration_factor").value)
        self.calibration_factor_angular = float(self.get_parameter("calibration_factor_angular").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.imu_weight = float(self.get_parameter("imu_weight").value)
        self.stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self.twist_history_window_s = float(
            self.get_parameter("twist_history_window_s").value
        )
        if self.twist_history_window_s <= 0.0:
            self.get_logger().warn(
                "twist_history_window_s must be > 0; falling back to 0.8 s"
            )
            self.twist_history_window_s = 0.8

        self.pose_source = str(self.get_parameter("pose_source").value).strip().lower()
        if self.pose_source not in ("rpm", "revolutions"):
            self.get_logger().warn(
                f"Invalid pose_source '{self.pose_source}'; falling back to 'revolutions'"
            )
            self.pose_source = "revolutions"

        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)

        self.left_dir_sign = int(self.get_parameter("left_dir_sign").value)
        self.right_dir_sign = int(self.get_parameter("right_dir_sign").value)
        self.imu_angular_z_sign = int(self.get_parameter("imu_angular_z_sign").value)
        self.odom_angular_z_sign = int(self.get_parameter("odom_angular_z_sign").value)

        self.imu_weight = max(0.0, min(1.0, self.imu_weight))

        self.distance_per_sprocket_rev_m = (
            self.calibration_factor * self.chain_pitch_mm * self.sprocket_teeth / 1000.0
        )

        # Latest input values
        self.left_dir: int = 0
        self.right_dir: int = 0
        self.left_rpm: float = 0.0
        self.right_rpm: float = 0.0
        self.left_revolutions: float = 0.0
        self.right_revolutions: float = 0.0
        self.imu_angular_z: float = 0.0

        # Timestamps for stale detection
        self.left_dir_time: Optional[float] = None
        self.right_dir_time: Optional[float] = None
        self.left_rpm_time: Optional[float] = None
        self.right_rpm_time: Optional[float] = None
        self.left_revolutions_time: Optional[float] = None
        self.right_revolutions_time: Optional[float] = None
        self.imu_time: Optional[float] = None

        # Integrated odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_update_time: Optional[float] = None
        self.last_left_revolutions: Optional[float] = None
        self.last_right_revolutions: Optional[float] = None

        # Cumulative odometry increments used only for smoothed twist in
        # pose_source="revolutions". Pose integration below still uses the
        # per-callback delta_s_odom / delta_theta_odom.
        self.total_s_odom = 0.0
        self.total_theta_odom = 0.0
        self.twist_history = deque()

        self.create_subscription(
            Int32,
            "/crawler/motor/left/dir",
            self.left_dir_callback,
            10,
        )
        self.create_subscription(
            Int32,
            "/crawler/motor/right/dir",
            self.right_dir_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/odometer/left/rpm",
            self.left_rpm_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/odometer/right/rpm",
            self.right_rpm_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/odometer/left/revolutions",
            self.left_revolutions_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/odometer/right/revolutions",
            self.right_revolutions_callback,
            10,
        )
        self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            10,
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        timer_period_s = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period_s, self.publish_odom)

        self.get_logger().info("crawler_odom started")
        self.get_logger().info(f"track_width_m: {self.track_width_m:.4f}")
        self.get_logger().info(f"chain_pitch_mm: {self.chain_pitch_mm:.2f}")
        self.get_logger().info(f"sprocket_teeth: {self.sprocket_teeth}")
        self.get_logger().info(
            f"distance_per_sprocket_rev_m: {self.distance_per_sprocket_rev_m:.5f}"
        )
        self.get_logger().info(f"imu_weight: {self.imu_weight:.2f}")
        self.get_logger().info(f"pose_source: {self.pose_source}")
        self.get_logger().info(
            f"twist_history_window_s: {self.twist_history_window_s:.2f}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def value_is_fresh(self, timestamp: Optional[float], now_s: float) -> bool:
        if timestamp is None:
            return False
        return (now_s - timestamp) <= self.stale_timeout_s

    def left_dir_callback(self, msg: Int32):
        self.left_dir = self.normalize_dir(msg.data) * self.left_dir_sign
        self.left_dir_time = self.now_s()

    def right_dir_callback(self, msg: Int32):
        self.right_dir = self.normalize_dir(msg.data) * self.right_dir_sign
        self.right_dir_time = self.now_s()

    def left_rpm_callback(self, msg: Float32):
        if math.isfinite(msg.data):
            self.left_rpm = abs(float(msg.data))
            self.left_rpm_time = self.now_s()

    def right_rpm_callback(self, msg: Float32):
        if math.isfinite(msg.data):
            self.right_rpm = abs(float(msg.data))
            self.right_rpm_time = self.now_s()

    def left_revolutions_callback(self, msg: Float32):
        if math.isfinite(msg.data):
            self.left_revolutions = float(msg.data)
            self.left_revolutions_time = self.now_s()

    def right_revolutions_callback(self, msg: Float32):
        if math.isfinite(msg.data):
            self.right_revolutions = float(msg.data)
            self.right_revolutions_time = self.now_s()

    def imu_callback(self, msg: Imu):
        angular_z = float(msg.angular_velocity.z)
        if math.isfinite(angular_z):
            self.imu_angular_z = self.imu_angular_z_sign * angular_z
            self.imu_time = self.now_s()

    @staticmethod
    def normalize_dir(value: int) -> int:
        if value > 0:
            return 1
        if value < 0:
            return -1
        return 0

    def rpm_to_track_speed_mps(self, rpm: float, direction: int) -> float:
        rev_per_s = rpm / 60.0
        speed = rev_per_s * self.distance_per_sprocket_rev_m
        return direction * speed

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        half_yaw = yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        return qz, qw

    def update_smoothed_revolutions_twist(
        self,
        now_s: float,
        delta_s_odom: float,
        delta_theta_odom: float,
    ):
        """Return smoothed (linear_x, angular_z) from revolution deltas.

        The revolution count changes only when a magnet peak is detected. At
        low speed that produces many callbacks with zero delta and occasional
        large delta steps. For pose this is fine, because the steps are
        accumulated. For twist it looks bad, so estimate velocity from the
        distance covered over a short history window.
        """
        self.total_s_odom += delta_s_odom
        self.total_theta_odom += delta_theta_odom

        self.twist_history.append(
            (now_s, self.total_s_odom, self.total_theta_odom)
        )

        while (
            len(self.twist_history) > 1
            and (now_s - self.twist_history[0][0]) > self.twist_history_window_s
        ):
            self.twist_history.popleft()

        if len(self.twist_history) < 2:
            return 0.0, 0.0

        old_t, old_s, old_theta = self.twist_history[0]
        dt_window = now_s - old_t
        if dt_window <= 0.0:
            return 0.0, 0.0

        linear_x = (self.total_s_odom - old_s) / dt_window
        angular_z = (self.total_theta_odom - old_theta) / dt_window
        return linear_x, angular_z

    def publish_odom(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        if self.last_update_time is None:
            self.last_update_time = now_s
            return

        dt = now_s - self.last_update_time
        self.last_update_time = now_s

        if dt <= 0.0 or dt > 1.0:
            return

        left_dir_ok = self.value_is_fresh(self.left_dir_time, now_s)
        right_dir_ok = self.value_is_fresh(self.right_dir_time, now_s)
        left_rpm_ok = self.value_is_fresh(self.left_rpm_time, now_s)
        right_rpm_ok = self.value_is_fresh(self.right_rpm_time, now_s)
        left_rev_ok = self.value_is_fresh(self.left_revolutions_time, now_s)
        right_rev_ok = self.value_is_fresh(self.right_revolutions_time, now_s)
        imu_ok = self.value_is_fresh(self.imu_time, now_s)

        rpm_odom_ok = left_dir_ok and right_dir_ok and left_rpm_ok and right_rpm_ok
        revolutions_available = left_dir_ok and right_dir_ok and left_rev_ok and right_rev_ok

        if rpm_odom_ok:
            v_left_rpm = self.rpm_to_track_speed_mps(self.left_rpm, self.left_dir)
            v_right_rpm = self.rpm_to_track_speed_mps(self.right_rpm, self.right_dir)
            linear_x_rpm = (v_right_rpm + v_left_rpm) * 0.5
            angular_z_rpm = (v_right_rpm - v_left_rpm) / self.track_width_m
            angular_z_rpm *= self.odom_angular_z_sign
        else:
            v_left_rpm = 0.0
            v_right_rpm = 0.0
            linear_x_rpm = 0.0
            angular_z_rpm = 0.0

        if self.pose_source == "revolutions":
            odom_ok = revolutions_available

            if odom_ok and (
                self.last_left_revolutions is None
                or self.last_right_revolutions is None
            ):
                self.last_left_revolutions = self.left_revolutions
                self.last_right_revolutions = self.right_revolutions
                odom_ok = False

            if odom_ok:
                delta_left_rev = self.left_revolutions - self.last_left_revolutions
                delta_right_rev = self.right_revolutions - self.last_right_revolutions

                self.last_left_revolutions = self.left_revolutions
                self.last_right_revolutions = self.right_revolutions

                # Revolutions only count magnitude. The motor direction topics
                # supply the sign for forward/reverse travel.
                delta_left_m = (
                    delta_left_rev
                    * self.distance_per_sprocket_rev_m
                    * self.left_dir
                )
                delta_right_m = (
                    delta_right_rev
                    * self.distance_per_sprocket_rev_m
                    * self.right_dir
                )

                delta_s_odom = (delta_right_m + delta_left_m) * 0.5
                delta_theta_odom = (delta_right_m - delta_left_m) / self.track_width_m
                delta_theta_odom *= self.odom_angular_z_sign
                delta_theta_odom *= self.calibration_factor_angular

                linear_x, angular_z_odom = self.update_smoothed_revolutions_twist(
                    now_s=now_s,
                    delta_s_odom=delta_s_odom,
                    delta_theta_odom=delta_theta_odom,
                )
            else:
                delta_s_odom = 0.0
                delta_theta_odom = 0.0
                linear_x = 0.0
                angular_z_odom = 0.0
                self.twist_history.clear()

        else:
            odom_ok = rpm_odom_ok
            self.twist_history.clear()
            linear_x = linear_x_rpm
            angular_z_odom = angular_z_rpm
            delta_s_odom = linear_x * dt
            delta_theta_odom = angular_z_odom * dt

        if odom_ok and imu_ok:
            angular_z = (
                (1.0 - self.imu_weight) * angular_z_odom
                + self.imu_weight * self.imu_angular_z
            )
        elif odom_ok:
            angular_z = angular_z_odom
        elif imu_ok:
            # IMU-only fallback: only angular velocity is trusted.
            linear_x = 0.0
            delta_s_odom = 0.0
            angular_z = self.imu_angular_z
        else:
            linear_x = 0.0
            delta_s_odom = 0.0
            angular_z = 0.0

        # Integrate pose. Use the midpoint heading for the translation step.
        delta_theta = angular_z * dt
        theta_mid = self.theta + 0.5 * delta_theta

        self.x += delta_s_odom * math.cos(theta_mid)
        self.y += delta_s_odom * math.sin(theta_mid)

        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        qz, qw = self.yaw_to_quaternion(self.theta)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = linear_x
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_z

        # Rough covariance values.
        # These can be tuned later after real tests.
        msg.pose.covariance[0] = 0.05      # x
        msg.pose.covariance[7] = 0.05      # y
        msg.pose.covariance[14] = 99999.0  # z
        msg.pose.covariance[21] = 99999.0  # roll
        msg.pose.covariance[28] = 99999.0  # pitch
        msg.pose.covariance[35] = 0.10     # yaw

        msg.twist.covariance[0] = 0.05      # vx
        msg.twist.covariance[7] = 99999.0   # vy
        msg.twist.covariance[14] = 99999.0  # vz
        msg.twist.covariance[21] = 99999.0  # wx
        msg.twist.covariance[28] = 99999.0  # wy
        msg.twist.covariance[35] = 0.10     # wz

        self.odom_pub.publish(msg)

        # Broadcast transform odom -> base_link via tf2.
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CrawlerOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()