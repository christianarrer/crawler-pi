import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


CRAWLER_HARD_V_MAX = 0.18  # Hardware speed limit in m/s
CRAWLER_DEADZONE_V = 0.005
CRAWLER_TRACK_WIDTH = 0.245  # Effective distance between tracks in m

SPROCKET_TEETH = 12
CHAIN_PITCH_M = 0.011
ODOM_CALIBRATION_FACTOR = 1.17

# Calibration linear regression parameters.
# Formula: duty_fraction_abs = A * rpm_abs + B
# Use +duty for forward and -duty for reverse.
LEFT_FWD_A = 0.01362320
LEFT_FWD_B = 0.08028112
LEFT_REV_A = 0.01348750
LEFT_REV_B = 0.08261412

RIGHT_FWD_A = 0.01374704
RIGHT_FWD_B = 0.07573920
RIGHT_REV_A = 0.01377946
RIGHT_REV_B = 0.07894751


class VelocityTargetNode(Node):
    def __init__(self):
        super().__init__("crawler_velocity_target")

        self.declare_parameter("hard_v_max_m_s", 0.18)
        self.declare_parameter("deadzone_v_m_s", 0.005)
        self.declare_parameter("track_width_m", 0.245)
        self.declare_parameter("sprocket_teeth", 12)
        self.declare_parameter("chain_pitch_m", 0.011)
        self.declare_parameter("odom_calibration_factor", 1.17)
        self.declare_parameter("left_fwd_a", 0.0136232)
        self.declare_parameter("left_fwd_b", 0.08028112)
        self.declare_parameter("left_rev_a", 0.0134875)
        self.declare_parameter("left_rev_b", 0.08261412)
        self.declare_parameter("right_fwd_a", 0.01374704)
        self.declare_parameter("right_fwd_b", 0.0757392)
        self.declare_parameter("right_rev_a", 0.01377946)
        self.declare_parameter("right_rev_b", 0.07894751)

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            "cmd_vel",
            self.read_cmd_vel,
            16,
        )

        self.left_target_publisher = self.create_publisher(
            Float32,
            "crawler/motor/left/target",
            16,
        )

        self.right_target_publisher = self.create_publisher(
            Float32,
            "crawler/motor/right/target",
            16,
        )

    def read_cmd_vel(self, msg):
        v_linear = float(msg.linear.x)
        omega = float(msg.angular.z)

        v_track_left = v_linear - omega * (self.get_parameter("track_width_m").value / 2.0)
        v_track_right = v_linear + omega * (self.get_parameter("track_width_m").value / 2.0)

        v_track_left, v_track_right = self.scale_track_velocities(
            v_track_left,
            v_track_right,
        )

        # Deadzone must be applied before converting velocity to PWM target.
        # Otherwise tiny non-zero velocities still get the regression offset B.
        v_track_left = self.apply_deadzone(v_track_left)
        v_track_right = self.apply_deadzone(v_track_right)

        left_target = self.velocity_to_left_target(v_track_left)
        right_target = self.velocity_to_right_target(v_track_right)

        left_target = self.clamp_target(left_target)
        right_target = self.clamp_target(right_target)

        self.left_target_publisher.publish(Float32(data=left_target))
        self.right_target_publisher.publish(Float32(data=right_target))

    def scale_track_velocities(self, left, right):
        max_abs = max(abs(left), abs(right))

        if max_abs <= self.get_parameter("hard_v_max_m_s").value:
            return left, right

        scale = self.get_parameter("hard_v_max_m_s").value / max_abs
        return left * scale, right * scale

    def speed_to_rpm_abs(self, v_abs_m_s: float) -> float:
        distance_per_rev_m = self.get_parameter("sprocket_teeth").value * self.get_parameter("chain_pitch_m").value * self.get_parameter("odom_calibration_factor").value
        return v_abs_m_s * 60.0 / distance_per_rev_m

    def velocity_to_left_target(self, velocity):
        if velocity > 0.0:
            rpm_abs = self.speed_to_rpm_abs(abs(velocity))
            duty_abs = self.get_parameter("left_fwd_a").value * rpm_abs + self.get_parameter("left_fwd_b").value
            return duty_abs

        if velocity < 0.0:
            rpm_abs = self.speed_to_rpm_abs(abs(velocity))
            duty_abs = self.get_parameter("left_rev_a").value * rpm_abs + self.get_parameter("left_rev_b").value
            return -duty_abs

        return 0.0

    def velocity_to_right_target(self, velocity):
        if velocity > 0.0:
            rpm_abs = self.speed_to_rpm_abs(abs(velocity))
            duty_abs = self.get_parameter("right_fwd_a").value * rpm_abs + self.get_parameter("right_fwd_b").value
            return duty_abs

        if velocity < 0.0:
            rpm_abs = self.speed_to_rpm_abs(abs(velocity))
            duty_abs = self.get_parameter("right_rev_a").value * rpm_abs + self.get_parameter("right_rev_b").value
            return -duty_abs

        return 0.0

    def clamp_target(self, target):
        if target > 1.0:
            return 1.0

        if target < -1.0:
            return -1.0

        return target

    def apply_deadzone(self, velocity):
        if abs(velocity) < self.get_parameter("deadzone_v_m_s").value:
            return 0.0

        return velocity

    def publish_stop(self):
        self.left_target_publisher.publish(Float32(data=0.0))
        self.right_target_publisher.publish(Float32(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    velocity_target_node = VelocityTargetNode()

    try:
        rclpy.spin(velocity_target_node)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_target_node.publish_stop()
        velocity_target_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
