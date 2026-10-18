import threading

import gradio as gr
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
import warnings

warnings.filterwarnings(
    "ignore",
    message=".*HTTP_422_UNPROCESSABLE_ENTITY.*",
    category=Warning,
)


COMM_RATE = 0.1  # Publish rate in seconds
CRAWLER_SOFT_V_MAX = 0.18  # v_max crawler in m/s, linear.x max
CRAWLER_TRACK_WIDTH = 0.245
CRAWLER_SOFT_OMEGA_MAX = round(
    2 * CRAWLER_SOFT_V_MAX / CRAWLER_TRACK_WIDTH,
    2,
)  # Max angular speed [rad/s]


class TeleopNode(Node):
    def __init__(self):
        super().__init__("crawler_teleop_node")

        self.lock = threading.Lock()

        self.publish_enabled = False

        self.cmd_vel = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0),
        )

        self.odom_linear_x = 0.0
        self.odom_angular_z = 0.0

        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 16)

        self.cmd_vel_publish_timer = self.create_timer(
            COMM_RATE,
            self.publish_cmd_vel,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        with gr.Blocks(title="Crawler Teleop") as self.web_iface:
            gr.Markdown("# Crawler Teleop")

            status = gr.Textbox(label="Status", interactive=False)

            publish_checkbox = gr.Checkbox(
                value=False,
                label="Enable publishing to /cmd_vel",
            )

            linear_slider = gr.Slider(
                minimum=-CRAWLER_SOFT_V_MAX,
                maximum=CRAWLER_SOFT_V_MAX,
                value=0.0,
                step=0.01,
                label="cmd_vel linear.x [m/s]",
            )

            angular_slider = gr.Slider(
                minimum=-CRAWLER_SOFT_OMEGA_MAX,
                maximum=CRAWLER_SOFT_OMEGA_MAX,
                value=0.0,
                step=0.05,
                label="cmd_vel angular.z [rad/s]",
            )

            stop_button = gr.Button("STOP")

            gr.Markdown("## Live feedback from /odom")

            with gr.Row():
                odom_linear_box = gr.Number(
                    label="/odom linear.x [m/s]",
                    value=0.0,
                    precision=3,
                    interactive=False,
                )

                odom_angular_box = gr.Number(
                    label="/odom angular.z [rad/s]",
                    value=0.0,
                    precision=3,
                    interactive=False,
                )

            with gr.Row():
                linear_error_box = gr.Number(
                    label="linear error: cmd - odom [m/s]",
                    value=0.0,
                    precision=3,
                    interactive=False,
                )

                angular_error_box = gr.Number(
                    label="angular error: cmd - odom [rad/s]",
                    value=0.0,
                    precision=3,
                    interactive=False,
                )

            publish_checkbox.change(
                fn=self.set_publish_enabled,
                inputs=publish_checkbox,
                outputs=status,
            )

            linear_slider.change(
                fn=self.set_cmd_vel,
                inputs=[linear_slider, angular_slider],
                outputs=status,
            )

            angular_slider.change(
                fn=self.set_cmd_vel,
                inputs=[linear_slider, angular_slider],
                outputs=status,
            )

            stop_button.click(
                fn=self.stop,
                inputs=[],
                outputs=[linear_slider, angular_slider, status],
            )

            odom_timer = gr.Timer(value=0.2)

            odom_timer.tick(
                fn=self.get_live_feedback,
                inputs=[],
                outputs=[
                    odom_linear_box,
                    odom_angular_box,
                    linear_error_box,
                    angular_error_box,
                ],
            )

        self.web_iface.launch(
            server_name="0.0.0.0",
            server_port=7860,
            prevent_thread_lock=True,
        )

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.odom_linear_x = msg.twist.twist.linear.x
            self.odom_angular_z = msg.twist.twist.angular.z

    def get_live_feedback(self):
        with self.lock:
            cmd_linear_x = self.cmd_vel.linear.x
            cmd_angular_z = self.cmd_vel.angular.z
            odom_linear_x = self.odom_linear_x
            odom_angular_z = self.odom_angular_z

        linear_error = cmd_linear_x - odom_linear_x
        angular_error = cmd_angular_z - odom_angular_z

        return (
            round(odom_linear_x, 3),
            round(odom_angular_z, 3),
            round(linear_error, 3),
            round(angular_error, 3),
        )

    def set_publish_enabled(self, enabled):
        with self.lock:
            self.publish_enabled = bool(enabled)

        if not self.publish_enabled:
            self.publish_stop()
            return "Publishing disabled - STOP sent"

        return "Publishing enabled"

    def set_cmd_vel(self, linear_x, angular_z):
        with self.lock:
            self.cmd_vel.linear.x = float(linear_x)
            self.cmd_vel.angular.z = float(angular_z)

            current_linear_x = self.cmd_vel.linear.x
            current_angular_z = self.cmd_vel.angular.z
            current_publish_enabled = self.publish_enabled

        return (
            f"cmd_vel set: linear.x={current_linear_x:.3f} m/s, "
            f"angular.z={current_angular_z:.3f} rad/s, "
            f"publishing={current_publish_enabled}"
        )

    def publish_cmd_vel(self):
        with self.lock:
            if not self.publish_enabled:
                return

            msg = Twist(
                linear=Vector3(
                    x=self.cmd_vel.linear.x,
                    y=self.cmd_vel.linear.y,
                    z=self.cmd_vel.linear.z,
                ),
                angular=Vector3(
                    x=self.cmd_vel.angular.x,
                    y=self.cmd_vel.angular.y,
                    z=self.cmd_vel.angular.z,
                ),
            )

        self.cmd_vel_publisher.publish(msg)

    def stop(self):
        self.publish_stop()
        return 0.0, 0.0, "STOP"

    def publish_stop(self):
        stop_msg = Twist()

        with self.lock:
            self.cmd_vel = stop_msg

        self.cmd_vel_publisher.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.publish_stop()
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()