#!/usr/bin/env python3

import subprocess
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


MOTOR_EN_GPIO = 5

WARN_CURRENT_A = 6.0
TRIP_CURRENT_A = 9.0

# Current must stay above the trip threshold for this long.
TRIP_TIME_S = 0.15

# Moving average window.
WINDOW_S = 0.30

# Use absolute current. This avoids trouble if INA226 polarity is reversed later.
USE_ABSOLUTE_CURRENT = True


class CurrentGuardNode(Node):
    def __init__(self):
        super().__init__("current_guard_node")

        self.samples = deque()
        self.over_start = None
        self.fault_latched = False
        self.last_warn_time = 0.0

        self.get_logger().info("Starting crawler current guard")
        self.get_logger().info(f"MOTOR_EN GPIO: {MOTOR_EN_GPIO}")
        self.get_logger().info(
            f"Warning at {WARN_CURRENT_A:.1f} A, trip at {TRIP_CURRENT_A:.1f} A "
            f"for {TRIP_TIME_S:.2f} s"
        )

        self.set_motor_enable(True)

        self.subscription = self.create_subscription(
            Float32,
            "/battery/current",
            self.current_callback,
            10,
        )

    def set_motor_enable(self, enabled: bool):
        value = "1" if enabled else "0"

        result = subprocess.run(
            ["gpioset", "gpiochip0", f"{MOTOR_EN_GPIO}={value}"],
            text=True,
            capture_output=True,
        )

        if result.returncode != 0:
            self.get_logger().error(
                f"Failed to set MOTOR_EN GPIO {MOTOR_EN_GPIO} to {value}: "
                f"{result.stderr.strip()}"
            )

    def announce_fault(self, current_a: float, avg_a: float):
        message = (
            f"CRAWLER SAFETY TRIP: {avg_a:.2f} A average "
            f"({current_a:.2f} A latest) >= {TRIP_CURRENT_A:.1f} A "
            f"-> MOTOR_EN GPIO {MOTOR_EN_GPIO}=0 -> motors disabled"
        )

        self.get_logger().error(message)

        subprocess.run(
            ["logger", "-p", "daemon.err", message],
            check=False,
        )

        subprocess.run(
            ["wall", message],
            check=False,
        )

    def current_callback(self, msg: Float32):
        now = time.monotonic()

        current_a = float(msg.data)
        if USE_ABSOLUTE_CURRENT:
            current_a = abs(current_a)

        self.samples.append((now, current_a))

        while self.samples and now - self.samples[0][0] > WINDOW_S:
            self.samples.popleft()

        avg_a = sum(value for _, value in self.samples) / len(self.samples)

        if self.fault_latched:
            self.set_motor_enable(False)
            return

        # Rate-limit warning spam.
        if avg_a >= WARN_CURRENT_A and now - self.last_warn_time > 1.0:
            self.last_warn_time = now
            self.get_logger().warn(
                f"High current: latest={current_a:.2f} A, avg={avg_a:.2f} A"
            )

        if avg_a >= TRIP_CURRENT_A:
            if self.over_start is None:
                self.over_start = now

            if now - self.over_start >= TRIP_TIME_S:
                self.fault_latched = True
                self.set_motor_enable(False)
                self.announce_fault(current_a, avg_a)
        else:
            self.over_start = None


def main(args=None):
    rclpy.init(args=args)

    node = CurrentGuardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Fail-safe: disable motors when this safety node exits.
        node.set_motor_enable(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
