#!/usr/bin/env python3

import math
import subprocess
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


CURRENT_DATA_TIMEOUT_S = 0.5
STATUS_PUBLISH_PERIOD_S = 0.05

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
        self.last_current_time = None
        self.status_publisher = self.create_publisher(Bool, "/crawler/safety/current_ok", 10)
        self.status_timer = self.create_timer(STATUS_PUBLISH_PERIOD_S, self.publish_safety_status)

        self.get_logger().info("Starting crawler current guard")
        self.get_logger().info(
            f"Warning at {WARN_CURRENT_A:.1f} A, trip at {TRIP_CURRENT_A:.1f} A "
            f"for {TRIP_TIME_S:.2f} s"
        )

        self.subscription = self.create_subscription(
            Float32,
            "/battery/current",
            self.current_callback,
            10,
        )

    def publish_safety_status(self):
        """Fail closed until fresh current samples arrive; latch on overcurrent."""
        fresh = (
            self.last_current_time is not None
            and time.monotonic() - self.last_current_time <= CURRENT_DATA_TIMEOUT_S
        )
        msg = Bool()
        msg.data = bool(fresh and not self.fault_latched)
        self.status_publisher.publish(msg)

    def announce_fault(self, current_a: float, avg_a: float):
        message = (
            f"CRAWLER SAFETY TRIP: {avg_a:.2f} A average "
            f"({current_a:.2f} A latest) >= {TRIP_CURRENT_A:.1f} A "
            "-> current_ok=False (latched); motor driver disables outputs"
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
        if not math.isfinite(current_a):
            self.fault_latched = True
            self.get_logger().error("Non-finite current measurement: safety trip latched")
            self.publish_safety_status()
            return
        self.last_current_time = now
        if USE_ABSOLUTE_CURRENT:
            current_a = abs(current_a)

        self.samples.append((now, current_a))

        while self.samples and now - self.samples[0][0] > WINDOW_S:
            self.samples.popleft()

        avg_a = sum(value for _, value in self.samples) / len(self.samples)

        if self.fault_latched:
            self.publish_safety_status()
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
                self.announce_fault(current_a, avg_a)
                self.publish_safety_status()
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
        # No GPIO ownership: stopping this process makes current_ok stale.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
