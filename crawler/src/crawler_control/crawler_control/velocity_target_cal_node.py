#!/usr/bin/env python3

"""
Calibrate crawler motor target -> RPM using the normal ROS motor driver path.

This node publishes Float32 targets to:
  /crawler/motor/left/target
  /crawler/motor/right/target

and reads odometer topics:
  /odometer/left/rpm
  /odometer/left/revolutions
  /odometer/right/rpm
  /odometer/right/revolutions

The regression printed at the end is:
  duty_fraction_abs = A * rpm_abs + B

For the velocity_target node this means roughly:
  rpm_abs = abs(v_m_per_s) * 60 / (sprocket_teeth * chain_pitch_m * calibration_factor)
  duty_abs = A * rpm_abs + B
  target = +duty_abs for forward, -duty_abs for reverse

Default geometry is only used for the CSV speed column, not for the regression.
"""

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float32


# Measurement behavior
WARMUP_SECONDS = 3.0
MEASURE_SECONDS = 30.0
SAMPLE_SLEEP_SECONDS = 0.1
STOP_SETTLE_SECONDS = 1.0

# Same ranges as the old calibration script
PERCENT_VALUES = {
    "fwd": range(20, 105, 5),
    "rev": range(30, 105, 5),
}
DIRECTIONS = ["fwd", "rev"]
MOTORS = ["left", "right"]

# Geometry for the informational speed column in the CSV
CHAIN_PITCH_MM = 11.0
SPROCKET_TEETH = 12
DISTANCE_PER_SPROCKET_REV_M = (CHAIN_PITCH_MM * SPROCKET_TEETH) / 1000.0

CSV_FILE = "crawler-velocity-target-characterization.csv"


@dataclass
class TopicValue:
    value: Optional[float] = None
    stamp: Optional[float] = None


class VelocityTargetCalNode(Node):
    def __init__(self) -> None:
        super().__init__("velocity_target_cal_node")

        self.values: Dict[str, TopicValue] = {}

        self.left_rpm = self.add_data_topic("/odometer/left/rpm")
        self.left_revolutions = self.add_data_topic("/odometer/left/revolutions")
        self.right_rpm = self.add_data_topic("/odometer/right/rpm")
        self.right_revolutions = self.add_data_topic("/odometer/right/revolutions")

        self.left_target_pub = self.create_publisher(
            Float32, "/crawler/motor/left/target", 10
        )
        self.right_target_pub = self.create_publisher(
            Float32, "/crawler/motor/right/target", 10
        )

    def resolve_topic_type(self, topic_name: str) -> str:
        deadline = time.time() + 10.0

        while time.time() < deadline:
            topic_infos = dict(self.get_topic_names_and_types())
            if topic_name in topic_infos and topic_infos[topic_name]:
                return topic_infos[topic_name][0]
            rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError(f"Could not resolve ROS type for topic: {topic_name}")

    def add_data_topic(self, topic_name: str) -> TopicValue:
        topic_type = self.resolve_topic_type(topic_name)
        msg_class = get_message(topic_type)

        value_holder = TopicValue()
        self.values[topic_name] = value_holder

        def callback(msg) -> None:
            value_holder.value = float(msg.data)
            value_holder.stamp = time.time()

        self.create_subscription(msg_class, topic_name, callback, 10)
        self.get_logger().info(f"Subscribed to {topic_name} as {topic_type}")
        return value_holder

    def publish_targets(self, left: float, right: float) -> None:
        left = max(-1.0, min(1.0, float(left)))
        right = max(-1.0, min(1.0, float(right)))

        self.left_target_pub.publish(Float32(data=left))
        self.right_target_pub.publish(Float32(data=right))

    def stop_motors(self) -> None:
        # Publish stop more than once so the motor driver reliably receives it.
        for _ in range(5):
            self.publish_targets(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)


def spin_for(node: VelocityTargetCalNode, seconds: float) -> None:
    end_time = time.time() + seconds
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)


def wait_for_initial_data(node: VelocityTargetCalNode) -> None:
    print("Waiting for odometer topics...")

    deadline = time.time() + 10.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (
            node.left_rpm.value is not None
            and node.left_revolutions.value is not None
            and node.right_rpm.value is not None
            and node.right_revolutions.value is not None
        ):
            print("All odometer topics received.")
            return

    raise RuntimeError("Timeout while waiting for odometer topics.")


def select_odom_values(
    node: VelocityTargetCalNode, motor_side: str
) -> Tuple[TopicValue, TopicValue]:
    if motor_side == "left":
        return node.left_rpm, node.left_revolutions
    if motor_side == "right":
        return node.right_rpm, node.right_revolutions
    raise ValueError(f"Invalid motor side: {motor_side}")


def target_for_case(motor_side: str, direction: str, percent: int) -> Tuple[float, float]:
    duty_abs = percent / 100.0
    signed_target = duty_abs if direction == "fwd" else -duty_abs

    if motor_side == "left":
        return signed_target, 0.0
    if motor_side == "right":
        return 0.0, signed_target
    raise ValueError(f"Invalid motor side: {motor_side}")


def measure_case(
    node: VelocityTargetCalNode, motor_side: str, direction: str, percent: int
) -> Dict[str, float]:
    rpm_value, rev_value = select_odom_values(node, motor_side)
    left_target, right_target = target_for_case(motor_side, direction, percent)

    duty_fraction_abs = percent / 100.0
    commanded_sign = 1.0 if direction == "fwd" else -1.0

    print(f"Command target: left={left_target:+.3f}, right={right_target:+.3f}")

    node.publish_targets(left_target, right_target)
    print(f"Warmup {WARMUP_SECONDS:.1f}s...")
    spin_for(node, WARMUP_SECONDS)

    start_rev = rev_value.value
    if start_rev is None:
        raise RuntimeError(f"No revolution value for {motor_side}")

    rpm_sum = 0.0
    abs_rpm_sum = 0.0
    samples = 0
    min_rpm: Optional[float] = None
    max_rpm: Optional[float] = None

    start_time = time.time()
    end_time = start_time + MEASURE_SECONDS
    current_rev = start_rev

    print(f"Start revolutions: {start_rev:.3f}")
    print(f"Measuring for:     {MEASURE_SECONDS:.1f}s")

    while time.time() < end_time:
        # Keep publishing the target. This makes the calibration independent of
        # whether the motor_driver expects repeated commands or latches the last one.
        node.publish_targets(left_target, right_target)
        rclpy.spin_once(node, timeout_sec=SAMPLE_SLEEP_SECONDS)

        current_rpm = rpm_value.value
        current_rev = rev_value.value

        if current_rpm is None or current_rev is None:
            continue

        rpm_sum += current_rpm
        abs_rpm_sum += abs(current_rpm)
        samples += 1

        min_rpm = current_rpm if min_rpm is None else min(min_rpm, current_rpm)
        max_rpm = current_rpm if max_rpm is None else max(max_rpm, current_rpm)

        elapsed = time.time() - start_time
        delta_rev = current_rev - start_rev
        print(
            f"{motor_side:5s} {direction:3s} {percent:3d}% "
            f"t={elapsed:5.1f}s rev={current_rev:8.3f} "
            f"delta={delta_rev:8.3f} rpm={current_rpm:8.3f}"
        )

    elapsed = time.time() - start_time
    end_rev = current_rev
    delta_rev = end_rev - start_rev

    if samples > 0:
        avg_rpm = rpm_sum / samples
        avg_abs_rpm = abs_rpm_sum / samples
        min_rpm_value = min_rpm if min_rpm is not None else math.nan
        max_rpm_value = max_rpm if max_rpm is not None else math.nan
    else:
        avg_rpm = math.nan
        avg_abs_rpm = math.nan
        min_rpm_value = math.nan
        max_rpm_value = math.nan

    speed_m_per_s_signed_from_abs = (
        commanded_sign * DISTANCE_PER_SPROCKET_REV_M * avg_abs_rpm / 60.0
    )
    speed_m_per_s_from_signed_rpm = DISTANCE_PER_SPROCKET_REV_M * avg_rpm / 60.0

    node.stop_motors()
    spin_for(node, STOP_SETTLE_SECONDS)

    return {
        "motor": motor_side,
        "direction": direction,
        "motor_percent": percent,
        "target_value": left_target if motor_side == "left" else right_target,
        "duty_fraction_abs": duty_fraction_abs,
        "avg_rpm": avg_rpm,
        "avg_abs_rpm": avg_abs_rpm,
        "min_rpm": min_rpm_value,
        "max_rpm": max_rpm_value,
        "samples": samples,
        "speed_m_per_s_signed_from_abs": speed_m_per_s_signed_from_abs,
        "speed_m_per_s_from_signed_rpm": speed_m_per_s_from_signed_rpm,
        "start_revolutions": start_rev,
        "end_revolutions": end_rev,
        "delta_revolutions": delta_rev,
        "elapsed_seconds": elapsed,
    }


def linear_regression(points: Iterable[Tuple[float, float]]) -> Optional[Tuple[float, float, float, int]]:
    """Return A, B, R^2, n for y = A*x + B."""
    clean_points = [
        (float(x), float(y))
        for x, y in points
        if math.isfinite(float(x)) and math.isfinite(float(y))
    ]
    n = len(clean_points)
    if n < 2:
        return None

    sum_x = sum(x for x, _ in clean_points)
    sum_y = sum(y for _, y in clean_points)
    sum_xx = sum(x * x for x, _ in clean_points)
    sum_xy = sum(x * y for x, y in clean_points)

    denom = n * sum_xx - sum_x * sum_x
    if abs(denom) < 1e-12:
        return None

    a = (n * sum_xy - sum_x * sum_y) / denom
    b = (sum_y - a * sum_x) / n

    mean_y = sum_y / n
    ss_tot = sum((y - mean_y) ** 2 for _, y in clean_points)
    ss_res = sum((y - (a * x + b)) ** 2 for x, y in clean_points)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else math.nan

    return a, b, r2, n


def print_regressions(results: List[Dict[str, float]]) -> None:
    print("\n" + "=" * 70)
    print("Regression: duty_fraction_abs = A * avg_abs_rpm + B")
    print("Use +duty for forward and -duty for reverse.")
    print("=" * 70)

    for motor in MOTORS:
        for direction in DIRECTIONS:
            subset = [
                row
                for row in results
                if row["motor"] == motor and row["direction"] == direction
            ]
            reg = linear_regression(
                (row["avg_abs_rpm"], row["duty_fraction_abs"]) for row in subset
            )
            label = f"{motor.upper()}_{direction.upper()}"
            if reg is None:
                print(f"{label}: not enough valid points")
                continue

            a, b, r2, n = reg
            print(f"{label}_A = {a:.8f}")
            print(f"{label}_B = {b:.8f}")
            print(f"{label}_R2 = {r2:.6f}   # n={n}")
            print(f"# duty_abs = {a:.8f} * rpm_abs + {b:.8f}")

            if direction == "fwd":
                print(f"# target = +duty_abs")
            else:
                print(f"# target = -duty_abs")
            print()

    print("Geometry currently used only for CSV speed column:")
    print(f"SPROCKET_TEETH = {SPROCKET_TEETH}")
    print(f"CHAIN_PITCH_MM = {CHAIN_PITCH_MM}")
    print(f"DISTANCE_PER_SPROCKET_REV_M = {DISTANCE_PER_SPROCKET_REV_M:.6f}")
    print("For calibrated velocity later, put your 1.17 factor into the velocity->rpm conversion.")


def main() -> None:
    rclpy.init()
    node = VelocityTargetCalNode()
    csv_path = Path(CSV_FILE)

    fieldnames = [
        "motor",
        "direction",
        "motor_percent",
        "target_value",
        "duty_fraction_abs",
        "avg_rpm",
        "avg_abs_rpm",
        "min_rpm",
        "max_rpm",
        "samples",
        "speed_m_per_s_signed_from_abs",
        "speed_m_per_s_from_signed_rpm",
        "start_revolutions",
        "end_revolutions",
        "delta_revolutions",
        "elapsed_seconds",
    ]

    results: List[Dict[str, float]] = []

    try:
        wait_for_initial_data(node)
        node.stop_motors()

        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=";")
            writer.writeheader()

            for motor_side in MOTORS:
                for direction in DIRECTIONS:
                    for percent in PERCENT_VALUES[direction]:
                        print("=" * 70)
                        print(
                            f"Measuring motor={motor_side}, "
                            f"direction={direction}, percent={percent}"
                        )
                        print("=" * 70)

                        result = measure_case(node, motor_side, direction, percent)
                        results.append(result)
                        writer.writerow(result)
                        f.flush()

                        print(
                            f"RESULT {motor_side} {direction} {percent}%: "
                            f"avg_rpm={result['avg_rpm']:.3f}, "
                            f"avg_abs_rpm={result['avg_abs_rpm']:.3f}, "
                            f"speed_abs_signed={result['speed_m_per_s_signed_from_abs']:.4f} m/s, "
                            f"samples={result['samples']}"
                        )

                        time.sleep(1.0)

        print_regressions(results)

    finally:
        print("Stopping motors...")
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

    print(f"Done. Results written to: {csv_path}")


if __name__ == "__main__":
    main()
