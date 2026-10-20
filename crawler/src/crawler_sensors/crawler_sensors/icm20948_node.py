#!/usr/bin/env python3

import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Header

try:
    from smbus2 import SMBus
except ImportError as exc:
    raise ImportError(
        "Missing Python package smbus2. Install with: sudo apt install python3-smbus2"
    ) from exc


class Icm20948RawNode(Node):
    """Minimal ROS2 IMU node for ICM-20948 over I2C.

    This node publishes raw accelerometer and gyroscope data on /imu.
    The ICM-20948 does not provide a fused quaternion like the BNO086, so
    orientation is intentionally marked as unavailable according to
    sensor_msgs/Imu convention: orientation_covariance[0] = -1.
    """

    # ICM-20948 common registers
    REG_BANK_SEL = 0x7F

    # Bank 0
    WHO_AM_I = 0x00
    USER_CTRL = 0x03
    LP_CONFIG = 0x05
    PWR_MGMT_1 = 0x06
    PWR_MGMT_2 = 0x07
    ACCEL_XOUT_H = 0x2D
    WHO_AM_I_EXPECTED = 0xEA

    # Bank 2
    GYRO_SMPLRT_DIV = 0x00
    GYRO_CONFIG_1 = 0x01
    ACCEL_SMPLRT_DIV_1 = 0x10
    ACCEL_SMPLRT_DIV_2 = 0x11
    ACCEL_CONFIG = 0x14

    # Full-scale sensitivities
    ACCEL_SCALE_FACTORS = {
        2: 16384.0,
        4: 8192.0,
        8: 4096.0,
        16: 2048.0,
    }  # LSB/g

    GYRO_SCALE_FACTORS = {
        250: 131.0,
        500: 65.5,
        1000: 32.8,
        2000: 16.4,
    }  # LSB/(deg/s)

    ACCEL_FS_BITS = {2: 0b00, 4: 0b01, 8: 0b10, 16: 0b11}
    GYRO_FS_BITS = {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}

    def __init__(self):
        super().__init__("icm20948_node")

        self.declare_parameter("bus_number", 1)
        self.declare_parameter("address", 0x69)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("accel_range_g", 4)
        self.declare_parameter("gyro_range_dps", 500)
        self.declare_parameter("max_consecutive_errors", 5)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("invert_z", False)
        # Gyroscope zero-rate calibration (performed while the robot is stationary).
        self.declare_parameter("gyro_calibration_samples", 500)
        self.declare_parameter("gyro_calibration_settle_s", 2.0)
        self.declare_parameter("gyro_calibration_max_stddev_dps", 0.20)

        self.bus_number = int(self.get_parameter("bus_number").value)
        self.address = int(self.get_parameter("address").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.accel_range_g = int(self.get_parameter("accel_range_g").value)
        self.gyro_range_dps = int(self.get_parameter("gyro_range_dps").value)
        self.max_consecutive_errors = int(self.get_parameter("max_consecutive_errors").value)
        self.reconnect_delay_s = float(self.get_parameter("reconnect_delay_s").value)
        self.swap_xy = bool(self.get_parameter("swap_xy").value)
        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)
        self.invert_z = bool(self.get_parameter("invert_z").value)
        self.gyro_calibration_samples = int(self.get_parameter("gyro_calibration_samples").value)
        self.gyro_calibration_settle_s = float(self.get_parameter("gyro_calibration_settle_s").value)
        self.gyro_calibration_max_stddev_dps = float(
            self.get_parameter("gyro_calibration_max_stddev_dps").value
        )

        if self.accel_range_g not in self.ACCEL_SCALE_FACTORS:
            raise ValueError("accel_range_g must be one of: 2, 4, 8, 16")
        if self.gyro_range_dps not in self.GYRO_SCALE_FACTORS:
            raise ValueError("gyro_range_dps must be one of: 250, 500, 1000, 2000")
        if self.publish_rate_hz <= 0:
            raise ValueError("publish_rate_hz must be > 0")

        if self.gyro_calibration_samples < 2:
            raise ValueError("gyro_calibration_samples must be >= 2")
        if self.gyro_calibration_settle_s < 0:
            raise ValueError("gyro_calibration_settle_s must be >= 0")
        if self.gyro_calibration_max_stddev_dps <= 0:
            raise ValueError("gyro_calibration_max_stddev_dps must be > 0")

        self.accel_lsb_per_g = self.ACCEL_SCALE_FACTORS[self.accel_range_g]
        self.gyro_lsb_per_dps = self.GYRO_SCALE_FACTORS[self.gyro_range_dps]

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.imu_pub = self.create_publisher(Imu, "/imu", qos)

        self.bus: Optional[SMBus] = None
        self.consecutive_errors = 0
        self.last_reconnect_attempt = 0.0
        self.gyro_offset = (0.0, 0.0, 0.0)  # Raw sensor counts, not rad/s.
        self.gyro_calibrated = False
        self.calibration_settle_until = 0.0
        self.calibration_count = 0
        self.calibration_sums = [0.0, 0.0, 0.0]
        self.calibration_sums_sq = [0.0, 0.0, 0.0]

        self.connect_sensor()

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def connect_sensor(self) -> bool:
        now = time.monotonic()
        if now - self.last_reconnect_attempt < self.reconnect_delay_s:
            return False
        self.last_reconnect_attempt = now

        self.close_bus()

        try:
            self.get_logger().info(
                f"Connecting ICM-20948 on /dev/i2c-{self.bus_number}, address 0x{self.address:02X}"
            )
            self.bus = SMBus(self.bus_number)

            self.select_bank(0)
            who = self.read_u8(self.WHO_AM_I)
            if who != self.WHO_AM_I_EXPECTED:
                raise RuntimeError(
                    f"Unexpected WHO_AM_I 0x{who:02X}, expected 0x{self.WHO_AM_I_EXPECTED:02X}"
                )

            self.configure_sensor()
            self.start_gyro_calibration()
            self.consecutive_errors = 0
            self.get_logger().info(
                "ICM-20948 connected. "
                f"accel=+/-{self.accel_range_g}g, gyro=+/-{self.gyro_range_dps}dps, "
                f"publish_rate={self.publish_rate_hz:.1f}Hz"
            )
            return True

        except Exception as exc:
            self.consecutive_errors += 1
            self.get_logger().warn(f"ICM-20948 connect failed: {exc}")
            self.close_bus()
            return False

    def start_gyro_calibration(self) -> None:
        """Reset calibration state after each successful sensor connection."""
        self.gyro_calibrated = False
        self.gyro_offset = (0.0, 0.0, 0.0)
        self.calibration_count = 0
        self.calibration_sums = [0.0, 0.0, 0.0]
        self.calibration_sums_sq = [0.0, 0.0, 0.0]
        self.calibration_settle_until = time.monotonic() + self.gyro_calibration_settle_s
        self.get_logger().info(
            "Gyro calibration: keep crawler stationary; "
            f"settling {self.gyro_calibration_settle_s:.1f}s, "
            f"then collecting {self.gyro_calibration_samples} samples "
            f"at ~{self.publish_rate_hz:.1f} Hz. /imu paused."
        )

    def collect_gyro_calibration_sample(self, gyro_raw: Tuple[int, int, int]) -> None:
        """Collect raw samples and reject calibration if any axis is too noisy."""
        for i, value in enumerate(gyro_raw):
            self.calibration_sums[i] += value
            self.calibration_sums_sq[i] += value * value
        self.calibration_count += 1
        if self.calibration_count < self.gyro_calibration_samples:
            return

        n = self.calibration_count
        means = [total / n for total in self.calibration_sums]
        # Sample standard deviation, in raw counts; convert to deg/s for logs.
        stddev_raw = [
            math.sqrt(max(0.0, (self.calibration_sums_sq[i] - n * means[i] ** 2) / (n - 1)))
            for i in range(3)
        ]
        stddev_dps = [value / self.gyro_lsb_per_dps for value in stddev_raw]
        fmt = lambda values: "(" + ", ".join(f"{value:.4f}" for value in values) + ")"
        self.get_logger().info(
            f"Gyro calibration: mean raw={fmt(means)}, "
            f"stddev deg/s={fmt(stddev_dps)} "
            f"(limit {self.gyro_calibration_max_stddev_dps:.3f} deg/s)."
        )
        if any(value > self.gyro_calibration_max_stddev_dps for value in stddev_dps):
            self.get_logger().warn(
                "Gyro calibration rejected: excessive variation. "
                "Check that the crawler and motors are stationary; retrying."
            )
            self.start_gyro_calibration()
            return

        self.gyro_offset = tuple(means)
        self.gyro_calibrated = True
        offsets_dps = [value / self.gyro_lsb_per_dps for value in means]
        self.get_logger().info(
            f"Gyro calibration complete: offset raw={fmt(self.gyro_offset)}, "
            f"offset deg/s={fmt(offsets_dps)}. Publishing /imu."
        )

    def configure_sensor(self) -> None:
        # Bank 0: wake sensor, use best available clock source.
        self.select_bank(0)
        self.write_u8(self.PWR_MGMT_1, 0x80)  # reset
        time.sleep(0.12)
        self.write_u8(self.PWR_MGMT_1, 0x01)  # auto/select PLL clock, sleep off
        time.sleep(0.02)
        self.write_u8(self.PWR_MGMT_2, 0x00)  # enable accel + gyro axes
        self.write_u8(self.LP_CONFIG, 0x00)   # continuous mode, not low-power cycling
        self.write_u8(self.USER_CTRL, 0x00)   # no internal I2C master for this raw node
        time.sleep(0.02)

        # Bank 2: configure full-scale ranges. Keep DLPF conservative.
        self.select_bank(2)

        gyro_fs = self.GYRO_FS_BITS[self.gyro_range_dps]
        accel_fs = self.ACCEL_FS_BITS[self.accel_range_g]

        # GYRO_CONFIG_1: [2:1] gyro full-scale, bit0 enables DLPF.
        # 0x01 enables gyro DLPF with default low-noise config.
        gyro_config = (gyro_fs << 1) | 0x01
        self.write_u8(self.GYRO_CONFIG_1, gyro_config)

        # ACCEL_CONFIG: [2:1] accel full-scale, bit0 enables DLPF.
        accel_config = (accel_fs << 1) | 0x01
        self.write_u8(self.ACCEL_CONFIG, accel_config)

        # Sample-rate dividers. Internal base differs by DLPF mode, but this is
        # not critical because ROS publication is timer-driven. Set divider 0.
        self.write_u8(self.GYRO_SMPLRT_DIV, 0x00)
        self.write_u8(self.ACCEL_SMPLRT_DIV_1, 0x00)
        self.write_u8(self.ACCEL_SMPLRT_DIV_2, 0x00)

        self.select_bank(0)
        time.sleep(0.02)

    def timer_callback(self) -> None:
        if self.bus is None:
            self.connect_sensor()
            return

        try:
            # Do not publish uncalibrated angular velocity to odometry.
            if not self.gyro_calibrated and time.monotonic() < self.calibration_settle_until:
                return
            accel_raw, gyro_raw = self.read_accel_gyro_raw()
            if not self.gyro_calibrated:
                self.collect_gyro_calibration_sample(gyro_raw)
                self.consecutive_errors = 0
                return

            gyro_corrected = tuple(
                raw - offset for raw, offset in zip(gyro_raw, self.gyro_offset)
            )
            accel = self.convert_accel(accel_raw)
            gyro = self.convert_gyro(gyro_corrected)

            accel = self.apply_axis_mapping(accel)
            gyro = self.apply_axis_mapping(gyro)

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id

            msg = Imu()
            msg.header = header

            # ICM-20948 is not a fusion IMU like BNO086. No valid orientation here.
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0

            gx, gy, gz = gyro
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            ax, ay, az = accel
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            self.fill_covariances(msg)
            self.imu_pub.publish(msg)
            self.consecutive_errors = 0

        except Exception as exc:
            self.consecutive_errors += 1
            self.get_logger().warn(
                f"ICM-20948 read failed "
                f"({self.consecutive_errors}/{self.max_consecutive_errors}): {exc}"
            )
            if self.consecutive_errors >= self.max_consecutive_errors:
                self.get_logger().warn("Too many ICM-20948 errors, reconnecting sensor.")
                self.connect_sensor()

    def read_accel_gyro_raw(self) -> Tuple[Tuple[int, int, int], Tuple[int, int, int]]:
        self.select_bank(0)
        data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 12)
        accel_raw = (
            self.to_int16(data[0], data[1]),
            self.to_int16(data[2], data[3]),
            self.to_int16(data[4], data[5]),
        )
        gyro_raw = (
            self.to_int16(data[6], data[7]),
            self.to_int16(data[8], data[9]),
            self.to_int16(data[10], data[11]),
        )
        return accel_raw, gyro_raw

    def convert_accel(self, raw: Tuple[int, int, int]) -> Tuple[float, float, float]:
        # raw -> g -> m/s^2
        return tuple((v / self.accel_lsb_per_g) * 9.80665 for v in raw)

    def convert_gyro(self, raw: Tuple[float, float, float]) -> Tuple[float, float, float]:
        # raw -> deg/s -> rad/s
        return tuple(math.radians(v / self.gyro_lsb_per_dps) for v in raw)

    def apply_axis_mapping(self, values: Tuple[float, float, float]) -> Tuple[float, float, float]:
        x, y, z = values
        if self.swap_xy:
            x, y = y, x
        if self.invert_x:
            x = -x
        if self.invert_y:
            y = -y
        if self.invert_z:
            z = -z
        return x, y, z

    def select_bank(self, bank: int) -> None:
        if bank not in (0, 1, 2, 3):
            raise ValueError("ICM-20948 bank must be 0, 1, 2 or 3")
        self.write_u8(self.REG_BANK_SEL, bank << 4)

    def read_u8(self, reg: int) -> int:
        return int(self.bus.read_byte_data(self.address, reg))

    def write_u8(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def close_bus(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None

    @staticmethod
    def to_int16(msb: int, lsb: int) -> int:
        value = (msb << 8) | lsb
        if value & 0x8000:
            value -= 0x10000
        return value

    @staticmethod
    def fill_covariances(msg: Imu) -> None:
        # ROS convention: -1 in element 0 means orientation is not provided.
        msg.orientation_covariance[0] = -1.0

        # Placeholders, not calibrated covariance.
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        msg.linear_acceleration_covariance[0] = 0.2
        msg.linear_acceleration_covariance[4] = 0.2
        msg.linear_acceleration_covariance[8] = 0.2

    def destroy_node(self) -> bool:
        self.close_bus()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Icm20948RawNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
