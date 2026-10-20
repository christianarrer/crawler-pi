#!/usr/bin/env python3

from collections import deque
import struct
import time

from smbus2 import SMBus

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import UInt64


# TMAG5273 registers.
REG_DEVICE_CONFIG_1 = 0x00
REG_DEVICE_CONFIG_2 = 0x01
REG_SENSOR_CONFIG_1 = 0x02
REG_SENSOR_CONFIG_2 = 0x03
REG_MANUFACTURER_ID_LSB = 0x0E
REG_Z_MSB_RESULT = 0x16

# Register values.
MAG_CH_Z = 0x04
SLEEP_1MS = 0x00
MODE_CONTINUOUS = 0x02
CONV_AVG_4X = 0x02
CONV_AVG_8X = 0x03

# Adafruit uses 0x5449 for TI manufacturer ID.
EXPECTED_MANUFACTURER_ID = 0x5449


class Tmag5273Minimal:
    def __init__(
        self,
        bus,
        address,
        range_z_mt=80.0,
        conversion_average=CONV_AVG_4X,
        verify_manufacturer=False,
    ):
        self.bus = bus
        self.address = int(address)
        self.range_z_mt = float(range_z_mt)

        if verify_manufacturer:
            manufacturer_id = self.read_manufacturer_id()
            if manufacturer_id != EXPECTED_MANUFACTURER_ID:
                raise RuntimeError(
                    f"TMAG5273 at 0x{self.address:02X}: "
                    f"unexpected manufacturer ID 0x{manufacturer_id:04X}"
                )

        self.configure_z_only(conversion_average=conversion_average)

    def read_register(self, register):
        return self.bus.read_byte_data(self.address, register)

    def write_register(self, register, value):
        self.bus.write_byte_data(self.address, register, value & 0xFF)

    def read_manufacturer_id(self):
        data = self.bus.read_i2c_block_data(
            self.address,
            REG_MANUFACTURER_ID_LSB,
            2,
        )
        return data[0] | (data[1] << 8)

    def configure_z_only(self, conversion_average):
        # DEVICE_CONFIG_1:
        # bits 4:2 = conversion average.
        # Keep CRC off and tempco none for minimal bus operation.
        self.write_register(
            REG_DEVICE_CONFIG_1,
            (int(conversion_average) & 0x07) << 2,
        )

        # SENSOR_CONFIG_1:
        # bits 7:4 = magnetic channels, bits 3:0 = sleep time.
        # Z only, sleep 1 ms.
        self.write_register(
            REG_SENSOR_CONFIG_1,
            (MAG_CH_Z << 4) | SLEEP_1MS,
        )

        # SENSOR_CONFIG_2:
        # bit 0 = Z wide range.
        # For A1 wide is 80 mT, for A2 wide is 266 mT.
        # Your previous Adafruit values looked like A1/80mT range,
        # so range_z_mt=80.0 is the default.
        self.write_register(REG_SENSOR_CONFIG_2, 0x01)

        # DEVICE_CONFIG_2:
        # bit 4 = low noise, bits 1:0 = operating mode.
        # Low noise + continuous.
        self.write_register(REG_DEVICE_CONFIG_2, 0x10 | MODE_CONTINUOUS)

        time.sleep(0.01)

    def read_z_raw(self):
        data = self.bus.read_i2c_block_data(
            self.address,
            REG_Z_MSB_RESULT,
            2,
        )
        return struct.unpack(">h", bytes(data))[0]

    def read_z_ut(self):
        raw = self.read_z_raw()
        return (float(raw) / 32768.0) * self.range_z_mt * 1000.0


class PeakDetector:
    def __init__(
        self,
        magnets_per_rev,
        high_threshold,
        low_threshold,
        peak_history_size,
        rpm_timeout_s=1.5,
        max_peak_gap_s=3.0,
    ):
        self.magnets_per_rev = int(magnets_per_rev)
        self.high_threshold = float(high_threshold)
        self.low_threshold = float(low_threshold)
        self.peak_times = deque(maxlen=int(peak_history_size))

        self.rpm_timeout_s = float(rpm_timeout_s)
        self.max_peak_gap_s = float(max_peak_gap_s)

        self.armed = True
        self.peak_count = 0
        self.rpm = 0.0
        self.last_peak_time = None

    def update(self, value, now):
        # If no peak was seen recently, velocity is zero.
        if self.last_peak_time is not None:
            if (now - self.last_peak_time) > self.rpm_timeout_s:
                self.rpm = 0.0

        if self.armed and value > self.high_threshold:
            # If this peak is far away from the previous peak, start a new RPM window.
            if self.last_peak_time is not None:
                if (now - self.last_peak_time) > self.max_peak_gap_s:
                    self.peak_times.clear()

            self.peak_count += 1
            self.armed = False
            self.peak_times.append(now)
            self.last_peak_time = now
            self.rpm = self.calculate_rpm()

        elif not self.armed and value < self.low_threshold:
            self.armed = True
    def calculate_rpm(self):
        if len(self.peak_times) < 2:
            return 0.0

        dt = self.peak_times[-1] - self.peak_times[0]
        if dt <= 0.0:
            return 0.0

        peak_intervals = len(self.peak_times) - 1
        peak_frequency_hz = peak_intervals / dt

        return 60.0 * peak_frequency_hz / self.magnets_per_rev

    def revolutions(self):
        return float(self.peak_count) / float(self.magnets_per_rev)


class TmagOdomPairNode(Node):
    def __init__(self):
        super().__init__("tmag_odom_pair_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("left_address", 0x22)
        self.declare_parameter("right_address", 0x23)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.declare_parameter("magnets_per_rev", 4)
        self.declare_parameter("range_z_mt", 80.0)
        self.declare_parameter("conversion_average", CONV_AVG_4X)
        self.declare_parameter("verify_manufacturer", False)

        self.declare_parameter("left_high_threshold", 700.0)
        self.declare_parameter("left_low_threshold", 100.0)

        self.declare_parameter("right_high_threshold", 700.0)
        self.declare_parameter("right_low_threshold", 100.0)

        self.declare_parameter("peak_history_size", 6)

        self.i2c_bus_number = int(self.get_parameter("i2c_bus").value)
        self.left_address = int(self.get_parameter("left_address").value)
        self.right_address = int(self.get_parameter("right_address").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.magnets_per_rev = int(self.get_parameter("magnets_per_rev").value)
        self.range_z_mt = float(self.get_parameter("range_z_mt").value)
        self.conversion_average = int(self.get_parameter("conversion_average").value)
        self.verify_manufacturer = bool(
            self.get_parameter("verify_manufacturer").value
        )

        self.left_high_threshold = float(
            self.get_parameter("left_high_threshold").value
        )
        self.left_low_threshold = float(
            self.get_parameter("left_low_threshold").value
        )

        self.right_high_threshold = float(
            self.get_parameter("right_high_threshold").value
        )
        self.right_low_threshold = float(
            self.get_parameter("right_low_threshold").value
        )

        self.peak_history_size = int(
            self.get_parameter("peak_history_size").value
        )

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        self.bus = SMBus(self.i2c_bus_number)

        self.left_sensor = Tmag5273Minimal(
            bus=self.bus,
            address=self.left_address,
            range_z_mt=self.range_z_mt,
            conversion_average=self.conversion_average,
            verify_manufacturer=self.verify_manufacturer,
        )

        self.right_sensor = Tmag5273Minimal(
            bus=self.bus,
            address=self.right_address,
            range_z_mt=self.range_z_mt,
            conversion_average=self.conversion_average,
            verify_manufacturer=self.verify_manufacturer,
        )

        self.left_detector = PeakDetector(
            magnets_per_rev=self.magnets_per_rev,
            high_threshold=self.left_high_threshold,
            low_threshold=self.left_low_threshold,
            peak_history_size=self.peak_history_size,
            rpm_timeout_s=1.5,
            max_peak_gap_s=2.0,
        )

        self.right_detector = PeakDetector(
            magnets_per_rev=self.magnets_per_rev,
            high_threshold=self.right_high_threshold,
            low_threshold=self.right_low_threshold,
            peak_history_size=self.peak_history_size,
            rpm_timeout_s=1.5,
            max_peak_gap_s=2.0,
        )
        now = time.perf_counter()
        self.last_sample_time = now

        self.left_z_pub = self.create_publisher(
            Float32,
            "/odometer/left/magnetic_z",
            10,
        )
        self.left_raw_z_pub = self.create_publisher(
            Float32,
            "/odometer/left/raw_z",
            10,
        )
        self.left_peak_count_pub = self.create_publisher(
            UInt64,
            "/odometer/left/peak_count",
            10,
        )
        self.left_rpm_pub = self.create_publisher(
            Float32,
            "/odometer/left/rpm",
            10,
        )
        self.left_revolutions_pub = self.create_publisher(
            Float32,
            "/odometer/left/revolutions",
            10,
        )

        self.right_z_pub = self.create_publisher(
            Float32,
            "/odometer/right/magnetic_z",
            10,
        )
        self.right_raw_z_pub = self.create_publisher(
            Float32,
            "/odometer/right/raw_z",
            10,
        )
        self.right_peak_count_pub = self.create_publisher(
            UInt64,
            "/odometer/right/peak_count",
            10,
        )
        self.right_rpm_pub = self.create_publisher(
            Float32,
            "/odometer/right/rpm",
            10,
        )
        self.right_revolutions_pub = self.create_publisher(
            Float32,
            "/odometer/right/revolutions",
            10,
        )
        self.dt_pub = self.create_publisher(
            Float32,
            "/odometer/dt_s",
            10,
        )

        timer_period_s = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period_s, self.timer_callback)

        self.get_logger().info(
            "Started minimal TMAG odometer pair node: "
            f"bus={self.i2c_bus_number}, "
            f"left=0x{self.left_address:02X}, "
            f"right=0x{self.right_address:02X}, "
            f"rate={self.publish_rate_hz} Hz, "
            f"magnets_per_rev={self.magnets_per_rev}, "
            f"range_z_mt={self.range_z_mt}, "
            f"conversion_average={self.conversion_average}"
        )

    def publish_float(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def publish_uint64(self, publisher, value):
        msg = UInt64()
        msg.data = int(value)
        publisher.publish(msg)

    def timer_callback(self):
        try:
            left_raw_z = self.left_sensor.read_z_raw()
            right_raw_z = self.right_sensor.read_z_raw()

            left_z = (float(left_raw_z) / 32768.0) * self.range_z_mt * 1000.0
            right_z = (float(right_raw_z) / 32768.0) * self.range_z_mt * 1000.0

        except OSError as exc:
            self.get_logger().warn(f"I2C read failed: {exc}")
            return
        except TimeoutError as exc:
            self.get_logger().warn(f"I2C read timeout: {exc}")
            return

        sample_time = time.perf_counter()
        dt_s = sample_time - self.last_sample_time
        self.last_sample_time = sample_time

        self.left_detector.update(left_z, sample_time)
        self.right_detector.update(right_z, sample_time)

        self.publish_float(self.left_z_pub, left_z)
        self.publish_float(self.left_raw_z_pub, left_raw_z)
        self.publish_uint64(
            self.left_peak_count_pub,
            self.left_detector.peak_count,
        )
        self.publish_float(self.left_rpm_pub, self.left_detector.rpm)
        self.publish_float(
            self.left_revolutions_pub,
            self.left_detector.revolutions(),
        )

        self.publish_float(self.right_z_pub, right_z)
        self.publish_float(self.right_raw_z_pub, right_raw_z)
        self.publish_uint64(
            self.right_peak_count_pub,
            self.right_detector.peak_count,
        )
        self.publish_float(self.right_rpm_pub, self.right_detector.rpm)
        self.publish_float(
            self.right_revolutions_pub,
            self.right_detector.revolutions(),
        )

        self.publish_float(self.dt_pub, dt_s)

    def destroy_node(self):
        self.get_logger().info("Closing I2C bus")

        try:
            self.bus.close()
        except Exception as exc:
            self.get_logger().warn(f"SMBus close failed: {exc}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TmagOdomPairNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()