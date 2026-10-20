#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState
from smbus2 import SMBus


class Ina226:
    REG_CONFIG = 0x00
    REG_SHUNT_VOLTAGE = 0x01
    REG_BUS_VOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05

    def __init__(self, bus_number: int, address: int, shunt_ohms: float, max_current_a: float):
        self.bus = SMBus(bus_number)
        self.address = address
        self.shunt_ohms = shunt_ohms
        self.max_current_a = max_current_a

        # INA226 datasheet recommendation:
        # current_lsb = maximum expected current / 32768
        self.current_lsb = max_current_a / 32768.0

        # Calibration register:
        # calibration = 0.00512 / (current_lsb * shunt_ohms)
        calibration = int(0.00512 / (self.current_lsb * shunt_ohms))
        self.write_register(self.REG_CALIBRATION, calibration)

        # Continuous shunt and bus measurement mode.
        # This config is a reasonable starting point.
        self.write_register(self.REG_CONFIG, 0x4127)

    def close(self):
        self.bus.close()

    def write_register(self, register: int, value: int):
        msb = (value >> 8) & 0xFF
        lsb = value & 0xFF
        self.bus.write_i2c_block_data(self.address, register, [msb, lsb])

    def read_register_unsigned(self, register: int) -> int:
        data = self.bus.read_i2c_block_data(self.address, register, 2)
        return (data[0] << 8) | data[1]

    def read_register_signed(self, register: int) -> int:
        value = self.read_register_unsigned(register)
        if value & 0x8000:
            value -= 0x10000
        return value

    def read_bus_voltage_v(self) -> float:
        raw = self.read_register_unsigned(self.REG_BUS_VOLTAGE)
        return raw * 0.00125

    def read_shunt_voltage_v(self) -> float:
        raw = self.read_register_signed(self.REG_SHUNT_VOLTAGE)
        return raw * 0.0000025

    def read_current_a(self) -> float:
        raw = self.read_register_signed(self.REG_CURRENT)
        return raw * self.current_lsb

    def read_power_w(self) -> float:
        raw = self.read_register_unsigned(self.REG_POWER)
        power_lsb = 25.0 * self.current_lsb
        return raw * power_lsb


class Ina226Node(Node):
    def __init__(self):
        super().__init__("ina226_node")

        self.declare_parameter("bus_number", 1)
        self.declare_parameter("address", 0x40)
        self.declare_parameter("shunt_ohms", 0.002)
        self.declare_parameter("max_current_a", 10.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("frame_id", "battery")
        self.declare_parameter("current_sign", 1.0)

        # Optional robustness parameters.
        self.declare_parameter("max_i2c_errors_before_reopen", 2)
        self.declare_parameter("reopen_delay_s", 0.2)

        self.bus_number = int(self.get_parameter("bus_number").value)
        self.address = int(self.get_parameter("address").value)
        self.shunt_ohms = float(self.get_parameter("shunt_ohms").value)
        self.max_current_a = float(self.get_parameter("max_current_a").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.current_sign = float(self.get_parameter("current_sign").value)

        self.max_i2c_errors_before_reopen = int(
            self.get_parameter("max_i2c_errors_before_reopen").value
        )
        self.reopen_delay_s = float(self.get_parameter("reopen_delay_s").value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        self.i2c_error_count = 0
        self.sensor = None
        self.open_sensor()

        self.voltage_pub = self.create_publisher(Float32, "/battery/voltage", 10)
        self.current_pub = self.create_publisher(Float32, "/battery/current", 10)
        self.power_pub = self.create_publisher(Float32, "/battery/power", 10)
        self.battery_state_pub = self.create_publisher(BatteryState, "/battery/state", 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_measurements)

        self.get_logger().info(
            f"INA226 started on /dev/i2c-{self.bus_number}, "
            f"address 0x{self.address:02x}, "
            f"shunt={self.shunt_ohms} ohm, "
            f"max_current={self.max_current_a} A, "
            f"publish_rate={self.publish_rate_hz} Hz"
        )

    def open_sensor(self):
        self.sensor = Ina226(
            bus_number=self.bus_number,
            address=self.address,
            shunt_ohms=self.shunt_ohms,
            max_current_a=self.max_current_a,
        )

    def close_sensor(self):
        if self.sensor is None:
            return

        try:
            self.sensor.close()
        except Exception as exc:
            self.get_logger().warn(f"Could not close INA226 sensor cleanly: {exc}")
        finally:
            self.sensor = None

    def reopen_sensor(self):
        self.get_logger().warn(
            f"Reopening INA226 on /dev/i2c-{self.bus_number}, address 0x{self.address:02x}"
        )

        self.close_sensor()
        time.sleep(self.reopen_delay_s)

        try:
            self.open_sensor()
            self.i2c_error_count = 0
            self.get_logger().info("INA226 reopen successful")
        except OSError as exc:
            self.sensor = None
            self.get_logger().error(f"INA226 reopen failed: {exc}")
        except Exception as exc:
            self.sensor = None
            self.get_logger().error(f"Unexpected INA226 reopen error: {exc}")

    def publish_measurements(self):
        if self.sensor is None:
            self.reopen_sensor()
            return

        try:
            voltage = self.sensor.read_bus_voltage_v()
            current = self.sensor.read_current_a() * self.current_sign
            power = self.sensor.read_power_w()

            # Successful I2C transaction set.
            self.i2c_error_count = 0

            self.voltage_pub.publish(Float32(data=float(voltage)))
            self.current_pub.publish(Float32(data=float(current)))
            self.power_pub.publish(Float32(data=float(power)))

            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.header.frame_id = self.frame_id
            battery_msg.voltage = float(voltage)
            battery_msg.current = float(current)
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_NIMH
            battery_msg.present = True

            self.battery_state_pub.publish(battery_msg)

        except OSError as exc:
            self.i2c_error_count += 1
            self.get_logger().warn(
                f"INA226 I2C read failed "
                f"({self.i2c_error_count}/{self.max_i2c_errors_before_reopen}): {exc}"
            )

            if self.i2c_error_count >= self.max_i2c_errors_before_reopen:
                self.reopen_sensor()

        except Exception as exc:
            # This should not normally happen, but prevents the timer callback from dying
            # due to an unexpected exception.
            self.get_logger().error(f"Unexpected INA226 error: {exc}")

    def destroy_node(self):
        self.close_sensor()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ina226Node()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()