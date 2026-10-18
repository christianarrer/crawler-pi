import pigpio
import serial

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Int64MultiArray


COMM_RATE_SEC = 0.1

MOTOR_TARGET_TIMEOUT_SEC = 1.0

CLIFF_SERIAL_DEVICE = "/dev/ttyACM0"
CLIFF_SERIAL_BAUD = 115200
CLIFF_DATA_TIMEOUT_SEC = 0.30
CLIFF_SERIAL_POLL_SEC = 0.02
CLIFF_RECONNECT_SEC = 1.0

CLIFF_SAFE = 0
CLIFF_DETECTED = 1
CLIFF_UNKNOWN = 2

PWM_FREQUENCY_HZ = 50  # 50 Hz = 20 ms period

PIGPIO_HW_PWM_RANGE = 1_000_000
PIGPIO_SOFT_PWM_RANGE = 1_000

MOTOR_EN_GPIO = 5

LEFT_FWD_GPIO = 12
LEFT_REV_GPIO = 16

RIGHT_FWD_GPIO = 13
RIGHT_REV_GPIO = 26


def target_to_dir(target: float) -> int:
    if target > 0.0:
        return 1
    if target < 0.0:
        return -1
    return 0


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("crawler_motor_driver")

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpiod. Is pigpiod running?")

        # Commanded motor targets from the velocity target node.
        self.left_target = 0.0
        self.right_target = 0.0
        self.last_left_target_time = None
        self.last_right_target_time = None

        # Effective directions actually sent to the motor driver.
        self.left_dir = Int32()
        self.left_dir.data = 0
        self.right_dir = Int32()
        self.right_dir.data = 0

        # Cliff states reported by the Pico:
        # 4 = front-left, 5 = front-right,
        # 6 = rear-right, 7 = rear-left.
        self.cliff_states = {
            4: CLIFF_UNKNOWN,
            5: CLIFF_UNKNOWN,
            6: CLIFF_UNKNOWN,
            7: CLIFF_UNKNOWN,
        }
        self.last_cliff_data_time = None

        self.serial_port = None
        self.serial_rx_buffer = bytearray()
        self.last_serial_open_attempt_time = None

        # Used only to avoid repeating the same safety log message continuously.
        self.last_safety_reason = None

        self.setup_gpio()
        self.disable_motor_output()

        self.left_dir_publisher = self.create_publisher(
            Int32,
            "crawler/motor/left/dir",
            16,
        )
        self.right_dir_publisher = self.create_publisher(
            Int32,
            "crawler/motor/right/dir",
            16,
        )

        # Parsed cliff sensor data from the Pico.
        #
        # Topic:
        #   /crawler/cliff_sensors
        #
        # Int64MultiArray layout:
        #   [time_ms,
        #    d4, s4, c4,
        #    d5, s5, c5,
        #    d6, s6, c6,
        #    d7, s7, c7]
        #
        # Sensor positions:
        #   4 = front-left
        #   5 = front-right
        #   6 = rear-right
        #   7 = rear-left
        #
        # Cliff state:
        #   0 = SAFE
        #   1 = CLIFF_DETECTED
        #   2 = UNKNOWN
        #
        # The leading serial record marker "D" is deliberately not included
        # because the ROS topic already has a fixed message type.
        self.cliff_sensor_publisher = self.create_publisher(
            Int64MultiArray,
            "/crawler/cliff_sensors",
            16,
        )

        self.left_target_subscriber = self.create_subscription(
            Float32,
            "/crawler/motor/left/target",
            self.read_left_target,
            16,
        )
        self.right_target_subscriber = self.create_subscription(
            Float32,
            "/crawler/motor/right/target",
            self.read_right_target,
            16,
        )

        self.dir_publish_timer = self.create_timer(
            COMM_RATE_SEC,
            self.publish_dir,
        )

        # Polling is non-blocking. The node remains fail-safe if the Pico is
        # absent or the serial connection fails.
        self.cliff_serial_timer = self.create_timer(
            CLIFF_SERIAL_POLL_SEC,
            self.poll_cliff_serial,
        )

        # Re-evaluate all timeouts even when no ROS target or serial message
        # arrives.
        self.safety_timer = self.create_timer(
            0.05,
            self.safety_check,
        )

        # Try opening immediately. Failure is not fatal: motors stay disabled
        # and the timer keeps trying to reconnect.
        self.try_open_cliff_serial()

        self.get_logger().info("Crawler motor driver node started")
        self.get_logger().info(
            f"Cliff safety enabled on {CLIFF_SERIAL_DEVICE} at "
            f"{CLIFF_SERIAL_BAUD} baud"
        )

    # -------------------------------------------------------------------------
    # ROS I/O
    # -------------------------------------------------------------------------

    def publish_dir(self):
        self.left_dir_publisher.publish(self.left_dir)
        self.right_dir_publisher.publish(self.right_dir)

    def read_left_target(self, msg):
        self.left_target = self.clamp_target(msg.data)
        self.last_left_target_time = self.get_clock().now()
        self.apply_motor_targets()

    def read_right_target(self, msg):
        self.right_target = self.clamp_target(msg.data)
        self.last_right_target_time = self.get_clock().now()
        self.apply_motor_targets()

    # -------------------------------------------------------------------------
    # Cliff sensor serial input
    # -------------------------------------------------------------------------

    def try_open_cliff_serial(self):
        now = self.get_clock().now()

        if self.serial_port is not None:
            return

        if self.last_serial_open_attempt_time is not None:
            elapsed = now - self.last_serial_open_attempt_time
            if elapsed < Duration(seconds=CLIFF_RECONNECT_SEC):
                return

        self.last_serial_open_attempt_time = now

        try:
            self.serial_port = serial.Serial(
                port=CLIFF_SERIAL_DEVICE,
                baudrate=CLIFF_SERIAL_BAUD,
                timeout=0,
            )
            self.serial_rx_buffer.clear()
            self.get_logger().info(
                f"Opened cliff sensor serial device {CLIFF_SERIAL_DEVICE}"
            )
        except (serial.SerialException, OSError) as exc:
            self.serial_port = None
            self.set_cliff_data_unavailable()
            self.log_safety_reason(
                f"cliff serial unavailable ({exc})"
            )

    def close_cliff_serial(self):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except (serial.SerialException, OSError):
                pass

        self.serial_port = None
        self.serial_rx_buffer.clear()

    def poll_cliff_serial(self):
        if self.serial_port is None:
            self.try_open_cliff_serial()
            self.apply_motor_targets()
            return

        try:
            waiting = self.serial_port.in_waiting
            if waiting > 0:
                self.serial_rx_buffer.extend(self.serial_port.read(waiting))

            while b"\n" in self.serial_rx_buffer:
                raw_line, _, remainder = self.serial_rx_buffer.partition(b"\n")
                self.serial_rx_buffer = bytearray(remainder)

                line = raw_line.decode("ascii", errors="replace").strip()
                if line:
                    self.process_cliff_serial_line(line)

        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(
                f"Cliff serial connection lost: {exc}"
            )
            self.close_cliff_serial()
            self.set_cliff_data_unavailable()

        # A newly received cliff state must affect the motors immediately,
        # without waiting for the next velocity target message.
        self.apply_motor_targets()

    def process_cliff_serial_line(self, line: str):
        # Ignore Pico startup/information lines beginning with '#'.
        if line.startswith("#"):
            return

        # Pico format:
        # D,time_ms,d4,s4,c4,d5,s5,c5,d6,s6,c6,d7,s7,c7
        fields = line.split(",")

        if len(fields) != 14 or fields[0] != "D":
            return

        try:
            # Parse all numeric fields so a damaged/truncated line is rejected.
            values = [int(value) for value in fields[1:]]
        except ValueError:
            return

        time_ms = values[0]

        d4, s4, c4 = values[1:4]
        d5, s5, c5 = values[4:7]
        d6, s6, c6 = values[7:10]
        d7, s7, c7 = values[10:13]

        states = (c4, c5, c6, c7)
        if any(
            state not in (CLIFF_SAFE, CLIFF_DETECTED, CLIFF_UNKNOWN)
            for state in states
        ):
            return

        self.cliff_states[4] = c4
        self.cliff_states[5] = c5
        self.cliff_states[6] = c6
        self.cliff_states[7] = c7

        # Freshness is measured with the Raspberry Pi ROS clock. The Pico
        # timestamp is deliberately not used because the two clocks are not
        # synchronized.
        self.last_cliff_data_time = self.get_clock().now()

        # Publish every valid Pico measurement immediately. Consumers such as
        # cliff_costmap_node should use their own ROS receive time for timeout
        # handling; time_ms is useful as the Pico-side sample timestamp.
        cliff_msg = Int64MultiArray()
        cliff_msg.data = [
            time_ms,
            d4, s4, c4,
            d5, s5, c5,
            d6, s6, c6,
            d7, s7, c7,
        ]
        self.cliff_sensor_publisher.publish(cliff_msg)

    def set_cliff_data_unavailable(self):
        self.last_cliff_data_time = None
        for sensor in self.cliff_states:
            self.cliff_states[sensor] = CLIFF_UNKNOWN

    # -------------------------------------------------------------------------
    # Safety logic
    # -------------------------------------------------------------------------

    def clamp_target(self, target):
        return max(-1.0, min(1.0, float(target)))

    def motor_targets_are_fresh(self):
        if (
            self.last_left_target_time is None
            or self.last_right_target_time is None
        ):
            return False

        now = self.get_clock().now()

        return (
            now - self.last_left_target_time
            <= Duration(seconds=MOTOR_TARGET_TIMEOUT_SEC)
            and now - self.last_right_target_time
            <= Duration(seconds=MOTOR_TARGET_TIMEOUT_SEC)
        )

    def cliff_data_are_fresh(self):
        if self.last_cliff_data_time is None:
            return False

        elapsed = self.get_clock().now() - self.last_cliff_data_time
        return elapsed <= Duration(seconds=CLIFF_DATA_TIMEOUT_SEC)

    def apply_cliff_safety(self, left, right):
        """
        Return (safe_left, safe_right, reason).

        Fail-safe rules:
        - No fresh cliff data -> stop.
        - Any UNKNOWN sensor -> stop.
        - No cliff -> requested differential-drive targets pass unchanged.
        - Any cliff -> rotation is forbidden.
        - Front cliff (4 or 5) blocks forward motion.
        - Rear cliff (6 or 7) blocks reverse motion.
        - An allowed escape direction is forced straight by reducing both
          motor targets to the smaller requested magnitude.
        """

        if not self.cliff_data_are_fresh():
            return 0.0, 0.0, "no fresh cliff sensor data"

        unknown_sensors = [
            sensor
            for sensor, state in self.cliff_states.items()
            if state == CLIFF_UNKNOWN
        ]
        if unknown_sensors:
            sensor_list = ",".join(str(sensor) for sensor in unknown_sensors)
            return (
                0.0,
                0.0,
                f"cliff sensor UNKNOWN: {sensor_list}",
            )

        front_cliff = (
            self.cliff_states[4] == CLIFF_DETECTED
            or self.cliff_states[5] == CLIFF_DETECTED
        )
        rear_cliff = (
            self.cliff_states[6] == CLIFF_DETECTED
            or self.cliff_states[7] == CLIFF_DETECTED
        )

        if not front_cliff and not rear_cliff:
            return left, right, "clear"

        # A cliff exists both in front and behind. There is no automatically
        # safe translational escape direction.
        if front_cliff and rear_cliff:
            return 0.0, 0.0, "front and rear cliff detected"

        # From here on at least one cliff is present, therefore rotation must
        # always be blocked. Only same-direction commands on both tracks can
        # be converted into an exactly straight escape motion.

        if left > 0.0 and right > 0.0:
            if front_cliff:
                return 0.0, 0.0, "front cliff blocks forward motion"

            # Rear cliff: straight forward escape is allowed.
            target = min(left, right)
            return (
                target,
                target,
                "rear cliff: straight forward escape",
            )

        if left < 0.0 and right < 0.0:
            if rear_cliff:
                return 0.0, 0.0, "rear cliff blocks reverse motion"

            # Front cliff: straight reverse escape is allowed.
            target = -min(abs(left), abs(right))
            return (
                target,
                target,
                "front cliff: straight reverse escape",
            )

        # Opposite signs, one motor stopped, or any other mixed command has a
        # rotational component and is therefore blocked while a cliff exists.
        return 0.0, 0.0, "rotation blocked while cliff detected"

    def apply_motor_targets(self):
        if not self.motor_targets_are_fresh():
            self.disable_motor_output()
            self.log_safety_reason("motor target timeout or incomplete target pair")
            return

        safe_left, safe_right, reason = self.apply_cliff_safety(
            self.left_target,
            self.right_target,
        )

        if safe_left == 0.0 and safe_right == 0.0:
            self.disable_motor_output()
            self.log_safety_reason(reason)
            return

        self.pi.write(MOTOR_EN_GPIO, 1)

        self.set_motor(
            fwd_gpio=LEFT_FWD_GPIO,
            rev_gpio=LEFT_REV_GPIO,
            target=safe_left,
        )
        self.left_dir.data = target_to_dir(safe_left)

        self.set_motor(
            fwd_gpio=RIGHT_FWD_GPIO,
            rev_gpio=RIGHT_REV_GPIO,
            target=safe_right,
        )
        self.right_dir.data = target_to_dir(safe_right)

        self.log_safety_reason(reason)

    def safety_check(self):
        # Re-evaluate both motor-command and cliff-data freshness periodically.
        # This guarantees a stop even if no new callback arrives.
        self.apply_motor_targets()

    def log_safety_reason(self, reason):
        if reason == self.last_safety_reason:
            return

        self.last_safety_reason = reason

        if reason == "clear":
            self.get_logger().info("Cliff safety: clear")
        elif "escape" in reason:
            self.get_logger().warn(f"Cliff safety: {reason}")
        else:
            self.get_logger().warn(f"Cliff safety STOP: {reason}")

    # -------------------------------------------------------------------------
    # Motor hardware
    # -------------------------------------------------------------------------

    def setup_gpio(self):
        self.pi.set_mode(MOTOR_EN_GPIO, pigpio.OUTPUT)
        self.pi.write(MOTOR_EN_GPIO, 0)

        # Forward pins use hardware PWM.
        self.pi.set_mode(LEFT_FWD_GPIO, pigpio.OUTPUT)
        self.pi.set_mode(RIGHT_FWD_GPIO, pigpio.OUTPUT)

        # Reverse pins use pigpio software PWM.
        for gpio in (LEFT_REV_GPIO, RIGHT_REV_GPIO):
            self.pi.set_mode(gpio, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(gpio, PWM_FREQUENCY_HZ)
            self.pi.set_PWM_range(gpio, PIGPIO_SOFT_PWM_RANGE)
            self.pi.set_PWM_dutycycle(gpio, 0)

        self.pi.hardware_PWM(
            LEFT_FWD_GPIO,
            PWM_FREQUENCY_HZ,
            0,
        )
        self.pi.hardware_PWM(
            RIGHT_FWD_GPIO,
            PWM_FREQUENCY_HZ,
            0,
        )

    def set_motor(self, fwd_gpio, rev_gpio, target):
        target = self.clamp_target(target)

        hw_duty = int(abs(target) * PIGPIO_HW_PWM_RANGE)
        sw_duty = int(abs(target) * PIGPIO_SOFT_PWM_RANGE)

        if target > 0.0:
            # Forward: forward PWM active, reverse PWM off.
            self.pi.set_PWM_dutycycle(rev_gpio, 0)
            self.pi.hardware_PWM(
                fwd_gpio,
                PWM_FREQUENCY_HZ,
                hw_duty,
            )

        elif target < 0.0:
            # Reverse: forward PWM off, reverse PWM active.
            self.pi.hardware_PWM(
                fwd_gpio,
                PWM_FREQUENCY_HZ,
                0,
            )
            self.pi.set_PWM_dutycycle(rev_gpio, sw_duty)

        else:
            # Stop this motor side.
            self.pi.hardware_PWM(
                fwd_gpio,
                PWM_FREQUENCY_HZ,
                0,
            )
            self.pi.set_PWM_dutycycle(rev_gpio, 0)

    def disable_motor_output(self):
        """
        Stop the hardware outputs without modifying the commanded targets.

        Keeping requested targets separate from effective outputs is important:
        the safety layer may temporarily block motion, but it must not pretend
        that the upstream velocity target itself changed.
        """
        self.left_dir.data = 0
        self.right_dir.data = 0

        self.pi.hardware_PWM(
            LEFT_FWD_GPIO,
            PWM_FREQUENCY_HZ,
            0,
        )
        self.pi.hardware_PWM(
            RIGHT_FWD_GPIO,
            PWM_FREQUENCY_HZ,
            0,
        )

        self.pi.set_PWM_dutycycle(LEFT_REV_GPIO, 0)
        self.pi.set_PWM_dutycycle(RIGHT_REV_GPIO, 0)

        self.pi.write(MOTOR_EN_GPIO, 0)

    def shutdown(self):
        self.get_logger().info("Shutting down motor driver")
        self.disable_motor_output()
        self.close_cliff_serial()
        self.pi.stop()


def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriverNode()

    try:
        rclpy.spin(motor_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver_node.shutdown()
        motor_driver_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
