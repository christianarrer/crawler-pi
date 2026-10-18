# crawler_sensors

ROS 2 Jazzy Python package providing three I²C sensor nodes for the Crawler Pi tracked robot, running on a Raspberry Pi 4B.

The package reads battery electrical measurements (INA226), track rotation measurements (two TMAG5273 magnetic sensors), and raw inertial measurements (ICM-20948). It **does not** implement fused orientation, full wheel/track odometry, motor control, or battery state-of-charge estimation.

## Package layout

| File | ROS executable | Purpose |
| --- | --- | --- |
| `crawler_sensors/ina226_node.py` | `ina226_node` | Battery bus voltage, shunt current and power |
| `crawler_sensors/tmag_odom_pair_node.py` | `tmag_odom_pair_node` | Magnet peak counts and track RPM from two TMAG5273 devices |
| `crawler_sensors/icm20948_node.py` | `icm20948_node` | Raw accelerometer and calibrated gyro data |

The Python node implementations live in `crawler_sensors/`; the `setup.py` `console_scripts` entries expose the executables. This README is placed at the ROS package root (alongside `package.xml`).

## Prerequisites and running

- ROS 2 Jazzy with `rclpy`, `std_msgs` and `sensor_msgs`.
- Python `smbus2`, for example `sudo apt install python3-smbus2` on Debian/Ubuntu when available.
- Linux I²C support and permissions to access the appropriate `/dev/i2c-N` device; enable I²C on the Raspberry Pi.
- Sensors connected with correct power, ground, SDA and SCL wiring; verify addresses and avoid duplicate addresses on the same bus.

From the ROS workspace root:

```bash
colcon build --packages-select crawler_sensors --symlink-install
source install/setup.bash
ros2 run crawler_sensors ina226_node
# Or, separately:
ros2 run crawler_sensors tmag_odom_pair_node
ros2 run crawler_sensors icm20948_node
```

These commands use *source-code defaults*. The actual crawler bringup may override parameters via launch files; check that configuration before assuming the defaults match the hardware. Nodes generally read parameters at startup rather than reacting to later `ros2 param set` changes.

For example, explicitly set different magnetic-sensor addresses:

```bash
ros2 run crawler_sensors tmag_odom_pair_node --ros-args \
  -p i2c_bus:=1 -p left_address:=34 -p right_address:=35
```

ROS integer parameter values above are decimal: `34 = 0x22` and `35 = 0x23`. Discover the **real** addresses first; do not assume the defaults apply to another crawler.

Useful checks:

```bash
sudo i2cdetect -y 1
ros2 node list
ros2 topic list
ros2 topic echo /imu
ros2 topic echo /battery/state
ros2 topic echo /odometer/left/rpm
```

Do not scan a bus while applications are actively accessing devices unless you understand the risks; `i2cdetect` is a diagnostic utility and can interfere with some hardware.

## 1. INA226 power-monitoring node

**Implementation:** `crawler_sensors/ina226_node.py`; ROS node name: `ina226_node`.

### Operation

The `Ina226` helper opens the specified SMBus, programs the INA226 calibration register using the configured shunt resistance and maximum expected current, and configures continuous shunt/bus-voltage conversion (`REG_CONFIG = 0x4127`). It provides register reads for bus voltage, shunt voltage, current and power. The ROS node periodically reads **bus voltage**, **current** and **power**, converts them to engineering units and publishes the values.

The `current_sign` parameter multiplies the published current, allowing the sign to be adjusted to the wiring convention. The reported power is the INA226 power-register result; **it is not sign-adjusted by `current_sign`**. Consecutive I²C `OSError`s trigger a bus close, short delay, sensor reopening and reconfiguration. If no sensor is open, a new attempt is made at the next timer callback. Unexpected errors are logged.

### Published topics

| Topic | Type | Meaning / units |
| --- | --- | --- |
| `/battery/voltage` | `std_msgs/msg/Float32` | INA226 bus voltage, volts (V) |
| `/battery/current` | `std_msgs/msg/Float32` | Shunt current, amperes (A), modified by `current_sign` |
| `/battery/power` | `std_msgs/msg/Float32` | INA226 power register converted to watts (W) |
| `/battery/state` | `sensor_msgs/msg/BatteryState` | Timestamped voltage and current; fixed metadata for a present NiMH battery |

The `BatteryState` message uses the configured `frame_id`, sets status to `DISCHARGING`, health to `UNKNOWN`, technology to `NIMH`, and `present = true`. These status fields are hardcoded: the node does **not** detect charging, derive remaining charge percentage, or estimate runtime. Fields not assigned by the node retain their message defaults; consumers should not treat them as independently measured values.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `bus_number` | `1` | Linux `/dev/i2c-N` bus |
| `address` | `0x40` (64) | INA226 7-bit I²C address |
| `shunt_ohms` | `0.002` | Current-shunt resistance, Ω |
| `max_current_a` | `10.0` | Expected full-scale current, A; used for calibration |
| `publish_rate_hz` | `5.0` | Measurement publication rate |
| `frame_id` | `battery` | Header frame on `/battery/state` |
| `current_sign` | `1.0` | Multiplier for reported current |
| `max_i2c_errors_before_reopen` | `2` | Consecutive I²C read errors before reopening |
| `reopen_delay_s` | `0.2` | Reopen pause, seconds |

**Calibration note:** Check the actual installed shunt value and the electrical current range. Incorrect values directly affect calculated current and power. The node exposes a `read_shunt_voltage_v()` helper but does not publish shunt voltage.

## 2. TMAG5273 dual track-rotation node

**Implementation:** `crawler_sensors/tmag_odom_pair_node.py`; ROS node name: `tmag_odom_pair_node`.

### Hardware addressing — important

The node directly opens **two TMAG5273 devices on one I²C bus** and independently reads the left and right tracks. The defaults are:

- **Left sensor:** `left_address = 0x22` (decimal 34)
- **Right sensor:** `right_address = 0x23` (decimal 35)
- **Bus:** `i2c_bus = 1`

These are **configuration defaults, not a guaranteed address assignment**. The real devices must be reachable at **two distinct 7-bit addresses on the selected bus**. Set `left_address` and `right_address` to match the actual hardware, and verify which physical sensor belongs to which track. If both sensors have the same address, they cannot be independently addressed on the same un-multiplexed I²C segment; use an appropriate addressing or bus-isolation arrangement. A device on the wrong address typically prevents node initialization. Sensor replacement or wiring changes may require changes to the launch parameters.

### Operation

`Tmag5273Minimal` optionally verifies the TI manufacturer ID (`0x5449`), then configures each sensor for Z-only magnetic-field measurement, selected conversion averaging, the wide-range Z setting, low-noise mode and continuous operation. It reads a signed 16-bit Z sample and scales it into **microtesla (µT)** according to `range_z_mt`.

For each track, `PeakDetector` applies hysteresis: a sample above `high_threshold` while armed increments the peak counter, and the detector only rearms once the field falls below `low_threshold`. The timestamps of recent peaks estimate **unsigned RPM**:

`RPM = 60 × (number of intervals / elapsed seconds) / magnets_per_rev`

If no peak has appeared for 1.5 seconds, RPM is set to zero; a long gap (over 2 seconds in this node's detector setup) clears the RPM estimation history before new peaks. Revolutions equal `peak_count / magnets_per_rev` and represent cumulative **detected magnetic events**, not signed travel. Neither direction of rotation nor backwards travel is determined by this node. The count is reset when the node restarts. One unsuccessful paired I²C sampling cycle is skipped and logged; this node does not implement the INA226-style auto-reconnect logic.

### Published topics

| Topic | Type | Description |
| --- | --- | --- |
| `/odometer/left/magnetic_z` | `std_msgs/msg/Float32` | Left Z magnetic field, µT |
| `/odometer/left/raw_z` | `std_msgs/msg/Float32` | Left signed 16-bit raw Z register value represented as float |
| `/odometer/left/peak_count` | `std_msgs/msg/UInt64` | Left cumulative positive-threshold crossing count |
| `/odometer/left/rpm` | `std_msgs/msg/Float32` | Left unsigned rotational-speed estimate, RPM |
| `/odometer/left/revolutions` | `std_msgs/msg/Float32` | Left detected revolutions (`peaks / magnets_per_rev`) |
| `/odometer/right/magnetic_z` | `std_msgs/msg/Float32` | Right Z magnetic field, µT |
| `/odometer/right/raw_z` | `std_msgs/msg/Float32` | Right signed raw Z value represented as float |
| `/odometer/right/peak_count` | `std_msgs/msg/UInt64` | Right cumulative positive-threshold crossing count |
| `/odometer/right/rpm` | `std_msgs/msg/Float32` | Right unsigned rotational-speed estimate, RPM |
| `/odometer/right/revolutions` | `std_msgs/msg/Float32` | Right detected revolutions |
| `/odometer/dt_s` | `std_msgs/msg/Float32` | Time between successful paired samples, seconds |

All topics are published once per successful sampling callback (nominally at `publish_rate_hz`). This node **does not** publish `nav_msgs/Odometry` or TF; other crawler components can combine these readings with motion direction and robot geometry.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `i2c_bus` | `1` | Shared bus `/dev/i2c-N` |
| `left_address` | `0x22` (34) | Left 7-bit I²C address |
| `right_address` | `0x23` (35) | Right 7-bit I²C address |
| `publish_rate_hz` | `20.0` | Requested sampling / publish rate |
| `magnets_per_rev` | `4` | Number of detectable magnet peaks per revolution |
| `range_z_mt` | `80.0` | Magnetic full-scale used for raw-to-µT conversion, millitesla |
| `conversion_average` | `2` | Device conversion-averaging setting (`2` means 4× in this implementation) |
| `verify_manufacturer` | `false` | Optionally validate manufacturer ID |
| `left_high_threshold` | `700.0` | Left peak-trigger threshold, µT |
| `left_low_threshold` | `100.0` | Left hysteresis rearm threshold, µT |
| `right_high_threshold` | `700.0` | Right peak-trigger threshold, µT |
| `right_low_threshold` | `100.0` | Right hysteresis rearm threshold, µT |
| `peak_history_size` | `6` | Maximum number of peak timestamps used for RPM estimation |

**Important calibration:** Thresholds and the `range_z_mt` value depend on magnet installation, sensor variant and mounting geometry. Check that the thresholds produce **one event per magnet pass**, not extra or missed events. The wide-range configuration is hardcoded; `range_z_mt` affects scaling but does not automatically select a different physical sensor range.

## 3. ICM-20948 raw IMU node

**Implementation:** `crawler_sensors/icm20948_node.py`; ROS node name: `icm20948_node`.

### Operation

This node reads the ICM-20948 over I²C, verifies `WHO_AM_I == 0xEA`, resets and configures the IMU, enables all accelerometer and gyroscope axes, selects measurement ranges and reads 12 bytes of raw acceleration/gyro readings in a timer callback.

It converts accelerometer raw counts into **m/s²**, gyro raw counts into **rad/s**, and optionally swaps X/Y or inverts individual axes to match the installed sensor orientation. This mapping is performed on both acceleration and angular velocity.

**Stationary startup calibration:** On each successful sensor connection, it waits for `gyro_calibration_settle_s`, collects `gyro_calibration_samples` raw gyroscope readings, and calculates a mean zero-rate offset on each axis. It rejects a calibration if any axis exceeds `gyro_calibration_max_stddev_dps` standard deviation and starts calibration again. **No `/imu` data are published until calibration succeeds.** Keep the crawler and its motors stationary during this stage. A reconnection starts calibration again.

After `max_consecutive_errors` read failures, the node attempts to reconnect; the retry delay is controlled by `reconnect_delay_s`.

### Published topic

| Topic | Type | Description |
| --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | Raw acceleration (m/s²) and gyro-bias-corrected angular velocity (rad/s) |

The `Imu` message uses the configured `frame_id` (default `imu_link`) and a ROS clock timestamp. **No orientation is estimated:** the quaternion is set to identity only as a placeholder, with `orientation_covariance[0] = -1` indicating that orientation is unavailable. A downstream estimator must **not** interpret that quaternion as measured attitude. The angular-velocity and linear-acceleration covariance diagonals are fixed placeholders (`0.02` and `0.2` respectively), not sensor-specific measured covariances. This node does **not** read the IMU magnetometer or provide sensor fusion.

The publisher uses reliable QoS, depth 5. It does not publish during startup calibration or while disconnected.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `bus_number` | `1` | Linux I²C bus |
| `address` | `0x69` (105) | ICM-20948 7-bit address |
| `frame_id` | `imu_link` | Published IMU message frame |
| `publish_rate_hz` | `50.0` | Timer-based ROS publication rate |
| `accel_range_g` | `4` | Full scale ±2, ±4, ±8 or ±16 g |
| `gyro_range_dps` | `500` | Full scale ±250, ±500, ±1000 or ±2000 deg/s |
| `max_consecutive_errors` | `5` | Error threshold for reconnection |
| `reconnect_delay_s` | `1.0` | Minimum time between reconnect attempts |
| `swap_xy` | `false` | Swap sensor X and Y |
| `invert_x` | `false` | Negate X readings |
| `invert_y` | `false` | Negate Y readings |
| `invert_z` | `false` | Negate Z readings |
| `gyro_calibration_samples` | `500` | Samples used to determine zero-rate bias |
| `gyro_calibration_settle_s` | `2.0` | Wait before sampling for calibration, seconds |
| `gyro_calibration_max_stddev_dps` | `0.20` | Maximum allowed gyro sample standard deviation per axis, deg/s |

The node configures measurement dividers but ROS publication is **timer-driven**, so the ROS rate should not be confused with the exact hardware output data rate. Validate frame-axis mapping with the installed physical sensor orientation before fusing IMU and track odometry.

## Useful entry points and responsibilities

### INA226

- `Ina226.__init__()`: opens I²C, calculates/writes calibration, starts continuous conversion.
- `Ina226.read_bus_voltage_v()`, `read_current_a()`, `read_power_w()`: decode sensor registers into SI units.
- `Ina226Node.publish_measurements()`: publishes four ROS topics and fills `BatteryState`.
- `Ina226Node.reopen_sensor()`: recovers from repeated bus errors.

### TMAG5273 pair

- `Tmag5273Minimal.configure_z_only()`: programs magnetic Z measurement mode.
- `Tmag5273Minimal.read_z_raw()` / `read_z_ut()`: reads a signed sample and converts to µT.
- `PeakDetector.update()`: hysteresis, peak counting, RPM timeout and history management.
- `PeakDetector.calculate_rpm()` / `revolutions()`: calculates RPM and cumulative magnet-based revolutions.
- `TmagOdomPairNode.timer_callback()`: reads both sensors and publishes 11 topics.

### ICM-20948

- `Icm20948RawNode.connect_sensor()` / `configure_sensor()`: verifies and initializes the device.
- `start_gyro_calibration()` / `collect_gyro_calibration_sample()`: stationary bias calibration with motion rejection based on gyro variation.
- `timer_callback()`: sensor reading, calibration control, conversion, axis mapping and IMU publishing.
- `convert_accel()` / `convert_gyro()`: physical-unit conversion.
- `fill_covariances()`: flags orientation as unavailable and supplies placeholder covariance values.

## Notes for integration and testing

- The I²C addresses in this README describe **node defaults**, not independently verified crawler wiring. Check launch overrides and actual hardware before deployment.
- Topic names are **absolute** (start with `/`), and so are not automatically scoped by a ROS namespace.
- `smbus2` is imported by all three drivers. The Python package dependency must be installed on the machine; `setup.py` currently only lists `setuptools`.
- Test sensor topics on actual hardware after any changes to launch parameters or I²C wiring. Static Python syntax checks cannot validate live sensor I/O.
- This package was documented without changing the existing node logic.

## License and credits

Licensed under the **GNU General Public License version 3.0 only** (`GPL-3.0-only`); see [LICENSE](LICENSE). Package maintainer: Christian Arrer. Documentation and metadata assistance: ChatGPT (OpenAI). Replace the example `.invalid` contact email in `package.xml` and `setup.py` with a real address before publishing.
