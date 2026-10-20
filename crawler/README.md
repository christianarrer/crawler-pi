# Raspberry Pi ROS 2 workspace

This folder is a standalone colcon workspace. ROS 2 packages live in `src/`; firmware for the Pico lives in `firmware/` and is not built by colcon.

```bash
cd ~/crawler-pi/crawler
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch crawler_bringup crawler.launch.py
```

Packages: crawler_bringup, crawler_control, crawler_safety, crawler_sensors.

Important: Hardware-specific third-party ROS packages, such as the LDS-01 driver, must also be installed or available in the ROS environment before launch.

## Migrating the existing systemd service

Do not assume that cloning to `~/crawler-pi` changes the existing `crawler-ros.service`: it may still source `~/crawler_ws/install/setup.bash`. Review:

```bash
systemctl cat crawler-ros.service
systemctl cat crawler-ros.timer
```

Stop both the timer and service before a test, inspect the actual paths and choose one workspace to run. Keep the old `~/crawler_ws` unchanged as a rollback while verifying the new checkout. After testing, adjust the unit's `WorkingDirectory`, `Environment` and `ExecStart` as applicable, reload the daemon, and restart intentionally. Avoid running two instances of the motor driver.

## Safety notes

- Only motor_driver_node owns motor-enable GPIO 5.
- current_guard_node publishes `/crawler/safety/current_ok`; the motor driver requires a recent `true` status in addition to fresh cliff states and wheel targets.
- Current guard fault latch is in-memory only, **not persistent across process restarts**. This software mechanism is not a hardware emergency-stop.
- IMU position `base_link -> imu_link` is (0, -0.062, 0.017) m; its zero rotation was confirmed from the installation's aligned axes.
- Test with the tracks lifted and with an operator ready to disconnect motor power before trying the current guard.

Configuration is in `src/crawler_bringup/config/crawler.yaml`.
