# Crawler ROS 2 (Jazzy)

Authors: ChatGPT and Christian Arrer <christianarrer@gmail.com>  
License: GPL-3.0-or-later (see LICENSE).

Packages: `crawler_bringup`, `crawler_control`, `crawler_safety`, `crawler_sensors`.

## Build
```bash
cd ~/crawler_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch crawler_bringup crawler.launch.py
```

Configuration: `crawler_bringup/config/crawler.yaml`. ROS 2 parameters can be inspected with `ros2 param list /crawler_velocity_target`.

**Safety:** The current guard and motor driver both access GPIO 5; check their ownership and fail-safe behavior on hardware before operating. Keep tracks lifted during the first test. The INA226 launch file previously set `publish_rate_hz: 0.2` (one sample every 5 s); it is now 5 Hz. The invalid `max_current` parameter has been corrected to `max_current_a`.

**Scope:** Velocity target calibration and kinematic constants are now ROS parameters. Other hardware constants and thresholds still exist in node source; they have not all been migrated. Navigation stack (Nav2, SLAM Toolbox, AMCL) is external to this archive. The documentation describes their integration as conceptual, not as nodes launched here.

**Test checklist:** `python3 -m compileall src`; `colcon build`; `ros2 launch`; check `/tf`, `/tf_static`, `/odom`, `/imu`, `/scan`, motor targets, and cliff fail-safe; validate actual battery sample rate.
