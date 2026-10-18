# Server ROS 2 workspace (vm111)

This directory is reserved for the server-side ROS 2 Jazzy packages (navigation, mapping, exploration, future docking/ML). It is intentionally **not populated** with speculative packages yet. Keep existing working `vm111` installs unchanged.

Clone the entire repository on vm111, then work from this directory:

```bash
cd ~/crawler-pi/server
source /opt/ros/jazzy/setup.bash
# Later, when packages have been imported into src/:
# colcon build --symlink-install
```

Third-party components such as frontier exploration, SLAM Toolbox and Nav2 should normally remain external dependencies. Once their precise upstream repository URLs, release branches and installation methods are verified, record them in a reproducible `.repos` manifest or an install/dependency guide. Do not blindly vendor or overwrite the currently installed frontier explorer.
