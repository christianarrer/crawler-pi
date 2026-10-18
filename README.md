# Crawler Pi

Open-source tracked mobile robot using ROS 2 Jazzy. The Raspberry Pi 4B runs sensors, motor control and safety. A separate Linux server (vm111) runs navigation, mapping and exploration.

## Repository layout

- `crawler/` — self-contained ROS 2 colcon workspace for the Raspberry Pi; packages are in `crawler/src/`.
- `crawler/firmware/` — RP2040 cliff-sensor firmware and documentation (not compiled by colcon).
- `server/` — separate ROS 2 colcon workspace for vm111; currently a placeholder because its local packages and dependency revisions have not yet been imported.
- `docs/` — WordPress HTML documentation.

Clone the **whole repository** onto both machines; build only the relevant workspace. Build, install and log outputs are ignored by Git.

## Raspberry Pi initial checkout

```bash
git clone --branch safety-refactor https://github.com/christianarrer/crawler-pi.git ~/crawler-pi
cd ~/crawler-pi/crawler
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Keeping a checkout current

```bash
cd ~/crawler-pi
git status --short
git pull --ff-only
cd crawler                # or: cd server, on vm111
colcon build --symlink-install
```

Do not mix `main` and `safety-refactor` on a running robot without coordinating a deployment.

## Safety / deployment warning

The current guard and motor driver refactor is **untested on hardware**. Keep the robot's tracks lifted for the first tests. Back up the SD card. Stop `crawler-ros.timer` and `crawler-ros.service` before changing the active installation.

The current systemd unit may still use `/home/ros/crawler_ws/install/setup.bash` and related paths. **A repository checkout does not update the systemd unit.** Verify `systemctl cat crawler-ros.service` and migrate or update its workspace paths explicitly. Do not start the service until its launch environment resolves to the intended tested workspace.

For the complete project see the workspace READMEs. License: GPL-3.0-or-later.
