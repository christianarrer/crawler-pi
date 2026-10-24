# Crawler Software Setup — Raspberry Pi 4B (ROS 2 Jazzy)

> **Status:** Documented from the clean installation and hardware bring-up performed in October 2026.  
> **Target:** Raspberry Pi 4B (4 GB), Ubuntu Server 24.04.5 LTS (64-bit), ROS 2 Jazzy, Python 3.12.  
> **Repository:** [christianarrer/crawler-pi](https://github.com/christianarrer/crawler-pi) — branch `main`.  
> **Scope:** Onboard Raspberry Pi software. The external navigation/mapping server is **not** covered here.
>
> This is a hardware-specific installation record and a reproducible guide, **not** a guarantee that every software dependency or electrical component will behave identically on other builds. Check the wiring, GPIO pins, current limits and sensor addresses against your own hardware.

## Table of contents

- [1. Overview and precautions](#1-overview-and-precautions)
- [2. Prepare the SD card on Windows](#2-prepare-the-sd-card-on-windows)
- [3. First boot, SSH and Ubuntu packages](#3-first-boot-ssh-and-ubuntu-packages)
- [4. Time synchronization with Chrony](#4-time-synchronization-with-chrony)
- [5. Raspberry Pi hardware interfaces](#5-raspberry-pi-hardware-interfaces)
- [6. pigpio — GPIO and PWM](#6-pigpio--gpio-and-pwm)
- [7. ROS 2 Jazzy and Cyclone DDS](#7-ros-2-jazzy-and-cyclone-dds)
- [8. Clone the repository and install Python dependencies](#8-clone-the-repository-and-install-python-dependencies)
- [9. Build the Crawler ROS workspace](#9-build-the-crawler-ros-workspace)
- [10. USB and I²C hardware discovery](#10-usb-and-ic-hardware-discovery)
- [11. TMAG5273 odometer address initialization](#11-tmag5273-odometer-address-initialization)
- [12. Test sensors, safety, and motor control](#12-test-sensors-safety-and-motor-control)
- [13. Install the ROS systemd service and boot timer](#13-install-the-ros-systemd-service-and-boot-timer)
- [14. Headlight shutdown indicator](#14-headlight-shutdown-indicator)
- [15. Reboot and final verification](#15-reboot-and-final-verification)
- [16. Troubleshooting](#16-troubleshooting)
- [17. Maintenance, limitations, and future work](#17-maintenance-limitations-and-future-work)

---

## 1. Overview and precautions

### 1.1 Architecture

The Raspberry Pi is responsible for local sensors, motor control, odometry, safety checks, and browser-based teleoperation. A separate Linux/ROS 2 machine performs more demanding navigation, SLAM, and exploration tasks.

| Component | Purpose |
| --- | --- |
| Ubuntu Server 24.04 LTS | 64-bit Linux operating system and systemd service manager |
| ROS 2 Jazzy | Nodes, topics, transforms and launch-system integration |
| Cyclone DDS | ROS 2 discovery and message transport over the network |
| Python 3 virtual environment | Isolates additional Python packages such as Gradio |
| `colcon` | Builds the four Crawler ROS 2 packages |
| `pigpio` / `pigpiod` | Hardware/software PWM and GPIO motor control |
| INA226 | Battery voltage, current and power monitoring |
| ICM-20948 | Inertial measurements (accelerometer/gyroscope) |
| 2 × TMAG5273 | Magnetic track-odometry measurements |
| LDS-01 lidar | Laser scans on `/scan` |
| Raspberry Pi Pico / RP2040 | Four cliff-distance sensors, sent via USB serial |
| `systemd` | Starts prerequisites, ROS 2 and shutdown indicator |

Repository layout:

```text
crawler-pi/
├── README.md
├── crawler/
│   ├── docs/crawler-software-setup.md    <-- this guide
│   ├── firmware/cliff_sensors/            <-- Pico firmware
│   ├── src/
│   │   ├── crawler_bringup/
│   │   ├── crawler_control/
│   │   ├── crawler_safety/
│   │   └── crawler_sensors/
│   └── system/
│       ├── raspberry-pi/config.txt
│       ├── scripts/
│       └── systemd/
└── server/                                <-- separate workspace
```

### 1.2 Electrical and safety prerequisites

1. **Keep both tracks lifted** during initial motor tests. Be able to cut motor power immediately.
2. Check that the power converter produces a safe, stable Pi supply. A software shutdown or current guard is **not** a hardware emergency stop.
3. Ensure the motor driver's enable line defaults to the **disabled** state. Here motor enable is **GPIO 5**.
4. Verify your wiring against the code before attaching powered motors: motor PWM uses GPIO **12/16 (left)** and **13/26 (right)**; the two TMAG disable signals use GPIO **17/27**; the headlight indicator uses GPIO **6**.
5. The tests in this guide were carried out with the Crawler supported off the ground, first using a laboratory power supply. Unattended operation and fault-condition validation remain separate tasks.

**Network security:** Enable SSH for headless operation, protect the `ros` account with a strong password, and preferably place the robot on a trusted/segmented robotics VLAN. Do not expose ROS 2 discovery or the teleoperation UI directly to the public internet.

## 2. Prepare the SD card on Windows

### 2.1 Flash Ubuntu using Raspberry Pi Imager 2.0.12

1. Start **Raspberry Pi Imager 2.0.12** on Windows.
2. **Device:** select *Raspberry Pi 4* (or your actual Raspberry Pi model).
3. **Operating System:** *Other general-purpose OS → Ubuntu → Ubuntu Server 24.04.5 LTS (64-bit)*.
4. **Storage:** choose the SDXC card carefully (for example, a drive shown by Windows as `D:\`). **The selected drive is erased.**
5. In OS customization, configure:
   - **Hostname:** `crawler-pi`.
   - **Localization:** your timezone (in this installation, `Europe/Vienna`) and keyboard layout.
   - **Username:** `ros`, with a strong password.
   - **Wi-Fi:** enter the SSID and passphrase that the Pi will use inside the robot.
   - **SSH:** enable SSH; the Pi's HDMI connector may be inaccessible after assembly.
6. Write and verify the card using Imager. If Windows subsequently offers to format unfamiliar Linux partitions, **decline**: Ubuntu partitions are expected and must not be reformatted.
7. Insert the card into the **powered-off** Raspberry Pi and boot it.

These settings allow headless installation without needing a display and keyboard inside the vehicle.

### 2.2 Find the Raspberry Pi and connect

Use your router/DHCP server to find `crawler-pi`, or try:

```powershell
ssh ros@crawler-pi
```

If hostname resolution is not available, use the assigned IP address:

```powershell
ssh ros@<RASPBERRY_PI_IP>
```

On first contact, verify/accept the new SSH host key. Wait for Ubuntu cloud-init to finish before installing packages:

```bash
cloud-init status --wait
hostnamectl
ip -br address
```

## 3. First boot, SSH and Ubuntu packages

### 3.1 Verify Ubuntu apt repositories

During the October 2026 installation, `noble-updates` was **missing** from `/etc/apt/sources.list.d/ubuntu.sources`. This caused incompatible dependency versions (including `libacl1`, `liblz4-1` and `libzstd1`). Verify it **before** installing ROS 2:

```bash
cat /etc/apt/sources.list.d/ubuntu.sources
```

The relevant Ubuntu stanza must include at least the Ubuntu release and updates pocket, for example:

```text
Suites: noble noble-updates
```

Keep the existing `URIs`, `Components`, and `Signed-By` entries, and any separate `noble-security` stanza. If `noble-updates` is absent, edit the file:

```bash
sudo nano /etc/apt/sources.list.d/ubuntu.sources
```

**Why?** The updates repository provides compatible security/bug-fix dependency versions that the original image's installed packages may already expect.

### 3.2 Update the base system

```bash
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y \
  git curl wget ca-certificates gnupg lsb-release \
  build-essential make pkg-config \
  python3-pip python3-venv python3-smbus2 python3-serial \
  python3-libgpiod gpiod i2c-tools
sudo reboot
```

Reconnect over SSH. Package purposes:

- `git`: obtains and updates the Crawler source code.
- `curl`, `wget`, `ca-certificates`, `gnupg`: download and verify repositories/keys.
- `build-essential`, `make`: compile `pigpio`.
- `python3-venv`, `python3-pip`: install application-only Python dependencies.
- `python3-smbus2`, `i2c-tools`: access and inspect I²C sensors.
- `python3-serial`: talk to the RP2040 cliff controller.
- `python3-libgpiod`, `gpiod`: manage Linux GPIO lines, including odometer initialization.

## 4. Time synchronization with Chrony

Accurate clocks matter for ROS 2 sensor timestamps, synchronization, logs and networked nodes.

```bash
sudo apt install -y chrony
chronyc sources -v
chronyc tracking
```

For the recorded installation, the preferred local time server was `192.168.21.11` (the local infrastructure machine `vm011`). On an equivalent network, configure your own reachable time source in `/etc/chrony/chrony.conf`, for example:

```conf
server 192.168.21.11 iburst prefer
```

Remove/comment any conflicting default pool entries **only if you intentionally want the Pi to use that local time source exclusively**. For a portable build, retain a valid fallback NTP pool.

```bash
sudo systemctl restart chrony
chronyc sources -v
chronyc tracking
```

**Why?** A well-synchronized Pi and ROS server reduce confusing timestamp and TF problems. The example IP is site-specific; do not copy it to an unrelated network.

## 5. Raspberry Pi hardware interfaces

### 5.1 Configure the boot firmware

Edit the actual Ubuntu boot configuration:

```bash
sudo nano /boot/firmware/config.txt
```

Under an `[all]` section, include the following relevant settings:

```ini
[all]
# I2C bus for INA226, IMU and TMAG5273 sensors
dtparam=i2c_arm=on,i2c_arm_baudrate=65536

# SPI enabled for potential peripherals
dtparam=spi=on

# Use pigpio hardware PWM, NOT the conflicting Linux PWM overlay
#dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4

# Headlights off at boot (LOW = OFF on this wiring)
gpio=6=op,dl
```

Do **not** overwrite the whole stock `config.txt`; preserve Ubuntu's other model-specific and boot settings. The I²C speed `65536` Hz is the deliberately conservative value used by this particular build; it is not a general requirement for every sensor board. The corresponding version-controlled reference is [`crawler/system/raspberry-pi/config.txt`](../system/raspberry-pi/config.txt).

**Why disable the PWM overlay?** The motor driver uses the **pigpio daemon's** `hardware_PWM()` on GPIO 12/13. The separate kernel PWM/sysfs init mechanism was removed from the repository; do not reinstall its obsolete `crawler-pwm-init.service`.

Reboot after firmware changes:

```bash
sudo reboot
```

### 5.2 Permissions and interface checks

```bash
sudo usermod -aG i2c,dialout ros
```

Log out and log back in (or reboot) for group changes to take effect.

```bash
groups
ls -l /dev/i2c-1
i2cdetect -y 1
gpiodetect
```

`dialout` grants access to USB serial hardware such as the lidar adapter and RP2040; `i2c` grants non-root sensor-bus access. If your distribution uses different device permissions, inspect them before adjusting groups.

## 6. pigpio — GPIO and PWM

`pigpio` controls the motor PWM outputs and GPIO enable line. The Crawler motor driver connects to `pigpiod`, so **the daemon must be running before the ROS motor node starts**.

### 6.1 Build pigpio v79 from upstream source

The clean installation compiled the `joan2937/pigpio` source at v79 rather than relying on an unavailable/incompatible package:

```bash
cd ~
git clone https://github.com/joan2937/pigpio.git
cd pigpio
git checkout v79
make -j2
sudo make install
```

If the Git tag `v79` cannot be resolved in a future upstream checkout, check the upstream tags and install the version you actually validate; do not silently substitute an untested API version.

### 6.2 Install the pigpiod systemd unit

Create:

```bash
sudo nano /etc/systemd/system/pigpiod.service
```

Contents:

```ini
[Unit]
Description=pigpio daemon
After=network.target

[Service]
Type=forking
ExecStart=/usr/local/bin/pigpiod -l
ExecStop=/usr/local/bin/pigpiod -x
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

The `-l` option restricts daemon command connections to local clients, which is appropriate when the ROS motor driver runs on the same Pi.

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now pigpiod.service
systemctl status pigpiod.service --no-pager
pigs t
python3 -c 'import pigpio; p=pigpio.pi(); print("connected:",p.connected,"pigpio:",p.get_pigpio_version() if p.connected else "n/a"); p.stop()'
```

Expected: an active daemon, a functioning `pigs` command and pigpio version 79 in the verified environment.

**Note:** Do not enable the legacy PWM-sysfs initialization mechanism alongside pigpio. Only one well-defined control path should own the motor outputs.

## 7. ROS 2 Jazzy and Cyclone DDS

ROS 2 provides standardized message types, publish/subscribe topics, node lifecycle, transforms and ROS launch files. `rmw_cyclonedds_cpp` selects the DDS middleware used across the Pi and the external ROS server.

### 7.1 Add the ROS 2 package repository

Install Ubuntu Universe support:

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
```

ROS 2's recommended apt-source packaging may evolve; consult the [official ROS 2 Jazzy Ubuntu installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for current repository setup. One direct repository configuration for **Ubuntu Noble (24.04)** is:

```bash
sudo mkdir -p /usr/share/keyrings
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update
```

Use either the official source-package workflow **or** the direct source entry, not duplicate entries for the same repository.

### 7.2 Install core ROS components

```bash
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-dev-tools \
  ros-jazzy-hls-lfcd-lds-driver
```

- `ros-jazzy-ros-base`: ROS command-line utilities, runtime nodes and core libraries without the desktop GUI.
- `ros-jazzy-rmw-cyclonedds-cpp`: Cyclone DDS transport implementation for network discovery and data exchange.
- `ros-dev-tools`: development utilities including `colcon` and ROS dependency helpers.
- `ros-jazzy-hls-lfcd-lds-driver`: publishes LDS-01 laser scan data as `sensor_msgs/LaserScan` on `/scan`.

### 7.3 Interactive ROS environment

Append the following to `~/.bashrc` **only once**:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=21
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Reload it:

```bash
source ~/.bashrc
echo "$ROS_DISTRO $ROS_DOMAIN_ID $RMW_IMPLEMENTATION"
```

**Why domain 21?** A shared ROS domain identifier limits accidental discovery between unrelated ROS installations on the same network. It is **not** an access-control/security boundary; use network segmentation and firewalls for that.

## 8. Clone the repository and install Python dependencies

### 8.1 Clone the whole project

```bash
cd ~
git clone --branch main https://github.com/christianarrer/crawler-pi.git
cd ~/crawler-pi
git status -sb
```

Use `main` for this guide. Old instructions referring to the `safety-refactor` branch are historical and should not be used for the clean installation.

### 8.2 Create the Python virtual environment

```bash
python3 -m venv --system-site-packages ~/crawler-venv
source ~/crawler-venv/bin/activate
python -m pip install --upgrade pip
python -m pip install 'gradio==6.30.0' cffi
python -m pip check
```

- `--system-site-packages` allows the venv to use Ubuntu's ROS-installed Python modules as well as apt-installed sensor modules.
- `gradio` provides the Crawler's browser-based teleoperation interface.
- `cffi` satisfies a missing Python dependency identified during the clean setup.
- `pip check` checks declared Python dependency compatibility.

The version `6.30.0` records the one installed and tested in October 2026; a future release may require application changes. Do not upgrade the Gradio major version without testing the teleop node.

Check imports:

```bash
python -c 'import gradio, serial, smbus2, pigpio, gpiod; print("Python hardware/UI imports OK")'
```

Keep the venv **outside** the Git checkout. It is machine-local generated state.

## 9. Build the Crawler ROS workspace

### 9.1 Dependencies and build

Start a shell with the ROS and Python environments:

```bash
source /opt/ros/jazzy/setup.bash
source ~/crawler-venv/bin/activate
cd ~/crawler-pi/crawler

rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m colcon build --executor sequential --symlink-install
source install/setup.bash
```

`rosdep` resolves declared system dependencies. `colcon` discovers and builds the ROS packages under `crawler/src`; sequential execution is gentler on the memory and power budget of a Raspberry Pi 4B. The `--symlink-install` setting helps with iterative development because Python code in the workspace can be symlinked instead of copied.

> The original `rosdep install --simulate` reported no additional packages. We still recommend running the real check as dependencies can change.

### 9.2 Verify package discovery

```bash
ros2 pkg list | grep '^crawler_'
ros2 pkg executables crawler_control
```

Expected packages:

```text
crawler_bringup
crawler_control
crawler_safety
crawler_sensors
```

The motor-driver **executable** is `motor_driver`, not `motor_driver_node`. Use the executable names from `setup.py` when invoking `ros2 run`.

The shared hardware configuration is:

[`crawler/src/crawler_bringup/config/crawler.yaml`](../src/crawler_bringup/config/crawler.yaml)

Do not reuse calibration constants or sensor addresses blindly on a different physical Crawler.

### 9.3 Note on Python entry points

ROS Python console scripts have an interpreter path recorded at build time. Check the generated teleop script:

```bash
head -n 1 ~/crawler-pi/crawler/install/crawler_control/lib/crawler_control/teleop
```

If its Python interpreter does not have Gradio available, activate the venv and rebuild the workspace; do not assume that sourcing the venv later overrides an executable's fixed shebang. The launch service also activates the venv explicitly.

## 10. USB and I²C hardware discovery

### 10.1 I²C addresses

```bash
i2cdetect -y 1
```

After the odometer initialization in §11, the tested devices are:

| Address | Device | Meaning |
| --- | --- | --- |
| `0x22` | TMAG5273 | Left odometer |
| `0x23` | TMAG5273 | Right odometer |
| `0x40` | INA226 | Battery/current sensor |
| `0x69` | ICM-20948 | IMU |

**Before** TMAG initialization, both odometers can use the same default address `0x22`, which creates an I²C address collision. The right sensor must therefore be isolated and reassigned first.

### 10.2 USB serial devices

```bash
ls -l /dev/ttyUSB0 /dev/ttyACM0
ls -l /dev/serial/by-id/ 2>/dev/null || true
lsusb
```

The tested hardware mapping was:

- `/dev/ttyUSB0`: LDS-01 lidar, Silicon Labs CP2102 USB-to-serial adapter (USB ID `10c4:ea60`).
- `/dev/ttyACM0`: Raspberry Pi Pico/RP2040 cliff-sensor controller (`2e8a:f00a`).

The Crawler launch configuration currently expects `/dev/ttyUSB0` for the lidar, and the motor driver expects `/dev/ttyACM0` for cliffs. Linux may assign different USB port numbers after hardware changes; persistent udev symlinks are a possible future improvement.

## 11. TMAG5273 odometer address initialization

Two TMAG5273 sensors start at the same I²C address. The initialization script controls the **left and right disable pins** (GPIO 17 and 27), isolates the right sensor, changes its address from `0x22` to `0x23`, then enables both sensors.

The right address is changed by writing `0x47` to register `0x0C`: `(0x23 << 1) | 1`. This is hardware-specific and depends on the code in the repository.

### 11.1 Install and test the script

```bash
sudo install -m 755 \
  ~/crawler-pi/crawler/system/scripts/crawler-odom-init.py \
  /usr/local/bin/crawler-odom-init.py

sudo /usr/local/bin/crawler-odom-init.py
i2cdetect -y 1
```

Expected: the script completes successfully and the I²C scan shows **both** `0x22` and `0x23`.

### 11.2 Install the oneshot systemd unit

```bash
sudo install -m 644 \
  ~/crawler-pi/crawler/system/systemd/crawler-odom-init.service \
  /etc/systemd/system/crawler-odom-init.service

sudo systemctl daemon-reload
sudo systemctl enable crawler-odom-init.service
sudo systemctl start crawler-odom-init.service
systemctl status crawler-odom-init.service --no-pager
```

`active (exited)` is normal for a successful `Type=oneshot` service with `RemainAfterExit=yes`.

Important: The unit must **not** contain an active `OnSuccess=crawler-ros.service` trigger. ROS startup is managed separately by the timer described below. Test a **cold power cycle** as well as a warm reboot when validating address initialization on a new build.

## 12. Test sensors, safety, and motor control

> **Do these tests before installing/starting the complete launch service.** Use several SSH terminals and source the environment in each:
>
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/crawler-venv/bin/activate
> source ~/crawler-pi/crawler/install/setup.bash
> ```
>
> Stop each individual `ros2 run` with `Ctrl+C` before launching the complete system. Never run duplicate motor drivers or sensor nodes.

### 12.1 Power measurement with INA226

Terminal A:

```bash
ros2 run crawler_sensors ina226_node --ros-args \
  --params-file ~/crawler-pi/crawler/src/crawler_bringup/config/crawler.yaml
```

Terminal B:

```bash
ros2 topic echo /battery/voltage --once
ros2 topic echo /battery/current --once
ros2 topic echo /battery/power --once
sudo vcgencmd get_throttled
```

The battery measurement reported approximately **7.805 V** while the laboratory supply was set near **7.98 V**. A difference at the INA226 sensing point does not automatically indicate sensor miscalibration: wiring, switches and protection components can produce voltage drops.

`throttled=0x0` means the Raspberry Pi reported no present or latched undervoltage/throttling condition since boot. Recheck under motor load and on the actual battery.

### 12.2 IMU (ICM-20948)

```bash
ros2 run crawler_sensors icm20948_node --ros-args \
  --params-file ~/crawler-pi/crawler/src/crawler_bringup/config/crawler.yaml
```

In another terminal:

```bash
ros2 topic echo /imu --once
ros2 topic hz /imu
```

The observed publication rate was about **50 Hz**; stationary acceleration was close to gravity on the Z axis. The IMU publishes accelerometer/gyroscope data; a placeholder quaternion should **not** be interpreted as a fused absolute orientation. Verify physical IMU frame orientation before relying on it for navigation.

### 12.3 Left and right odometers

```bash
ros2 run crawler_sensors tmag_odom_pair_node --ros-args \
  --params-file ~/crawler-pi/crawler/src/crawler_bringup/config/crawler.yaml
```

Check:

```bash
ros2 topic echo /odometer/left --once
ros2 topic echo /odometer/right --once
```

Both sensors should publish. At rest, their calculated RPM should be zero or approximately zero. A later controlled motor test is required to validate magnet peak counting and directional odometry.

### 12.4 LDS-01 lidar

```bash
ros2 run hls_lfcd_lds_driver hlds_laser_publisher --ros-args \
  --params-file ~/crawler-pi/crawler/src/crawler_bringup/config/crawler.yaml
```

In another terminal:

```bash
ros2 topic echo /scan --once
```

The lidar publishes polar range samples under the `base_scan` frame. Its mounting transform and field of view must remain unobstructed.

### 12.5 Cliff-controller serial data

The Pico firmware is stored in [`crawler/firmware/cliff_sensors/`](../firmware/cliff_sensors/). It handles four VL53L4CD time-of-flight sensors over a TCA9548A I²C multiplexer and streams records at 115200 baud to the Raspberry Pi.

To inspect raw data, **stop the motor driver first** so that only one process opens `/dev/ttyACM0`:

```bash
python3 - <<'PY'
import serial
with serial.Serial('/dev/ttyACM0', 115200, timeout=2) as port:
    for _ in range(10):
        print(port.readline().decode(errors='replace').strip())
PY
```

Records start with `D` and include timestamp plus four distance/status/cliff-state triplets. On the test bench, all four reported cliff state `0` (**SAFE**). A physical edge and sensor-loss test still needs deliberate validation.

### 12.6 Current Guard

Start INA226 in one terminal as in §12.1, then:

```bash
ros2 run crawler_safety current_guard_node
```

In a third terminal:

```bash
ros2 topic echo /crawler/safety/current_ok --once
```

Normal output with fresh sensor measurements is `data: true`. Stop INA226 to test the timeout: the guard should switch to `false`. Restore INA226 before any motor test.

The safety node monitors reported current and data freshness. Its overload latch is **in-process memory only**, resets when the node restarts, and is **not a hardware emergency stop**. Do not deliberately force a high-current fault just to test it without a separately approved safe test procedure.

### 12.7 Motor driver: lifted tracks only

With both tracks off the ground, a way to disconnect motor power, INA226 running and Current Guard running:

```bash
ros2 run crawler_control motor_driver --ros-args \
  --params-file ~/crawler-pi/crawler/src/crawler_bringup/config/crawler.yaml
```

The driver opens `/dev/ttyACM0`, receives the four cliff states and subscribes to both track targets and `/crawler/safety/current_ok`.

Check initially:

```bash
pigs r 5
ros2 topic echo /crawler/cliff_sensors --once
```

With no targets, **GPIO 5 should be LOW (`0`)**, and the driver should block output. Brief startup warnings about stale guard data or missing targets can be expected before the first valid messages arrive.

**Both** motor-target topics must be published continuously. Publishing only the left target is insufficient; the motor driver requires a fresh left/right target pair.

Terminal A, left track at a low command:

```bash
ros2 topic pub -r 5 /crawler/motor/left/target \
  std_msgs/msg/Float32 '{data: 0.3}'
```

Terminal B, right track stationary:

```bash
ros2 topic pub -r 5 /crawler/motor/right/target \
  std_msgs/msg/Float32 '{data: 0.0}'
```

The tested installation successfully moved the left track with this command pair. During valid safe commands, GPIO 5 may go HIGH (`1`). Stop both publishers with `Ctrl+C`; after the motor-target watchdog expires, output must stop and GPIO 5 must return LOW:

```bash
pigs r 5
```

Do not mistake this basic successful test for verification of all cliff, overcurrent, collision or unattended-operation scenarios.

## 13. Install the ROS systemd service and boot timer

The verified startup chain is:

```text
pigpiod.service -------+
                       +--> crawler-ros.service --> crawler.launch.py
crawler-odom-init -----+             ^
                                     |
                          crawler-ros.timer
                           OnBootSec=30
```

The service runs as user `ros`. Its launch script activates ROS 2, the venv and the built Crawler workspace, and waits for I²C and the lidar serial port.

### 13.1 Install script and unit

```bash
sudo install -m 755 \
  ~/crawler-pi/crawler/system/scripts/crawler-ros.sh \
  /usr/local/bin/crawler-ros.sh

sudo install -m 644 \
  ~/crawler-pi/crawler/system/systemd/crawler-ros.service \
  /etc/systemd/system/crawler-ros.service

sudo systemctl daemon-reload
```

The installed `crawler-ros.sh` contains:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ros/crawler-venv/bin/activate
source /home/ros/crawler-pi/crawler/install/setup.bash
```

The unit sets `ROS_DOMAIN_ID=21` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. It does **not** use the deprecated `ROS_LOCALHOST_ONLY` environment variable. For normal subnet discovery, default ROS 2 Jazzy behavior is sufficient; restrict discovery explicitly when deploying to different networks.

### 13.2 Test the service manually before enabling autostart

Stop all individually launched ROS nodes and motor publishers first.

```bash
sudo systemctl start crawler-ros.service
systemctl status crawler-ros.service --no-pager
journalctl -u crawler-ros.service -b -n 100 --no-pager
ros2 node list
```

Also test:

```bash
ros2 topic echo /battery/voltage --once
ros2 topic echo /crawler/safety/current_ok --once
ros2 topic echo /crawler/cliff_sensors --once
sudo vcgencmd get_throttled
```

Confirm that the teleoperation browser interface actually works before proceeding. Do not enable automatic movement or unattended teleoperation merely because the service starts.

### 13.3 Install the 30-second boot timer

```bash
sudo install -m 644 \
  ~/crawler-pi/crawler/system/systemd/crawler-ros.timer \
  /etc/systemd/system/crawler-ros.timer

sudo systemctl daemon-reload
sudo systemctl disable crawler-ros.service
sudo systemctl enable crawler-ros.timer

systemctl is-enabled crawler-ros.service
systemctl is-enabled crawler-ros.timer
```

Expected:

```text
disabled
enabled
```

The timer starts the service approximately **30 seconds after boot**, not 30 seconds after Wi-Fi becomes ready. The service separately waits for its prerequisite units and devices. `enable` prepares the timer for subsequent boots; `systemctl start crawler-ros.timer` can also activate it in the current boot, possibly immediately if the 30-second boot offset has already passed.

**Only the timer is enabled**, not the ROS service, to avoid two independent autostart mechanisms.

## 14. Headlight shutdown indicator

The current Crawler wiring uses GPIO 6 for the headlights. The boot configuration sets this pin LOW to keep the lights off during startup. A systemd oneshot unit stays active until shutdown and then runs the headlight script via `ExecStop`.

### 14.1 Install and enable

```bash
sudo install -m 755 \
  ~/crawler-pi/crawler/system/scripts/crawler-headlight-shutdown.sh \
  /usr/local/bin/crawler-headlight-shutdown.sh

sudo install -m 644 \
  ~/crawler-pi/crawler/system/systemd/crawler-headlight-shutdown.service \
  /etc/systemd/system/crawler-headlight-shutdown.service

sudo systemctl daemon-reload
sudo systemctl enable --now crawler-headlight-shutdown.service
systemctl status crawler-headlight-shutdown.service --no-pager
command -v gpioset
```

`active (exited)` is correct for this `Type=oneshot` unit with `RemainAfterExit=yes`.

### 14.2 Known limitation

The shutdown script attempts five flashes and then drives GPIO 6 HIGH. **The flashes were not fully reliable in the October 2026 tests**, although the headlights lit near the end of shutdown.

The final LED state is only an *informal visual indicator*. Because the script uses the old `gpioset` command pattern, the GPIO line's state after the program exits may not be guaranteed on every kernel/gpiod version. **Do not treat LEDs-on as proof that all SD-card writes are complete or that it is safe to cut power.** Wait until the Raspberry Pi has fully halted (and consider a future positive power-safe signal/controlled power-path circuit).

## 15. Reboot and final verification

### 15.1 Controlled reboot and auto-start test

After the services and timer are installed, reboot:

```bash
sudo reboot
```

Reconnect over SSH, then:

```bash
systemctl status pigpiod.service --no-pager
systemctl status crawler-odom-init.service --no-pager
systemctl status crawler-ros.timer --no-pager
systemctl status crawler-ros.service --no-pager
systemctl status crawler-headlight-shutdown.service --no-pager
ros2 node list
sudo vcgencmd get_throttled
```

After the boot timer has elapsed, the ROS service should be `active (running)`; the oneshot odometer and headlight units may be `active (exited)`. The verified installation ended with `throttled=0x0` under its test power conditions.

Inspect logs if anything failed:

```bash
journalctl -u crawler-ros.service -b --no-pager
journalctl -u crawler-odom-init.service -b --no-pager
journalctl -u pigpiod.service -b --no-pager
```

### 15.2 SD card filesystem state

```bash
lsblk -f
findmnt /
sudo tune2fs -l /dev/mmcblk0p2 | grep -E \
  'Filesystem state|Filesystem error count|Last checked'
```

The recorded Ubuntu root filesystem was `ext4` on `/dev/mmcblk0p2`, with `Filesystem state: clean`, about 3% used, and approximately 108 GB free. Verify the actual device with `findmnt` before assuming another SD card has the same partition layout.

**A reported clean filesystem is not a full offline `fsck`.** Never run a repairing `fsck` against the mounted root filesystem. For an offline integrity check, shut down the Pi and inspect the SD card from another Linux system or a suitable recovery environment.

## 16. Troubleshooting

| Symptom | Check / solution |
| --- | --- |
| Pi never appears on the network | Verify SD card is physically inserted, Wi-Fi configuration, router leases and `cloud-init` status. |
| Apt reports broken dependencies or conflicting library versions | Check whether `noble-updates` is included in `ubuntu.sources`; run `sudo apt update`. |
| `pigpio` cannot connect | `systemctl status pigpiod`; `pigs t`; inspect the pigpio Python import. |
| Kernel PWM conflicts | Keep the `pwm-2chan` device-tree overlay commented and do not install an obsolete PWM-sysfs service. |
| I²C sensors missing | Check `/dev/i2c-1`, wiring, I²C permissions, `i2cdetect -y 1`. |
| Only one TMAG or both at `0x22` | Run odometer init with the correct GPIO 17/27 wiring; check unit logs. |
| Lidar publishes no `/scan` | Check USB enumeration, `dialout` membership and `crawler.yaml` port mapping. |
| Pico cliff data missing | Inspect `/dev/ttyACM0`; avoid opening it simultaneously from the test script and motor driver. |
| `ros2 run ... motor_driver_node` fails | Correct executable: `ros2 run crawler_control motor_driver`. |
| Motors do not move when sending only one target | Publish **both** left and right targets continuously; check current guard and cliff states. |
| Motor unexpectedly stopped | Check command watchdog, current guard freshness, cliff states, and GPIO 5 enable level. |
| Teleop cannot import Gradio | Check `~/crawler-venv`, console-script interpreter shebang, and rebuild from the active venv. |
| Service works manually but not after boot | Inspect `systemctl status crawler-ros.timer`, prerequisite services and `journalctl -u crawler-ros.service -b`. |
| LEDs fail to blink fully at shutdown | Existing shutdown GPIO implementation is provisional; do not use it as a verified safe-power-off indicator. |
| `vcgencmd get_throttled` is nonzero | Investigate 5 V rail voltage, power conversion, battery state, cabling, and load transients. |

## 17. Maintenance, limitations, and future work

### 17.1 Updating from GitHub

A `git pull` updates the checked-out files but does **not** automatically replace installed systemd units or scripts.

For a normal source update:

```bash
sudo systemctl stop crawler-ros.timer
sudo systemctl stop crawler-ros.service

cd ~/crawler-pi
git status --short
git pull --ff-only

source /opt/ros/jazzy/setup.bash
source ~/crawler-venv/bin/activate
cd ~/crawler-pi/crawler
python3 -m colcon build --executor sequential --symlink-install
source install/setup.bash
```

For changes under `crawler/system/`, separately re-deploy the relevant files to `/usr/local/bin/` and `/etc/systemd/system/` (as documented in §§11–14), and then:

```bash
sudo systemctl daemon-reload
sudo systemctl start crawler-ros.service
sudo systemctl start crawler-ros.timer
```

Review running nodes and logs. Starting the timer while the service is already active does not create a second service instance, but avoid parallel manually launched ROS nodes.

### 17.2 Verified vs. not yet fully verified

**Verified in the clean installation:**

- ROS 2 Jazzy node startup, Cyclone DDS domain 21, and the Gradio teleoperation interface.
- `pigpiod` v79, I²C bus access, sensor topics, LDS-01 `/scan`.
- Both TMAG5273 addresses after initialization.
- Current-guard `true` on fresh INA226 data and `false` on INA226 timeout.
- Cliff telemetry from all four Pico sensors in the bench-safe state.
- Motor actuation using a simultaneous left/right target pair with tracks raised.
- ROS systemd service and boot-timer operation, stable test-power status `throttled=0x0`, and reported clean ext4 state.

**Remaining engineering work:**

- Repeat cold-boot, low-battery, heavy-load and supply-transient tests.
- Validate actual cliff-edge response, overcurrent trip behavior and e-stop independently.
- Calibrate IMU frame orientation, wheel/track odometry and motor response on the final chassis.
- Replace provisional shutdown LED logic with a verified power-safe indication.
- Consider persistent serial-device names, more robust service health checks and locked/pinned application dependencies.
- Document Pico firmware flashing, the electrical hardware assembly and the **separate** ROS navigation server in dedicated guides.

---

**License and attribution:** Crawler project: GPL-3.0-or-later. See the repository [LICENSE](../../LICENSE). Documentation prepared from the project's code, manual installation notes and recorded hardware tests by Christian Arrer and ChatGPT.
