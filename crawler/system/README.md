# Crawler Raspberry Pi system integration

This folder archives the **actual** files uploaded from the Raspberry Pi on 2026-10-20. These are versioned reference copies, not files automatically installed or loaded from this checkout.

- `systemd/`: five systemd units and the ROS startup timer, destined for `/etc/systemd/system/`.
- `scripts/`: four executables, destined for `/usr/local/bin/`.

Before modifying or deploying, review the live units with `systemctl cat` and compare them to the repository copies.

## Startup relationships

- `crawler-ros.timer`: starts `crawler-ros.service` 30 seconds after boot.
- `crawler-odom-init.service`: initializes TMAG5273 addresses; its **`OnSuccess=crawler-ros.service` can also start ROS**, potentially before the timer. This is an intentional copy of the working deployment, but it creates two independent start triggers. Resolve and test before changing it.
- `crawler-ros.service`: requires `pigpiod.service` and `crawler-odom-init.service`; runs `crawler-ros.sh` as user `ros`, with restart on failure.
- `crawler-pwm-init.service`: initializes PWM sysfs channels. **It is not explicitly ordered before crawler-ros.service** in the uploaded units.
- `crawler-headlight-shutdown.service`: shutdown indicator on GPIO 6.

## Important notes

1. A `git pull` does NOT update installed scripts or systemd units. They must be deployed explicitly after inspection, permission checks and `systemctl daemon-reload`. Do not copy over running services without a rollback plan.
2. `crawler-ros.sh` uses absolute paths under `/home/ros/crawler-pi/crawler`, and checks `/dev/i2c-1` and `/dev/ttyUSB0` before launch. The motor driver independently manages cliff Pico serial reconnects at `/dev/ttyACM0`.
3. The original service and shell script set `ROS_LOCALHOST_ONLY=0`; this is deprecated under ROS 2 Jazzy but remains in this snapshot.
4. `crawler-ros.service` uses `Restart=on-failure`. The current guard's overload latch is only in process memory; a guard restart clears it. This is **not a persistent fault latch or hardware E-stop**.
5. A motor safety test must be done with the tracks lifted; confirm default-off GPIO state at startup, guard heartbeat timeout, cliff/motor command watchdogs and shutdown before enabling unattended operation.
6. These files are tied to the current Raspberry Pi OS, `gpiod` API and GPIO wiring. Do not blindly deploy them to other hosts.
