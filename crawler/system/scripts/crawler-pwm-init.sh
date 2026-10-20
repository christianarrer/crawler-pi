#!/usr/bin/env bash
set -euo pipefail

PWMCHIP=/sys/class/pwm/pwmchip0
GROUP=gpio
PERIOD_NS=20000000

for CH in 0 1; do
    if [ ! -d "$PWMCHIP/pwm$CH" ]; then
        echo "$CH" > "$PWMCHIP/export"
        sleep 0.2
    fi

    echo 0 > "$PWMCHIP/pwm$CH/enable" || true
    echo "$PERIOD_NS" > "$PWMCHIP/pwm$CH/period"
    echo 0 > "$PWMCHIP/pwm$CH/duty_cycle"

    chgrp "$GROUP" "$PWMCHIP/pwm$CH"/period "$PWMCHIP/pwm$CH"/duty_cycle "$PWMCHIP/pwm$CH"/enable
    chmod g+rw "$PWMCHIP/pwm$CH"/period "$PWMCHIP/pwm$CH"/duty_cycle "$PWMCHIP/pwm$CH"/enable
done
