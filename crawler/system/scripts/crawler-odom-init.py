#!/usr/bin/env python3

import time
import sys

import gpiod
from smbus2 import SMBus


I2C_BUS = 1

TMAG_DEFAULT_ADDR = 0x22
TMAG_RIGHT_ADDR = 0x23

# TMAG5273 I2C address register
TMAG_I2C_ADDR_REG = 0x0C

GPIOCHIP = "gpiochip0"

ODO_L_DISABLE_GPIO = 17
ODO_R_DISABLE_GPIO = 27

# DISABLE line logic:
# 1 = disabled
# 0 = enabled
DISABLE = 1
ENABLE = 0


def write_tmag_address(bus: SMBus, old_addr: int, new_addr: int) -> None:
    """
    Set TMAG5273 I2C address.
    TMAG5273: bits 7:1 = 7-bit I2C address, bit 0 = address update enable.
    Example: 0x23 -> (0x23 << 1) | 0x01 = 0x47
    """
    new_reg_value = (new_addr << 1) | 0x01

    old_value = bus.read_byte_data(old_addr, TMAG_I2C_ADDR_REG)
    print(f"TMAG at 0x{old_addr:02x}: old address register value: 0x{old_value:02x}")

    print(
        f"Writing new address register value 0x{new_reg_value:02x} "
        f"to register 0x{TMAG_I2C_ADDR_REG:02x} at old address 0x{old_addr:02x}"
    )

    bus.write_byte_data(old_addr, TMAG_I2C_ADDR_REG, new_reg_value)
    time.sleep(0.2)

    # Do not require the register to read back as 0x47.
    # Your working test script shows it can read back as 0x00 after the address switch.
    if not address_exists(bus, new_addr):
        raise RuntimeError(f"TMAG did not appear at new address 0x{new_addr:02x}")

    print(f"TMAG now reachable at 0x{new_addr:02x}")


def address_exists(bus: SMBus, addr: int) -> bool:
    try:
        bus.read_byte(addr)
        return True
    except OSError:
        return False


def main() -> int:
    print("Starting crawler odometer address init")

    chip = gpiod.Chip(GPIOCHIP)

    line_l = chip.get_line(ODO_L_DISABLE_GPIO)
    line_r = chip.get_line(ODO_R_DISABLE_GPIO)

    # Request both lines as outputs and immediately set both sensors disabled.
    # This is important because the DISABLE lines have no pulldown.
    line_l.request(
        consumer="crawler-odom-init-left",
        type=gpiod.LINE_REQ_DIR_OUT,
        default_vals=[DISABLE],
    )
    line_r.request(
        consumer="crawler-odom-init-right",
        type=gpiod.LINE_REQ_DIR_OUT,
        default_vals=[DISABLE],
    )

    try:
        # Both disabled first.
        line_l.set_value(DISABLE)
        line_r.set_value(DISABLE)
        time.sleep(0.2)

        # Keep left disabled, enable only right sensor.
        line_l.set_value(DISABLE)
        line_r.set_value(ENABLE)
        time.sleep(0.2)

        with SMBus(I2C_BUS) as bus:
            if address_exists(bus, TMAG_RIGHT_ADDR):
                print(f"Right TMAG already reachable at 0x{TMAG_RIGHT_ADDR:02x}")
            elif address_exists(bus, TMAG_DEFAULT_ADDR):
                print(
                    f"Right TMAG reachable at default address 0x{TMAG_DEFAULT_ADDR:02x}, "
                    f"changing to 0x{TMAG_RIGHT_ADDR:02x}"
                )
                write_tmag_address(bus, TMAG_DEFAULT_ADDR, TMAG_RIGHT_ADDR)
            else:
                raise RuntimeError(
                    "Right TMAG not found at 0x22 or 0x23 while left sensor is disabled"
                )

        # Enable both sensors.
        line_l.set_value(ENABLE)
        line_r.set_value(ENABLE)
        time.sleep(0.2)

        with SMBus(I2C_BUS) as bus:
            left_ok = address_exists(bus, TMAG_DEFAULT_ADDR)
            right_ok = address_exists(bus, TMAG_RIGHT_ADDR)

            print(f"Left  TMAG 0x{TMAG_DEFAULT_ADDR:02x}: {'OK' if left_ok else 'MISSING'}")
            print(f"Right TMAG 0x{TMAG_RIGHT_ADDR:02x}: {'OK' if right_ok else 'MISSING'}")

            if not left_ok or not right_ok:
                raise RuntimeError("Final TMAG address check failed")

        print("Crawler odometer address init successful")
        return 0

    finally:
        try:
            print("Cleanup: enabling both odometer sensors")
            line_l.set_value(ENABLE)
            line_r.set_value(ENABLE)
            time.sleep(0.2)
        except Exception as cleanup_exc:
            print(f"Cleanup warning: could not enable both sensors: {cleanup_exc}", file=sys.stderr)

        line_l.release()
        line_r.release()
        chip.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
