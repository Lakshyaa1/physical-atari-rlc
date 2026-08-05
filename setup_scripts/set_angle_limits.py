#!/usr/bin/env python3
"""Set one Feetech servo's EEPROM min/max angle limits (addr 9 / 11).

The firmware clamps EVERY goal_position write to this range, silently and
regardless of torque. A servo that "will not reach" its target is usually this,
not a torque or wiring problem -- so these need to bracket the real working
travel, centred on the real neutral.

EEPROM: unlock (addr 55 = 0), write, relock (addr 55 = 1), then read back and
verify. Dry run by default. Refuses to run with torque enabled.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from feetech_protocol import FeetechBus

# Default for this robot; override with --path. Prefer a /dev/serial/by-id/
# path, which survives a replug where /dev/ttyACM0 does not.
PATH = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110727-if00"
A_MIN, A_MAX, A_LOCK, A_POS, A_TORQUE = 9, 11, 55, 56, 40


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--servo", type=int, required=True)
    ap.add_argument("--min", type=int, required=True)
    ap.add_argument("--max", type=int, required=True)
    ap.add_argument("--path", default=PATH)
    ap.add_argument("--execute", action="store_true")
    args = ap.parse_args()

    if not (0 <= args.min < args.max <= 4095):
        print("need 0 <= min < max <= 4095", file=sys.stderr)
        return 1

    bus = FeetechBus(args.path, 1000000)
    try:
        if bus.read_byte(args.servo, A_TORQUE):
            print(f"ABORT: torque enabled on {args.servo}; disable it first", file=sys.stderr)
            return 1
        cur_min, cur_max = bus.read_word(args.servo, A_MIN), bus.read_word(args.servo, A_MAX)
        pos = bus.read_word(args.servo, A_POS)
        print(f"servo {args.servo}: present {pos}")
        print(f"  limits {cur_min}..{cur_max}  ->  {args.min}..{args.max}")
        print(f"  span   {cur_max-cur_min}      ->  {args.max-args.min}")
        if not (args.min <= pos <= args.max):
            print(f"  WARNING: present position {pos} is outside the NEW range; "
                  f"the servo would be unable to return to where it is now.")
        if not args.execute:
            print("\n(dry run -- pass --execute to write EEPROM)")
            return 0

        if not bus.unlock_eeprom(args.servo):
            print("failed to unlock EEPROM", file=sys.stderr)
            return 1
        ok_min = bus.write(args.servo, A_MIN, [args.min & 0xFF, (args.min >> 8) & 0xFF])
        ok_max = bus.write(args.servo, A_MAX, [args.max & 0xFF, (args.max >> 8) & 0xFF])
        time.sleep(0.05)
        relocked = bus.lock_eeprom(args.servo)

        got_min, got_max = bus.read_word(args.servo, A_MIN), bus.read_word(args.servo, A_MAX)
        lock = bus.read_byte(args.servo, A_LOCK)
        print(f"  wrote min={ok_min} max={ok_max}, relocked={relocked} (lock reg = {lock})")
        print(f"  readback: {got_min}..{got_max}")
        if (got_min, got_max) != (args.min, args.max):
            print("  MISMATCH -- limits did not take", file=sys.stderr)
            return 1
        print("  verified")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
