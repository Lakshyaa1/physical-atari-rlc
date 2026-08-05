#!/usr/bin/env python3
"""Hold a servo at a series of increasing deflections so a human can identify
which one actuates the joystick switch.

Prints and holds each level long enough to be counted from across the room.
Same safety rules as the other energising tools: low torque limit, clamped to
EEPROM angle limits, torque off on every exit path including SIGTERM.
"""
import argparse
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from feetech_protocol import FeetechBus

# Default for this robot; override with --path. Prefer a /dev/serial/by-id/
# path, which survives a replug where /dev/ttyACM0 does not.
PATH = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110727-if00"
A_TORQUE, A_ACC, A_GOAL_POS, A_GOAL_SPEED, A_TORQUE_LIMIT = 40, 41, 42, 46, 48
A_POS, A_TEMP, A_CURRENT, A_MIN_LIM, A_MAX_LIM = 56, 63, 69, 9, 11


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--servo", type=int, required=True)
    ap.add_argument("--direction", type=int, required=True, choices=(1, -1))
    ap.add_argument("--levels", default="60,120,180,240,300")
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--torque-limit", type=int, default=300)
    ap.add_argument("--current-limit", type=int, default=80)
    ap.add_argument("--neutral", type=int, default=None,
                    help="absolute neutral. REQUIRED once the servo is coupled: a "
                         "mounted servo does not spring back, so using present "
                         "position re-baselines onto the last run's drift.")
    ap.add_argument("--path", default=PATH)
    args = ap.parse_args()

    levels = [int(x) for x in args.levels.split(",")]
    bus = FeetechBus(args.path, 1000000)

    def off():
        for _ in range(4):
            if bus.write_byte(args.servo, A_TORQUE, 0):
                return
            time.sleep(0.02)
        print("!! FAILED to disable torque -- CUT POWER", file=sys.stderr)

    def panic(signum, frame):
        off(); bus.close(); os._exit(1)

    signal.signal(signal.SIGINT, panic)
    signal.signal(signal.SIGTERM, panic)

    def wword(addr, v):
        return bus.write(args.servo, addr, [v & 0xFF, (v >> 8) & 0xFF])

    try:
        if bus.read_byte(args.servo, A_TORQUE):
            print("ABORT: torque already enabled", file=sys.stderr)
            return 1
        here = bus.read_word(args.servo, A_POS)
        neutral = args.neutral if args.neutral is not None else here
        lo, hi = bus.read_word(args.servo, A_MIN_LIM), bus.read_word(args.servo, A_MAX_LIM)
        print(f"servo {args.servo} neutral {neutral} (now at {here}) "
              f"limits {lo}..{hi} dir {args.direction:+d}")
        if abs(here - neutral) > 60:
            print(f"  NOTE: {abs(here-neutral)} ticks from the given neutral; "
                  f"moving there first.")

        wword(A_TORQUE_LIMIT, args.torque_limit)
        wword(A_GOAL_SPEED, 400)
        bus.write_byte(args.servo, A_ACC, 15)
        # Energise holding the CURRENT position, then travel to neutral, so
        # enabling torque can never produce a jump.
        wword(A_GOAL_POS, here)
        bus.write_byte(args.servo, A_TORQUE, 1)
        time.sleep(0.3)
        wword(A_GOAL_POS, neutral)
        time.sleep(0.4 + abs(here - neutral) / 400.0)

        for n, d in enumerate(levels, 1):
            target = neutral + args.direction * d
            if not (lo <= target <= hi):
                print(f"  level {n}: {target} outside angle limits -- skipped")
                continue
            wword(A_GOAL_POS, target)
            time.sleep(1.2)                      # travel
            pos = bus.read_word(args.servo, A_POS)
            cur = (bus.read_word(args.servo, A_CURRENT) or 0) & 0x7FFF
            print(f"  >>> LEVEL {n}: {d:4d} ticks   goal={target} pos={pos} "
                  f"err={abs(target-pos)} cur={cur}", flush=True)
            if cur > args.current_limit:
                print(f"  stopping: current {cur} exceeds {args.current_limit}")
                break
            time.sleep(args.hold)                # hold so it can be observed
            wword(A_GOAL_POS, neutral)           # return between levels
            time.sleep(1.2)

        wword(A_GOAL_POS, neutral)
        time.sleep(0.8)
        print(f"neutral, at {bus.read_word(args.servo, A_POS)}")
        return 0
    finally:
        off()
        print("torque disabled")
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
