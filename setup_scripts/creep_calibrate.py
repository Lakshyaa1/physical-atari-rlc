#!/usr/bin/env python3
"""Find one joystick deflection limit by creeping outward from neutral.

For a rig whose servos are mounted and cannot be back-driven by hand. Enables
torque at a deliberately low torque_limit and walks the goal position outward in
small steps, stopping at the FIRST sign of mechanical resistance.

Safety invariants (this is the only tool here that energises a servo):
  * The first goal written is the servo's CURRENT position, so enabling torque
    never produces a jump.
  * One servo, one direction, per invocation.
  * Every step checks present current, position error, and temperature.
  * A hard cap on total excursion, independent of any sensor.
  * Targets are clamped to the servo's own EEPROM angle limits, which the
    firmware would otherwise silently apply behind our back.
  * Torque is disabled on EVERY exit path: normal finish, stop condition,
    exception, SIGINT, and SIGTERM (so `timeout` cannot leave it energised).

Stop conditions are deliberately trigger-happy. Overshooting a joystick's hard
stop stalls the servo against it, which is exactly the sustained-load case the
shared power rail is known to be weak at.
"""
import argparse
import json
import os
import signal
import statistics as st
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from feetech_protocol import FeetechBus

# Default for this robot; override with --path. Prefer a /dev/serial/by-id/
# path, which survives a replug where /dev/ttyACM0 does not.
PATH = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110727-if00"
STORE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json")

A_TORQUE, A_ACC, A_GOAL_POS, A_GOAL_SPEED, A_TORQUE_LIMIT = 40, 41, 42, 46, 48
A_POS, A_LOAD, A_VOLT, A_TEMP, A_CURRENT = 56, 60, 62, 63, 69
A_MIN_LIM, A_MAX_LIM = 9, 11
MA_PER_COUNT = 6.5


class Calibrator:
    def __init__(self, bus, servo):
        self.bus, self.servo = bus, servo
        self.energised = False

    def word(self, addr):
        return self.bus.read_word(self.servo, addr)

    def write_word(self, addr, value):
        return self.bus.write(self.servo, addr, [value & 0xFF, (value >> 8) & 0xFF])

    def torque_off(self):
        # Belt and braces: retry, because a single lost packet here means a
        # servo left holding against a hard stop.
        for _ in range(4):
            if self.bus.write_byte(self.servo, A_TORQUE, 0):
                self.energised = False
                return True
            time.sleep(0.02)
        print(f"!! FAILED to disable torque on servo {self.servo} -- CUT POWER", file=sys.stderr)
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--servo", type=int, required=True)
    ap.add_argument("--direction", type=int, required=True, choices=(1, -1))
    ap.add_argument("--label", required=True)
    ap.add_argument("--step", type=int, default=8, help="ticks per step")
    ap.add_argument("--settle", type=float, default=0.30, help="seconds per step")
    ap.add_argument("--max-ticks", type=int, default=350, help="hard excursion cap")
    ap.add_argument("--torque-limit", type=int, default=300, help="0-1000")
    ap.add_argument("--goal-speed", type=int, default=200)
    ap.add_argument("--acc", type=int, default=10)
    ap.add_argument("--max-error", type=int, default=20, help="stall: goal-present")
    ap.add_argument("--current-limit", type=int, default=120, help="raw counts")
    ap.add_argument("--temp-limit", type=int, default=50)
    ap.add_argument("--path", default=PATH)
    ap.add_argument("--execute", action="store_true",
                    help="actually energise the servo (without this, report the plan only)")
    args = ap.parse_args()

    bus = FeetechBus(args.path, 1000000)
    cal = Calibrator(bus, args.servo)

    def panic(signum, frame):
        print(f"\n[signal {signum}] disabling torque", flush=True)
        cal.torque_off()
        bus.close()
        os._exit(1)

    signal.signal(signal.SIGINT, panic)
    signal.signal(signal.SIGTERM, panic)

    try:
        if bus.read_byte(args.servo, A_TORQUE):
            print(f"ABORT: torque already ENABLED on {args.servo}", file=sys.stderr)
            return 1

        neutral = cal.word(A_POS)
        lo_lim, hi_lim = cal.word(A_MIN_LIM), cal.word(A_MAX_LIM)
        temp, volt = bus.read_byte(args.servo, A_TEMP), bus.read_byte(args.servo, A_VOLT) / 10.0
        limit_stop = hi_lim if args.direction > 0 else lo_lim
        cap = neutral + args.direction * args.max_ticks
        cap = min(cap, hi_lim) if args.direction > 0 else max(cap, lo_lim)

        print(f"servo {args.servo}  '{args.label}'  direction {args.direction:+d}")
        print(f"  neutral {neutral}   angle limits {lo_lim}..{hi_lim}   {volt} V   {temp} C")
        print(f"  will creep to at most {cap} ({abs(cap-neutral)} ticks), "
              f"step {args.step}, torque_limit {args.torque_limit}")
        print(f"  stop on: current>{args.current_limit} counts "
              f"(~{args.current_limit*MA_PER_COUNT:.0f} mA), error>{args.max_error}, temp>{args.temp_limit}")
        if not args.execute:
            print("\n(dry run -- pass --execute to energise)")
            return 0
        if temp > args.temp_limit:
            print("ABORT: already too hot", file=sys.stderr)
            return 1

        # Profile + limits BEFORE torque, then hold current position.
        cal.write_word(A_TORQUE_LIMIT, args.torque_limit)
        cal.write_word(A_GOAL_SPEED, args.goal_speed)
        bus.write_byte(args.servo, A_ACC, args.acc)
        cal.write_word(A_GOAL_POS, neutral)
        bus.write_byte(args.servo, A_TORQUE, 1)
        cal.energised = True
        time.sleep(0.4)

        currents, last_good, reason = [], neutral, "reached excursion cap"
        goal = neutral
        while (goal - cap) * args.direction < 0:
            goal += args.direction * args.step
            if (goal - cap) * args.direction > 0:
                goal = cap
            cal.write_word(A_GOAL_POS, goal)
            time.sleep(args.settle)

            pos = cal.word(A_POS)
            cur = cal.word(A_CURRENT)
            t = bus.read_byte(args.servo, A_TEMP)
            if pos is None or cur is None or t is None:
                reason = "telemetry read failed"
                break
            cur &= 0x7FFF  # sign bit is direction, not magnitude
            err = abs(goal - pos)
            currents.append(cur)
            print(f"  goal={goal:5d} pos={pos:5d} err={err:3d} "
                  f"cur={cur:4d} ({cur*MA_PER_COUNT:6.0f} mA) {t}C", flush=True)

            if t > args.temp_limit:
                reason = f"temperature {t} C"
                break
            if cur > args.current_limit:
                reason = f"current {cur} counts (~{cur*MA_PER_COUNT:.0f} mA)"
                break
            if err > args.max_error:
                reason = f"position error {err} ticks (stalled)"
                break
            last_good = pos

        print(f"\n  STOPPED: {reason}")
        print(f"  last position tracking cleanly: {last_good} "
              f"({last_good-neutral:+d} from neutral {neutral})")
        if currents:
            print(f"  current: median {st.median(currents):.0f}, max {max(currents)} counts")
        if abs(last_good - limit_stop) < 5:
            print("  NOTE: stopped at the EEPROM angle limit, not a mechanical stop.")

        # Walk back to neutral rather than snapping to it.
        for g in range(last_good, neutral, -args.direction * 16) if last_good != neutral else []:
            cal.write_word(A_GOAL_POS, g)
            time.sleep(0.05)
        cal.write_word(A_GOAL_POS, neutral)
        time.sleep(0.6)
        print(f"  returned to neutral, now at {cal.word(A_POS)}")

        store = json.load(open(STORE)) if os.path.exists(STORE) else {}
        store[args.label] = {
            "servo": args.servo, "neutral": neutral, "limit_reached": last_good,
            "deflection": last_good - neutral, "stop_reason": reason,
            "max_current_counts": max(currents) if currents else None,
        }
        json.dump(store, open(STORE, "w"), indent=2)
        print(f"  saved -> {STORE}")
        return 0
    finally:
        cal.torque_off()
        print(f"  torque disabled (servo {args.servo})")
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
