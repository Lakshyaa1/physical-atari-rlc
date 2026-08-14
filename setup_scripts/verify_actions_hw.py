#!/usr/bin/env python3
"""Drive the real joystick through all 18 ALE actions and check each pose.

The hardware counterpart of input_output_cpp_library/tools/verify_actions.py,
which proves the same thing against an emulated bus. That one checks the driver
commands the right numbers; this one checks the joystick actually reaches them.

Both read the same action table, so the two cannot drift apart.

Same safety rules as the other energising tools: refuses to start with torque
already enabled, clamps every target to the servo's EEPROM angle limits, and
disables torque on every exit path including SIGTERM.

    python3 setup_scripts/verify_actions_hw.py \
        --config agent_code/physical_atari_configs/physical_atari_sample_config.json
    # add --execute to actually move; without it nothing is energised
"""
import argparse
import json
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "input_output_cpp_library", "tools"))
from feetech_protocol import FeetechBus  # noqa: E402
from verify_actions import ACTIONS, FIRE_ID, LR_ID, UD_ID  # noqa: E402

A_TORQUE, A_ACC, A_GOAL_POS, A_GOAL_SPEED, A_TORQUE_LIMIT = 40, 41, 42, 46, 48
A_POS, A_TEMP, A_CURRENT, A_MIN_LIM, A_MAX_LIM = 56, 63, 69, 9, 11


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True,
                    help="robot config JSON with this robot's calibrated positions")
    ap.add_argument("--execute", action="store_true",
                    help="actually energise and move; otherwise print the plan only")
    ap.add_argument("--actions", default=None,
                    help="comma-separated subset, e.g. 1,3,4 (default: all 18)")
    ap.add_argument("--hold", type=float, default=0.6,
                    help="seconds to hold each pose before reading back")
    ap.add_argument("--tolerance", type=int, default=25,
                    help="allowed steady-state position error in ticks")
    ap.add_argument("--torque-limit", type=int, default=300)
    ap.add_argument("--current-limit", type=int, default=80)
    ap.add_argument("--path", default=None, help="defaults to the config's serial_port")
    args = ap.parse_args()

    robot = json.load(open(args.config))["robot"]
    named = {
        "lr_default": robot["dpad_lr_default"], "ud_default": robot["dpad_ud_default"],
        "left": robot["dpad_servo_left"], "right": robot["dpad_servo_right"],
        "up": robot["dpad_servo_up"], "down": robot["dpad_servo_down"],
        "released": robot["button_servo_default"], "pressed": robot["button_deflection"],
    }
    neutral = {LR_ID: named["lr_default"], UD_ID: named["ud_default"],
               FIRE_ID: named["released"]}
    ids = (LR_ID, UD_ID, FIRE_ID)

    wanted = ([int(x) for x in args.actions.split(",")] if args.actions
              else list(range(len(ACTIONS))))
    for a in wanted:
        if not 0 <= a < len(ACTIONS):
            print(f"no such action: {a}", file=sys.stderr)
            return 2

    def pose(action):
        name, lr, ud, btn = ACTIONS[action]
        return name, {LR_ID: named[lr], UD_ID: named[ud], FIRE_ID: named[btn]}

    if not args.execute:
        print("DRY RUN -- nothing will be energised. Re-run with --execute.\n")
        print(f"{'#':>2}  {'action':<15}{'L/R(51)':>10}{'U/D(52)':>10}{'fire(50)':>10}")
        for a in wanted:
            name, want = pose(a)
            print(f"{a:>2}  {name:<15}" + "".join(f"{want[i]:>10}" for i in ids))
        return 0

    bus = FeetechBus(args.path or robot["serial_port"], robot["baud_rate"])

    def off():
        for sid in ids:
            for _ in range(4):
                if bus.write_byte(sid, A_TORQUE, 0):
                    break
                time.sleep(0.02)
            else:
                print(f"!! FAILED to disable torque on {sid} -- CUT POWER", file=sys.stderr)

    def panic(signum, frame):
        off(); bus.close(); os._exit(1)

    signal.signal(signal.SIGINT, panic)
    signal.signal(signal.SIGTERM, panic)

    def wword(sid, addr, v):
        return bus.write(sid, addr, [v & 0xFF, (v >> 8) & 0xFF])

    failures = []
    try:
        limits, start = {}, {}
        for sid in ids:
            if not bus.ping(sid):
                print(f"ABORT: servo {sid} did not respond", file=sys.stderr)
                return 1
            if bus.read_byte(sid, A_TORQUE):
                print(f"ABORT: torque already enabled on servo {sid}", file=sys.stderr)
                return 1
            limits[sid] = (bus.read_word(sid, A_MIN_LIM), bus.read_word(sid, A_MAX_LIM))
            start[sid] = bus.read_word(sid, A_POS)
            print(f"servo {sid}: at {start[sid]}, neutral {neutral[sid]}, "
                  f"limits {limits[sid][0]}..{limits[sid][1]}")

        for a in wanted:
            _, want = pose(a)
            for sid in ids:
                lo, hi = limits[sid]
                if not lo <= want[sid] <= hi:
                    print(f"ABORT: action {a} wants servo {sid} at {want[sid]}, "
                          f"outside its angle limits {lo}..{hi}. The firmware would "
                          f"clamp this silently.", file=sys.stderr)
                    return 1

        # Energise holding the CURRENT position, then travel to neutral, so
        # enabling torque can never produce a jump.
        for sid in ids:
            wword(sid, A_TORQUE_LIMIT, args.torque_limit)
            wword(sid, A_GOAL_SPEED, robot["goal_speed"])
            bus.write_byte(sid, A_ACC, robot["goal_acc"])
            wword(sid, A_GOAL_POS, start[sid])
            bus.write_byte(sid, A_TORQUE, 1)
        time.sleep(0.3)
        for sid in ids:
            wword(sid, A_GOAL_POS, neutral[sid])
        time.sleep(1.0)

        print(f"\n{'#':>2}  {'action':<15}{'L/R(51)':>18}{'U/D(52)':>18}{'fire(50)':>18}")
        print("-" * 74)
        for a in wanted:
            name, want = pose(a)
            for sid in ids:
                wword(sid, A_GOAL_POS, want[sid])
            time.sleep(args.hold)

            cells, bad = [], []
            for sid in ids:
                got = bus.read_word(sid, A_POS)
                cur = (bus.read_word(sid, A_CURRENT) or 0) & 0x7FFF
                err = abs(got - want[sid]) if got is not None else None
                cells.append(f"{str(got) + '/' + str(err) + 'e':>18}")
                if err is None or err > args.tolerance:
                    bad.append(f"servo {sid}: at {got}, wanted {want[sid]} (err {err})")
                if cur > args.current_limit:
                    bad.append(f"servo {sid}: current {cur} over limit {args.current_limit}")
            print(f"{a:>2}{' ' if not bad else 'X'} {name:<15}" + "".join(cells), flush=True)
            if bad:
                failures.append(f"  action {a} ({name}): " + "; ".join(bad))

            for sid in ids:                       # back to neutral between poses
                wword(sid, A_GOAL_POS, neutral[sid])
            time.sleep(0.4)

        print("\n(cells are present_position/error)")
        if failures:
            print(f"FAIL - {len(failures)} of {len(wanted)} actions off target:")
            print("\n".join(failures))
            return 1
        print(f"OK - all {len(wanted)} actions reached their calibrated positions "
              f"within {args.tolerance} ticks.")
        return 0
    finally:
        off()
        print("torque disabled")
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
