#!/usr/bin/env python3
"""Measure how long a joystick throw actually takes, at several goal speeds.

The throw is a large, fixed share of the end-to-end latency budget (the paper's
is ~165 ms), and it is the one part measurable with nothing but the servo bus --
no camera, no ESP32. Use it to decide whether the throw needs a mechanical fix
(a longer horn arm) or just a faster goal_speed.

Reports time to first motion, time to enter a tolerance band, and the settled
error, averaged over repeats.

Same safety rules as the other energising tools: refuses to start with torque
already enabled, clamps to EEPROM angle limits, torque off on every exit path.

    python3 setup_scripts/measure_throw_time.py \
        --config agent_code/physical_atari_configs/physical_atari_sample_config.json
    # add --execute to actually move
"""
import argparse
import json
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from feetech_protocol import FeetechBus  # noqa: E402

A_TORQUE, A_ACC, A_GOAL_POS, A_GOAL_SPEED, A_TORQUE_LIMIT = 40, 41, 42, 46, 48
A_POS, A_CURRENT, A_MIN_LIM, A_MAX_LIM = 56, 69, 9, 11


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--speeds", default="1200,2400,3400",
                    help="goal_speed values to compare (steps/s)")
    ap.add_argument("--repeats", type=int, default=4)
    ap.add_argument("--tolerance", type=int, default=20,
                    help="ticks from target that count as 'arrived'")
    ap.add_argument("--timeout", type=float, default=1.5, help="per-move give-up, seconds")
    ap.add_argument("--torque-limit", type=int, default=300)
    ap.add_argument("--path", default=None)
    args = ap.parse_args()

    robot = json.load(open(args.config))["robot"]
    speeds = [int(s) for s in args.speeds.split(",")]

    # Each move is (servo, from, to, label) -- the deflections the agent uses.
    moves = [
        (51, robot["dpad_lr_default"], robot["dpad_servo_left"],  "L/R  neutral->left"),
        (51, robot["dpad_lr_default"], robot["dpad_servo_right"], "L/R  neutral->right"),
        (52, robot["dpad_ud_default"], robot["dpad_servo_up"],    "U/D  neutral->up"),
        (52, robot["dpad_ud_default"], robot["dpad_servo_down"],  "U/D  neutral->down"),
        (50, robot["button_servo_default"], robot["button_deflection"], "fire press"),
    ]

    if not args.execute:
        print("DRY RUN -- nothing will be energised. Re-run with --execute.\n")
        for sid, a, b, label in moves:
            print(f"  servo {sid}  {label:<22} {a} -> {b}  ({abs(b - a)} ticks)")
        print(f"\nspeeds: {speeds}, {args.repeats} repeats each")
        return 0

    bus = FeetechBus(args.path or robot["serial_port"], robot["baud_rate"])
    ids = sorted({m[0] for m in moves})

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

    try:
        for sid in ids:
            if not bus.ping(sid):
                print(f"ABORT: servo {sid} did not respond", file=sys.stderr)
                return 1
            if bus.read_byte(sid, A_TORQUE):
                print(f"ABORT: torque already enabled on servo {sid}", file=sys.stderr)
                return 1
            lo, hi = bus.read_word(sid, A_MIN_LIM), bus.read_word(sid, A_MAX_LIM)
            for m in moves:
                if m[0] == sid and not (lo <= m[2] <= hi and lo <= m[1] <= hi):
                    print(f"ABORT: {m[3]} leaves servo {sid} angle limits {lo}..{hi}",
                          file=sys.stderr)
                    return 1

        for sid in ids:
            wword(sid, A_TORQUE_LIMIT, args.torque_limit)
            bus.write_byte(sid, A_ACC, robot["goal_acc"])
            wword(sid, A_GOAL_POS, bus.read_word(sid, A_POS))
            bus.write_byte(sid, A_TORQUE, 1)
        time.sleep(0.3)

        print(f"acc={robot['goal_acc']}, torque_limit={args.torque_limit}, "
              f"tolerance={args.tolerance} ticks, {args.repeats} repeats\n")
        print(f"{'speed':>6} {'move':<24}{'ticks':>7}{'t_move':>9}{'t_arrive':>10}"
              f"{'settled_err':>13}")
        print("-" * 70)

        for speed in speeds:
            for sid in ids:
                wword(sid, A_GOAL_SPEED, speed)
            for sid, home, target, label in moves:
                ticks = abs(target - home)
                first, arrive, errs = [], [], []
                for _ in range(args.repeats):
                    wword(sid, A_GOAL_POS, home)
                    time.sleep(0.6)
                    base = bus.read_word(sid, A_POS)

                    t0 = time.perf_counter()
                    wword(sid, A_GOAL_POS, target)
                    t_first = t_arrive = None
                    while True:
                        now = time.perf_counter() - t0
                        pos = bus.read_word(sid, A_POS)
                        if pos is not None:
                            if t_first is None and abs(pos - base) > 5:
                                t_first = now
                            if t_arrive is None and abs(pos - target) <= args.tolerance:
                                t_arrive = now
                                break
                        if now > args.timeout:
                            break
                    time.sleep(0.35)
                    settled = bus.read_word(sid, A_POS)
                    if t_first is not None:
                        first.append(t_first)
                    if t_arrive is not None:
                        arrive.append(t_arrive)
                    if settled is not None:
                        errs.append(abs(settled - target))

                def ms(xs):
                    return f"{1000 * sum(xs) / len(xs):.0f} ms" if xs else "  --  "
                err = f"{sum(errs) / len(errs):.1f}" if errs else "--"
                miss = "" if len(arrive) == args.repeats else f"  ({len(arrive)}/{args.repeats})"
                print(f"{speed:>6} {label:<24}{ticks:>7}{ms(first):>9}{ms(arrive):>10}"
                      f"{err:>13}{miss}", flush=True)
                wword(sid, A_GOAL_POS, home)
                time.sleep(0.4)
            print()

        print("t_move is the delay before the servo starts moving (bus + controller);")
        print("t_arrive is from command to entering the tolerance band.")
        return 0
    finally:
        off()
        print("torque disabled")
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
