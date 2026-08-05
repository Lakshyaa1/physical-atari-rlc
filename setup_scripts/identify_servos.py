#!/usr/bin/env python3
"""Identify which servo ID is which joint, with torque OFF, by hand.

Read-only: it never writes a register, never enables torque. It just polls
present position and reports how far each servo moved during the window.
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="4,5,7")
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--path", default=PATH)
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",")]
    bus = FeetechBus(args.path, 1000000)
    try:
        # Confirm torque really is off before asking anyone to touch the rig.
        for i in ids:
            if bus.read_byte(i, 40):
                print(f"ABORT: torque is ENABLED on servo {i}", file=sys.stderr)
                return 1

        start = {i: bus.read_word(i, 56) for i in ids}
        lo = dict(start)
        hi = dict(start)
        print(f"start: {start}")
        print(f"Move ONE servo by hand for {args.seconds:.0f}s ...")

        deadline = time.monotonic() + args.seconds
        next_report = time.monotonic()
        while time.monotonic() < deadline:
            now = {}
            for i in ids:
                p = bus.read_word(i, 56)
                if p is None:
                    continue
                now[i] = p
                lo[i] = min(lo[i], p)
                hi[i] = max(hi[i], p)
            if time.monotonic() >= next_report:
                next_report += 1.0
                left = deadline - time.monotonic()
                line = "  ".join(
                    f"{i}: pos={now.get(i, 0):>4} moved={hi[i]-lo[i]:>4}" for i in ids)
                print(f"[{left:4.0f}s left]  {line}", flush=True)
            time.sleep(0.03)

        print()
        ranked = sorted(ids, key=lambda i: hi[i] - lo[i], reverse=True)
        for i in ranked:
            span = hi[i] - lo[i]
            print(f"  ID {i}: moved {span:>5} ticks  ({lo[i]}..{hi[i]})")
        best = ranked[0]
        if hi[best] - lo[best] < 40:
            print("\nNothing moved appreciably -- was the right servo moved?")
        else:
            print(f"\n=> the servo you moved is ID {best}")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
