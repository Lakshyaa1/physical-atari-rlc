#!/usr/bin/env python3
"""Report progress of a running training job: steps, rate, ETA, reward split.

Reads the log written by learn_policy.py. Rate is measured over a live window
rather than taken from the config, because the real rate on physical hardware is
set by the servo throw and the camera, not by the configured fps -- on this rig
the configured 30 fps actually runs at about 15.

    python3 setup_scripts/training_status.py                # newest log
    python3 setup_scripts/training_status.py --log PATH
    python3 setup_scripts/training_status.py --watch        # refresh until Ctrl-C
"""
import argparse
import glob
import os
import re
import time

FRAME_RE = re.compile(r"frame:\s*(\d+)")


def last_frame(path):
    """Last reported step. Read from the end -- these logs get large."""
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        size = f.tell()
        blob = b""
        for back in (8192, 65536, 524288):
            f.seek(max(0, size - back))
            blob = f.read()
            if FRAME_RE.search(blob.decode("utf-8", "replace")):
                break
    hits = FRAME_RE.findall(blob.decode("utf-8", "replace"))
    return int(hits[-1]) if hits else None


def finished(path):
    """Did the job finish, and did it save? Distinguishing 'done' from 'hung'
    matters: both show a rate of 0 steps/s, and calling a completed 12-hour run
    STALLED sends you hunting a fault that is not there."""
    tail = ""
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        f.seek(max(0, f.tell() - 65536))
        tail = f.read().decode("utf-8", "replace")
    done = "completed in" in tail
    saved = "Results saved to" in tail
    saved_to = None
    for line in tail.splitlines():
        if line.startswith("Results saved to"):
            saved_to = os.path.dirname(line.split("Results saved to", 1)[1].strip())
    return done, saved, saved_to


def counts(path):
    pos = neg = 0
    with open(path, "r", errors="replace") as f:
        for line in f:
            if "Reward = 1" in line:
                pos += 1
            elif "Reward = -1" in line:
                neg += 1
    return pos, neg


def hms(seconds):
    seconds = int(max(0, seconds))
    return f"{seconds // 3600}h {(seconds % 3600) // 60}m"


def report(path, total_steps, eval_steps, window):
    done, saved, saved_to = finished(path)
    f1, t1 = last_frame(path), time.time()
    if f1 is None:
        print("no 'frame:' lines yet -- job may still be starting up")
        return
    if done:
        f2, rate = f1, 0.0
    else:
        time.sleep(window)
        f2, t2 = last_frame(path), time.time()
        rate = (f2 - f1) / (t2 - t1) if t2 > t1 else 0.0

    pos, neg = counts(path)
    pct = 100.0 * f2 / total_steps if total_steps else 0.0
    bar = int(pct / 2.5)
    print(f"log            : {path}")
    print(f"steps          : {f2:,} / {total_steps:,}  ({pct:.2f}%)")
    print(f"                 [{'#' * bar}{'.' * (40 - bar)}]")
    if rate > 0:
        left = (total_steps - f2) / rate
        print(f"rate           : {rate:.2f} steps/s  (measured over {window:.0f}s)")
        print(f"training left  : {hms(left)}")
        print(f"incl. {eval_steps // 1000}k eval : {hms(left + eval_steps / rate)}")
        print(f"finishes about : {time.strftime('%a %H:%M', time.localtime(time.time() + left))}")
    elif done:
        print("rate           : 0 steps/s -- run has FINISHED, not stalled")
    else:
        print("rate           : 0 steps/s -- STALLED? check the log tail")
    total = pos + neg
    share = f"{100.0 * pos / total:.1f}%" if total else "n/a"
    print(f"rewards        : +{pos}  -{neg}   (positive share {share})")
    if done:
        print(f"status         : COMPLETE"
              + (f", checkpoint saved to {saved_to}" if saved else
                 " -- but NO 'Results saved' line; weights may be lost"))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", default=None, help="log path (default: newest train_*.log in ~/sahil)")
    ap.add_argument("--steps", type=int, default=648000, help="configured training steps")
    ap.add_argument("--eval-steps", type=int, default=50000)
    ap.add_argument("--window", type=float, default=20.0, help="seconds to measure the rate over")
    ap.add_argument("--watch", action="store_true", help="repeat until interrupted")
    args = ap.parse_args()

    path = args.log
    if path is None:
        pointer = os.path.expanduser("~/sahil/current_train_log")
        if os.path.exists(pointer):
            path = open(pointer).read().strip()
        else:
            found = sorted(glob.glob(os.path.expanduser("~/sahil/train_*.log")))
            path = found[-1] if found else None
    if not path or not os.path.exists(path):
        print("no training log found")
        return 1

    try:
        while True:
            report(path, args.steps, args.eval_steps, args.window)
            if not args.watch:
                return 0
            print("-" * 60, flush=True)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
