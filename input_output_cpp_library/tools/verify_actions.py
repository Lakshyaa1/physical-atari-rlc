#!/usr/bin/env python3
# Copyright 2026 Keen Technologies, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
verify_actions.py - check that all 18 ALE actions drive the servos correctly.

Runs the real compiled Robotroller against fake_feetech_bus.py, calls setAction
for every action in the 18-action set, and reads back the goal positions that
actually reached the wire. Each is compared against what the action name means
in terms of this robot's calibrated positions.

This catches the class of bug that is otherwise invisible until the joystick is
physically watched: an action table row wired to the wrong servo, a swapped
left/right sign, or a dpad axis reusing the other axis's neutral.

    python3 tools/verify_actions.py \
        --config ../agent_code/physical_atari_configs/physical_atari_sample_config.json

No hardware, no power supply, and no Waveshare adapter are needed.
"""

import argparse
import gc
import json
import os
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_feetech_bus import load_log  # noqa: E402

FIRE_ID, LR_ID, UD_ID = 50, 51, 52

# Which conceptual position each action asks of each axis. Resolved against the
# config below, so this stays readable rather than a wall of encoder counts.
ACTIONS = [
    ("NOOP",           "lr_default",  "ud_default", "released"),
    ("FIRE",           "lr_default",  "ud_default", "pressed"),
    ("UP",             "lr_default",  "up",         "released"),
    ("RIGHT",          "right",       "ud_default", "released"),
    ("LEFT",           "left",        "ud_default", "released"),
    ("DOWN",           "lr_default",  "down",       "released"),
    ("UPRIGHT",        "right",       "up",         "released"),
    ("UPLEFT",         "left",        "up",         "released"),
    ("DOWNRIGHT",      "right",       "down",       "released"),
    ("DOWNLEFT",       "left",        "down",       "released"),
    ("UPFIRE",         "lr_default",  "up",         "pressed"),
    ("RIGHTFIRE",      "right",       "ud_default", "pressed"),
    ("LEFTFIRE",       "left",        "ud_default", "pressed"),
    ("DOWNFIRE",       "lr_default",  "down",       "pressed"),
    ("UPRIGHTFIRE",    "right",       "up",         "pressed"),
    ("UPLEFTFIRE",     "left",        "up",         "pressed"),
    ("DOWNRIGHTFIRE",  "right",       "down",       "pressed"),
    ("DOWNLEFTFIRE",   "left",        "down",       "pressed"),
]


def wait_for_pty(path_file, proc, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if proc.poll() is not None:
            raise SystemExit("fake bus exited before it published a pty path")
        if os.path.exists(path_file):
            path = open(path_file).read().strip()
            if path:
                return path
        time.sleep(0.05)
    raise SystemExit("timed out waiting for the fake bus pty")


# SyncWritePosEx starts its burst at the ACCELERATION register, not at the goal
# position, so one sync write covers addr 41..47:
#   [acc, pos_L, pos_H, time_L, time_H, speed_L, speed_H]
# Looking for writes to addr 42 finds nothing at all.
ADDR_ACC, ADDR_GOAL_POS = 41, 42
POS_OFFSET = {ADDR_ACC: 1, ADDR_GOAL_POS: 0}


def position_commands(events):
    """Goal positions from each sync write that carries a position.

    Returns one {servo_id: position} dict per sync write, in order. The driver
    commands all three servos in a single sync write precisely so the axes do
    not start moving at different times, so one entry here is one action.
    """
    out = []
    for e in events:
        if e["event"] != "sync_write" or e["addr"] not in POS_OFFSET:
            continue
        off = POS_OFFSET[e["addr"]]
        out.append({t["id"]: t["data"][off] | (t["data"][off + 1] << 8)
                    for t in e["targets"] if len(t["data"]) >= off + 2})
    return out


def wait_for_command(log_path, already_seen, deadline):
    """Block until a new position sync write appears; return it, or None.

    setAction() only hands the action to a worker thread, so the write reaches
    the wire some time after the call returns.
    """
    while True:
        cmds = position_commands(load_log(log_path))
        if len(cmds) > already_seen:
            return cmds[already_seen], len(cmds)
        if time.time() > deadline:
            return None, len(cmds)
        time.sleep(0.005)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True,
                    help="robot config JSON holding this robot's calibrated positions")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="seconds to wait for each setAction to reach the wire")
    args = ap.parse_args()

    robot = json.load(open(args.config))["robot"]
    named = {
        "lr_default": robot["dpad_lr_default"],
        "ud_default": robot["dpad_ud_default"],
        "left":       robot["dpad_servo_left"],
        "right":      robot["dpad_servo_right"],
        "up":         robot["dpad_servo_up"],
        "down":       robot["dpad_servo_down"],
        "released":   robot["button_servo_default"],
        "pressed":    robot["button_deflection"],
    }

    tmp = tempfile.mkdtemp(prefix="verify_actions.")
    log_path, path_file = os.path.join(tmp, "bus.json"), os.path.join(tmp, "pty")
    bus = subprocess.Popen(
        [sys.executable, os.path.join(here, "fake_feetech_bus.py"),
         "--log", log_path, "--path-file", path_file, "--seconds", "300"],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    failures = []
    rc = None
    try:
        device = wait_for_pty(path_file, bus)
        import robotroller

        rc = robotroller.Robotroller(
            device_path=device,
            baud_rate=robot["baud_rate"],
            position_p_gain=robot["P_gain"],
            position_i_gain=robot["I_gain"],
            position_d_gain=robot["D_gain"],
            dpad_lr_default=robot["dpad_lr_default"],
            dpad_ud_default=robot["dpad_ud_default"],
            dpad_servo_right=robot["dpad_servo_right"],
            dpad_servo_left=robot["dpad_servo_left"],
            dpad_servo_up=robot["dpad_servo_up"],
            dpad_servo_down=robot["dpad_servo_down"],
            button_servo_default=robot["button_servo_default"],
            button_deflection=robot["button_deflection"],
            goal_speed=robot["goal_speed"],
            goal_acc=robot["goal_acc"],
            torque_limit=robot["torque_limit"],
            overcurrent_counts=robot["overcurrent_counts"],
        )

        print(f"{'#':>2}  {'action':<15} {'L/R(51)':>16} {'U/D(52)':>16} {'fire(50)':>16}")
        print("-" * 72)

        # Action 0 goes last. The driver starts up believing the servos are
        # already at NOOP, so commanding 0 first is correctly a no-op and would
        # look like a missing write.
        order = list(range(1, len(ACTIONS))) + [0]
        seen = 0
        for action in order:
            name, lr, ud, btn = ACTIONS[action]
            want = {LR_ID: named[lr], UD_ID: named[ud], FIRE_ID: named[btn]}
            rc.setAction(action)
            got, seen = wait_for_command(log_path, seen, time.time() + args.settle)

            if got is None:
                print(f"{action:>2}X {name:<15}{'(no command sent)':>50}")
                failures.append(f"  action {action} ({name}): no position write reached the bus")
                continue

            cells, bad = [], []
            for sid in (LR_ID, UD_ID, FIRE_ID):
                g, w = got.get(sid), want[sid]
                if g == w:
                    cells.append(f"{g:>16}")
                else:
                    cells.append(f"{str(g) + '!=' + str(w):>16}")
                    bad.append(f"servo {sid}: got {g}, want {w}")
            mark = " " if not bad else "X"
            print(f"{action:>2}{mark} {name:<15}" + "".join(cells))
            if bad:
                failures.append(f"  action {action} ({name}): " + "; ".join(bad))
    finally:
        # Shut the Robotroller down FIRST. Its destructor joins the reflex
        # thread, which is mid-telemetry-read on the bus; killing the bus first
        # leaves that read blocked forever and the join never returns.
        rc = None
        gc.collect()
        # SIGKILL, not SIGTERM: the bus blocks reading its pty master and can
        # sit on a handled signal long enough to hang the caller's session.
        bus.kill()
        bus.wait(timeout=5)

    print()
    if failures:
        print(f"FAIL - {len(failures)} of {len(ACTIONS)} actions wrong:")
        print("\n".join(failures))
        return 1
    print(f"OK - all {len(ACTIONS)} actions map to the calibrated positions.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
