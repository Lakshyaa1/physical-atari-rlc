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
fake_joystick_bridge.py - pretend to be the ESP32 joystick bridge.

Creates a pseudo-terminal, prints its device path, and streams the same wire
protocol the real ESP32 firmware sends (see esp32_joystick_bridge.ino). Lets you
exercise the devbox's --input=serial path with no MCU, no CX40, and no robot:

    python3 tools/fake_joystick_bridge.py --hold down &
    # note the printed /dev/pts/N
    PhysicalALE ./games/ pong --input=serial:/dev/pts/N

Useful for confirming the devbox reacts to input at all before you go looking
for faults in wiring or firmware.
"""

import argparse
import os
import pty
import sys
import time

BUTTON_BITS = {"up": 0, "down": 1, "left": 2, "right": 3, "fire": 4}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hold", default="",
                        help="comma-separated buttons held down for the whole run "
                             "(up,down,left,right,fire). Default: nothing pressed.")
    parser.add_argument("--hz", type=float, default=1000.0,
                        help="sample rate, matching the firmware (default 1000)")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stop after this long (default: run until killed)")
    parser.add_argument("--path-file", default=None,
                        help="write the pty device path here (easier than scraping stdout)")
    args = parser.parse_args()

    state = 0
    for name in [b.strip() for b in args.hold.split(",") if b.strip()]:
        if name not in BUTTON_BITS:
            print(f"unknown button {name!r}; expected one of {sorted(BUTTON_BITS)}", file=sys.stderr)
            return 1
        state |= 1 << BUTTON_BITS[name]

    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)

    if args.path_file:
        with open(args.path_file, "w") as f:
            f.write(slave_path)
    print(f"[fake-bridge] serving on {slave_path}", flush=True)
    print(f"[fake-bridge] holding: {args.hold or '(nothing)'} -> frame byte 0x{0x80 | state:02X}",
          flush=True)

    frame = bytes([0x80 | state])
    interval = 1.0 / args.hz
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None

    try:
        while True:
            try:
                os.write(master_fd, frame)
            except OSError:
                break  # reader closed
            if deadline is not None and time.monotonic() >= deadline:
                break
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(master_fd)
        os.close(slave_fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
