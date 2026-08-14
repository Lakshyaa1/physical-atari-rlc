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
monitor_joystick.py - watch the ESP32 bridge live, on the terminal.

Prints one line per state CHANGE, plus a heartbeat with the frame rate so you
can tell "nothing is pressed" apart from "nothing is arriving". Use it to answer
the question the devbox cannot: is the robot actually closing the switches?

    python3 tools/monitor_joystick.py

The devbox must NOT be running: only one reader can drain the port, and two will
steal bytes from each other.

DIAGNOSING "the bridge streams but no switch ever closes"
--------------------------------------------------------
If the heartbeat shows a healthy frame rate and every frame is 0x80, the MCU and
the USB link are fine and the fault is in the switch wiring. All five switches
share ONE common return (DB9 pin 8 -> ESP32 GND), so a single loose common makes
all five fail at once while the bridge keeps streaming perfectly.

To separate the CX40 from the ESP32, touch a jumper from ESP32 GND to one signal
pin and watch this monitor:

    GPIO 14 -> up      GPIO 26 -> right
    GPIO 13 -> down    GPIO 27 -> fire
    GPIO 25 -> left

If the jumper registers but the joystick does not, the ESP32 and its firmware
are good and the problem is the CX40 cable, its connector, or the common return.
"""

import argparse
import glob
import sys
import time

import serial

BITS = [("up", 0), ("down", 1), ("left", 2), ("right", 3), ("fire", 4)]
DEFAULT_PORT = ("/dev/serial/by-id/"
                "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")


def describe(frame):
    names = [n for n, b in BITS if frame & (1 << b)]
    return ", ".join(names) if names else "(all released)"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None,
                    help="serial device (default: the CP2102 by-id path, then any ttyUSB*)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--heartbeat", type=float, default=2.0,
                    help="seconds between rate reports (0 to disable)")
    args = ap.parse_args()

    port = args.port
    if port is None:
        candidates = [DEFAULT_PORT] + sorted(glob.glob("/dev/ttyUSB*"))
        port = next((p for p in candidates if glob.glob(p)), None)
        if port is None:
            print("no serial device found -- is the ESP32 plugged in?", file=sys.stderr)
            return 1

    # Always prefer a by-id path. A USB bridge that re-enumerates moves from
    # ttyUSB0 to ttyUSB1, and anything holding the old name reads nothing
    # forever with no error -- which looks exactly like a robot that stopped
    # pressing.
    print(f"monitoring {port} @ {args.baud}")
    print("waiting for frames (bit 7 set)... Ctrl-C to stop\n")

    s = serial.Serial(port, args.baud, timeout=0.1)
    s.reset_input_buffer()

    last = None
    frames = 0
    window = 0
    t0 = time.time()
    try:
        while True:
            chunk = s.read(4096)
            for byte in chunk:
                if not (byte & 0x80):
                    continue  # boot log or noise; frames always have bit 7 set
                frames += 1
                window += 1
                if byte != last:
                    stamp = time.strftime("%H:%M:%S")
                    bits = " ".join(f"{n}={1 if byte & (1 << b) else 0}" for n, b in BITS)
                    print(f"[{stamp}] 0x{byte:02X}  {bits}   <- {describe(byte)}", flush=True)
                    last = byte
            now = time.time()
            if args.heartbeat and now - t0 >= args.heartbeat:
                rate = window / (now - t0)
                state = "idle (0x80)" if last == 0x80 else f"0x{last:02X}" if last else "no data"
                print(f"    ... {rate:6.0f} frames/s, holding {state}", flush=True)
                window, t0 = 0, now
    except KeyboardInterrupt:
        print(f"\nstopped after {frames} frames")
    finally:
        s.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
