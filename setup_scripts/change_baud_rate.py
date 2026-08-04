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
change_baud_rate.py - set the baud rate of one Feetech SMS/STS servo.

The Robotroller runs its bus at 1,000,000 baud, which is also the STS3215
factory setting -- so for factory-fresh servos this script usually has nothing
to do. Use it when a servo has been set to something else, or to move a whole
bus to a different rate.

NOTE ON PORTING: the Feetech baud register is address 6 and holds a small INDEX
(0 = 1 Mbps), whereas the Dynamixel version of this script wrote address 8 with
an entirely different table (7 = 1 Mbps). Writing Dynamixel's value into a
Feetech servo would set it to 38400 and appear to brick the servo. The two are
not interchangeable.

Change one servo at a time: the moment the write lands, that servo drops off the
current baud rate and stops answering until you reconnect at the new one.

Dry run by default; pass --execute to actually write EEPROM.

Example:
    python3 setup_scripts/change_baud_rate.py --path "$SERIAL" --id 52 \\
        --current_baud_rate 115200 --new_baud_rate 1000000 --execute
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from feetech_protocol import ADDR_BAUD_RATE, BAUD_INDEX, FeetechBus


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--path", required=True,
                        help="serial device, e.g. /dev/serial/by-id/usb-...")
    parser.add_argument("--id", type=int, required=True, help="servo ID to reconfigure")
    parser.add_argument("--current_baud_rate", type=int, default=1000000,
                        help="rate the servo is on now (default 1000000, the STS3215 "
                             "factory setting)")
    parser.add_argument("--new_baud_rate", type=int, required=True,
                        help=f"rate to set; one of {sorted(BAUD_INDEX)}")
    parser.add_argument("--execute", action="store_true",
                        help="actually write EEPROM (without this, only report the plan)")
    args = parser.parse_args()

    if args.new_baud_rate not in BAUD_INDEX:
        print(f"--new_baud_rate must be one of {sorted(BAUD_INDEX)}, got {args.new_baud_rate}",
              file=sys.stderr)
        return 1

    new_index = BAUD_INDEX[args.new_baud_rate]

    bus = FeetechBus(args.path, args.current_baud_rate)
    try:
        if not bus.ping(args.id):
            found = bus.scan()
            print(f"Servo ID {args.id} did not answer at {args.current_baud_rate} baud.",
                  file=sys.stderr)
            if found:
                print(f"  These IDs did answer at this rate: {found}", file=sys.stderr)
            else:
                print("  Nothing answered at this rate. Check external power, wiring, and\n"
                      "  try another --current_baud_rate (STS3215 ships at 1000000).",
                      file=sys.stderr)
            return 1

        current_index = bus.read_byte(args.id, ADDR_BAUD_RATE)
        print(f"Servo {args.id} responds at {args.current_baud_rate} baud "
              f"(baud register = {current_index})")

        if current_index == new_index:
            print(f"Already set to {args.new_baud_rate}; nothing to do.")
            return 0

        print(f"\nPlan: set servo {args.id} baud register {current_index} -> {new_index} "
              f"({args.new_baud_rate} baud, EEPROM)")

        if not args.execute:
            print("(dry run - pass --execute to actually write)")
            return 0

        if not bus.unlock_eeprom(args.id):
            print("Failed to unlock EEPROM", file=sys.stderr)
            return 1

        # After this write the servo is on the NEW rate, so the reply to this
        # packet -- and the EEPROM re-lock -- cannot be delivered on the old
        # one. Re-open at the new rate to finish the job.
        bus.write(args.id, ADDR_BAUD_RATE, [new_index])
    finally:
        bus.close()

    print(f"Reconnecting at {args.new_baud_rate} baud to verify and re-lock ...")
    new_bus = FeetechBus(args.path, args.new_baud_rate)
    try:
        if not new_bus.ping(args.id):
            print(f"Servo {args.id} did not answer at {args.new_baud_rate} baud.\n"
                  "Power-cycle it and scan each rate to find where it landed.",
                  file=sys.stderr)
            return 1
        if not new_bus.lock_eeprom(args.id):
            print(f"WARNING: could not re-lock EEPROM on servo {args.id}", file=sys.stderr)
        print(f"Success: servo {args.id} now runs at {args.new_baud_rate} baud")
        return 0
    finally:
        new_bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
