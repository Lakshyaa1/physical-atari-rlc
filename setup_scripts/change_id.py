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
change_id.py - assign a new ID to one Feetech SMS/STS servo (e.g. STS3215).

The Robotroller expects three servos on one bus:

    ID 50  fire button
    ID 51  left / right
    ID 52  up / down

DO THIS ONE SERVO AT A TIME, with only that servo connected.

Feetech servos all ship as **ID 1**. If two of them are on the bus at once they
both answer to ID 1, and a single write renames both -- leaving you with two
servos sharing an ID and no way to address either individually. This script
refuses to write when it sees more than one servo unless you name the source ID
explicitly with --current_id, and even then it warns, because duplicate IDs are
indistinguishable from a single servo on the wire.

Assign in descending order (52, then 51, then 50) so that a servo still sitting
at the factory ID 1 is never the target of a later write.

Dry run by default; pass --execute to actually write EEPROM.

Example:
    python3 setup_scripts/change_id.py --path "$SERIAL" --new_id 52
    python3 setup_scripts/change_id.py --path "$SERIAL" --new_id 52 --execute
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from feetech_protocol import ADDR_ID, FeetechBus


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--path", required=True,
                        help="serial device, e.g. /dev/serial/by-id/usb-...")
    parser.add_argument("--new_id", type=int, required=True,
                        help="ID to assign (0-252); Robotroller uses 50, 51, 52")
    parser.add_argument("--current_id", type=int, default=None,
                        help="ID of the servo to rename. Required when more than one "
                             "servo answers on the bus.")
    parser.add_argument("--baud_rate", type=int, default=1000000,
                        help="bus baud rate (default 1000000 - the STS3215 factory "
                             "setting; note this differs from Dynamixel's 57600)")
    parser.add_argument("--execute", action="store_true",
                        help="actually write EEPROM (without this, only report the plan)")
    args = parser.parse_args()

    if not 0 <= args.new_id <= 252:
        print(f"--new_id must be 0-252, got {args.new_id}", file=sys.stderr)
        return 1

    bus = FeetechBus(args.path, args.baud_rate)
    try:
        print(f"Scanning {args.path} at {args.baud_rate} baud ...")
        found = bus.scan()

        if not found:
            print("No servos answered.\n"
                  "  - is the external power supply on? (USB alone does not power the servos)\n"
                  "  - is the baud rate right? try --baud_rate 57600 or 115200\n"
                  "  - check the data wiring", file=sys.stderr)
            return 1

        print(f"Servos found: {found}")

        if args.current_id is None:
            if len(found) > 1:
                print(f"\nRefusing to continue: {len(found)} servos are on the bus {found}.\n"
                      "Renaming with more than one connected risks writing to the wrong servo.\n"
                      "Either connect just one servo, or name the source explicitly with\n"
                      "--current_id <id>.", file=sys.stderr)
                return 1
            current_id = found[0]
        else:
            current_id = args.current_id
            if current_id not in found:
                print(f"\nServo ID {current_id} did not answer. Present: {found}",
                      file=sys.stderr)
                return 1
            if len(found) > 1:
                print(f"\nWARNING: {len(found)} servos are on the bus. If any of them share\n"
                      f"ID {current_id}, they will ALL be renamed to {args.new_id} and become\n"
                      "impossible to address separately. Connecting one servo at a time is\n"
                      "strongly preferred.")

        if current_id == args.new_id:
            print(f"\nServo is already ID {args.new_id}; nothing to do.")
            return 0

        if args.new_id in found:
            print(f"\nRefusing to continue: ID {args.new_id} is already taken by another servo\n"
                  "on this bus. Pick a free ID, or renumber that servo first.", file=sys.stderr)
            return 1

        print(f"\nPlan: change servo ID {current_id} -> {args.new_id} (EEPROM)")

        if not args.execute:
            print("(dry run - pass --execute to actually write)")
            return 0

        if not bus.unlock_eeprom(current_id):
            print("Failed to unlock EEPROM", file=sys.stderr)
            return 1

        wrote = bus.write_byte(current_id, ADDR_ID, args.new_id)

        # The servo answers on its NEW id from this point on, so re-lock there.
        locked = bus.lock_eeprom(args.new_id)

        if not wrote:
            print("ID write failed; attempting to re-lock the original ID", file=sys.stderr)
            bus.lock_eeprom(current_id)
            return 1
        if not locked:
            print(f"WARNING: could not re-lock EEPROM on ID {args.new_id}", file=sys.stderr)

        if bus.ping(args.new_id):
            print(f"Success: servo now answers to ID {args.new_id}")
            return 0

        print(f"Wrote the new ID but ID {args.new_id} does not answer. Power-cycle the servo\n"
              "and re-scan to check.", file=sys.stderr)
        return 1
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
