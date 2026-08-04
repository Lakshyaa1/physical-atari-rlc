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
fake_feetech_bus.py - emulate three Feetech STS3215 servos on a pseudo-terminal.

Lets the real C++ Robotroller be exercised with no servos, no power supply, and
no Waveshare adapter. It speaks the actual SMS/STS wire protocol, so it
exercises the driver's real framing, register addresses, and byte order rather
than a mock of them.

Every register access is logged to a JSON file so a test can assert on what the
driver actually did -- which servo IDs it addressed, whether it used a sync
write, which EEPROM cells it touched.

    python3 tools/fake_feetech_bus.py --log /tmp/bus.json &
    # note the printed /dev/pts/N, then point the driver at it

--inject-overcurrent makes one servo report a large present current so the
high-current reflex can be observed firing.
"""

import argparse
import json
import os
import pty
import signal
import sys
import time

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE

# Register addresses (scservo/SMS_STS.h)
ADDR_MODEL_L, ADDR_ID, ADDR_BAUD = 3, 5, 6
ADDR_P_COEF, ADDR_D_COEF, ADDR_I_COEF = 21, 22, 23
ADDR_MODE, ADDR_TORQUE_ENABLE, ADDR_ACC = 33, 40, 41
ADDR_GOAL_POS_L, ADDR_GOAL_SPEED_L = 42, 46
ADDR_TORQUE_LIMIT_L, ADDR_LOCK = 48, 55
ADDR_PRESENT_POS_L, ADDR_PRESENT_VOLTAGE = 56, 62
ADDR_PRESENT_TEMP, ADDR_PRESENT_CURRENT_L = 63, 69

DEFAULT_IDS = (50, 51, 52)


class FakeServo:
    """One STS3215's register file, with just enough behaviour to be realistic."""

    def __init__(self, servo_id, overcurrent=False):
        self.id = servo_id
        self.mem = bytearray(256)
        self.mem[ADDR_MODEL_L] = 777 & 0xFF        # STS3215 model number, little-endian
        self.mem[ADDR_MODEL_L + 1] = (777 >> 8) & 0xFF
        self.mem[ADDR_ID] = servo_id
        self.mem[ADDR_BAUD] = 0                     # index 0 == 1 Mbps (factory default)
        self.mem[ADDR_P_COEF] = 32                  # factory PID coefficients
        self.mem[ADDR_D_COEF] = 0
        self.mem[ADDR_I_COEF] = 0
        self.mem[ADDR_MODE] = 0                     # servo/position mode
        self.mem[ADDR_LOCK] = 1                     # EEPROM locked
        self._set_word(ADDR_PRESENT_POS_L, 2048)    # centred
        self.mem[ADDR_PRESENT_VOLTAGE] = 74         # 7.4 V, in 0.1 V units
        self.mem[ADDR_PRESENT_TEMP] = 35
        self._set_word(ADDR_PRESENT_CURRENT_L, 900 if overcurrent else 20)

    def _set_word(self, addr, value):
        self.mem[addr] = value & 0xFF               # End=0 -> little-endian
        self.mem[addr + 1] = (value >> 8) & 0xFF

    def get_word(self, addr):
        return self.mem[addr] | (self.mem[addr + 1] << 8)

    def write(self, addr, data):
        for i, byte in enumerate(data):
            self.mem[addr + i] = byte
        # A real servo drives toward the goal; snap to it so position reads back
        # sensibly. Good enough to verify the driver, not a physics model.
        if addr <= ADDR_GOAL_POS_L < addr + len(data) + 1:
            self._set_word(ADDR_PRESENT_POS_L, self.get_word(ADDR_GOAL_POS_L))


def checksum(payload):
    return (~sum(payload)) & 0xFF


class FakeBus:
    def __init__(self, ids, overcurrent_id=None, log_path=None):
        self.servos = {
            i: FakeServo(i, overcurrent=(i == overcurrent_id)) for i in ids
        }
        self.log = []
        self.log_path = log_path

    def record(self, event, **kw):
        self.log.append(dict(event=event, t=round(time.monotonic(), 4), **kw))
        if self.log_path:
            with open(self.log_path, "w") as f:
                json.dump(self.log, f, indent=1)

    def status(self, servo_id, params=()):
        params = list(params)
        body = [servo_id, len(params) + 2, 0] + params
        return bytes([0xFF, 0xFF] + body + [checksum(body)])

    def handle(self, pkt):
        """pkt starts after the FF FF header. Returns a reply (or b'')."""
        servo_id, length, inst = pkt[0], pkt[1], pkt[2]
        params = pkt[3:3 + length - 2]

        if inst == INST_PING:
            if servo_id in self.servos:
                self.record("ping", id=servo_id)
                return self.status(servo_id)
            return b""

        if inst == INST_READ:
            addr, n = params[0], params[1]
            if servo_id not in self.servos:
                return b""
            data = list(self.servos[servo_id].mem[addr:addr + n])
            self.record("read", id=servo_id, addr=addr, length=n)
            return self.status(servo_id, data)

        if inst == INST_WRITE:
            addr, data = params[0], list(params[1:])
            if servo_id in self.servos:
                self.servos[servo_id].write(addr, data)
                self.record("write", id=servo_id, addr=addr, data=data)
                return self.status(servo_id)
            return b""

        if inst == INST_SYNC_WRITE:
            # FF FF FE LEN 83 ADDR NLEN [id d..] [id d..] ...  CHECKSUM
            addr, nlen = params[0], params[1]
            rest = params[2:]
            targets = []
            for off in range(0, len(rest), nlen + 1):
                chunk = rest[off:off + nlen + 1]
                if len(chunk) < nlen + 1:
                    break
                sid, data = chunk[0], list(chunk[1:])
                if sid in self.servos:
                    self.servos[sid].write(addr, data)
                targets.append({"id": sid, "data": data})
            self.record("sync_write", addr=addr, nlen=nlen, targets=targets)
            return b""  # sync write is not acknowledged

        self.record("unknown_instruction", inst=inst)
        return b""


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ids", default=",".join(str(i) for i in DEFAULT_IDS))
    parser.add_argument("--log", default=None, help="write a JSON event log here")
    parser.add_argument("--path-file", default=None, help="write the pty path here")
    parser.add_argument("--inject-overcurrent", type=int, default=None,
                        help="servo ID that should report a high present current")
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    bus = FakeBus(ids, args.inject_overcurrent, args.log)

    master_fd, slave_fd = pty.openpty()
    path = os.ttyname(slave_fd)
    if args.path_file:
        with open(args.path_file, "w") as f:
            f.write(path)
    print(f"[fake-bus] serving servos {ids} on {path}", flush=True)
    if args.inject_overcurrent:
        print(f"[fake-bus] servo {args.inject_overcurrent} reports overcurrent", flush=True)

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    deadline = time.monotonic() + args.seconds
    buf = b""

    try:
        while time.monotonic() < deadline:
            try:
                chunk = os.read(master_fd, 512)
            except OSError:
                break
            if not chunk:
                continue
            buf += chunk

            # Frame on the FF FF header; drop anything before it.
            while True:
                start = buf.find(b"\xff\xff")
                if start < 0 or len(buf) - start < 4:
                    break
                body = buf[start + 2:]
                length = body[1]
                total = 2 + 2 + length  # header + id + len + (inst..checksum)
                if len(buf) - start < total:
                    break
                pkt = buf[start + 2:start + total]
                buf = buf[start + total:]
                reply = bus.handle(pkt)
                if reply:
                    os.write(master_fd, reply)
    finally:
        if args.log:
            with open(args.log, "w") as f:
                json.dump(bus.log, f, indent=1)
        print(f"[fake-bus] {len(bus.log)} bus events", flush=True)
        os.close(master_fd)
        os.close(slave_fd)


if __name__ == "__main__":
    raise SystemExit(main())
