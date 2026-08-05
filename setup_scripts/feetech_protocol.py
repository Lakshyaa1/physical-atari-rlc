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
feetech_protocol.py - minimal Feetech SMS/STS serial protocol over pyserial.

Enough of the protocol to configure servos before first use: ping, read, write,
scan, and EEPROM lock/unlock. Deliberately dependency-light (pyserial only) so
that bringing up a fresh robot does not require building the C++ SDK first.

Replaces the DynamixelSDK-based scripts this project shipped with. The wire
format is Dynamixel-protocol-1.0-like but the register map and the servos are
Feetech's:

    request : FF FF  ID  LEN  INST  PARAM...  CHECKSUM
    reply   : FF FF  ID  LEN  ERR   PARAM...  CHECKSUM
    LEN      = len(params) + 2
    CHECKSUM = (~(ID + LEN + INST + sum(params))) & 0xFF

For the servo-side register map see scservo/SMS_STS.h in SCServo_Linux.
"""

import time

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover
    raise SystemExit(
        "pyserial is required.\n"
        "  pip install -r setup_scripts/requirements.txt"
    )

# --- instructions ----------------------------------------------------------
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

# --- SMS/STS register addresses (subset) -----------------------------------
ADDR_MODEL_L = 3
ADDR_ID = 5
ADDR_BAUD_RATE = 6
ADDR_MIN_ANGLE_LIMIT_L = 9
ADDR_MAX_ANGLE_LIMIT_L = 11
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
ADDR_PRESENT_POSITION_L = 56

BROADCAST_ID = 0xFE

# Feetech encodes the baud rate as a small index, not the rate itself.
# NOTE: STS3215 servos ship at 1,000,000 baud (index 0) -- unlike Dynamixel,
# which ships at 57600. Do not assume the factory rate is slow.
BAUD_INDEX = {
    1000000: 0,
    500000: 1,
    250000: 2,
    128000: 3,
    115200: 4,
    76800: 5,
    57600: 6,
    38400: 7,
}
INDEX_BAUD = {v: k for k, v in BAUD_INDEX.items()}


def checksum(payload):
    """payload = [id, length, instruction/error, *params]"""
    return (~sum(payload)) & 0xFF


class FeetechBus:
    """A half-duplex Feetech servo bus on one serial port."""

    def __init__(self, path, baud, timeout=0.02):
        self.path = path
        self.baud = baud
        try:
            self.port = serial.Serial(path, baud, timeout=timeout, write_timeout=timeout)
        except serial.SerialException as e:
            raise SystemExit(
                f"Could not open {path} at {baud} baud: {e}\n"
                "  - check the path (prefer a stable /dev/serial/by-id/... entry)\n"
                "  - check you are in the 'dialout' group\n"
                "  - check the servo's external power supply is on"
            )

    def close(self):
        self.port.close()

    # --- framing -----------------------------------------------------------

    def _send(self, servo_id, instruction, params=()):
        params = list(params)
        length = len(params) + 2
        body = [servo_id, length, instruction] + params
        packet = bytes([0xFF, 0xFF] + body + [checksum(body)])
        self.port.reset_input_buffer()
        self.port.write(packet)
        self.port.flush()

    def _recv(self, expected_params=0):
        """Read one status packet. Returns (error_byte, params) or None."""
        want = 6 + expected_params  # FF FF ID LEN ERR ... CHECKSUM
        deadline = time.monotonic() + 0.05
        buf = b""
        while len(buf) < want and time.monotonic() < deadline:
            chunk = self.port.read(want - len(buf))
            if not chunk:
                break
            buf += chunk

        # Resynchronise on the 0xFF 0xFF header: on a half-duplex bus an
        # adapter may echo part of our own transmission back at us.
        start = buf.find(b"\xff\xff")
        if start < 0 or len(buf) - start < 6:
            return None
        frame = buf[start:]

        servo_id, length, error = frame[2], frame[3], frame[4]
        # A full packet is 4 + LEN bytes (FF FF ID LEN, then LEN more), so the
        # checksum is the last one, at index 3 + LEN -- not 4 + LEN.
        if len(frame) < 4 + length:
            return None
        params = list(frame[5:5 + max(0, length - 2)])
        expected = checksum([servo_id, length, error] + params)
        if frame[3 + length] != expected:
            return None
        return error, params

    # --- operations --------------------------------------------------------

    def ping(self, servo_id):
        self._send(servo_id, INST_PING)
        return self._recv(0) is not None

    def read(self, servo_id, addr, length):
        self._send(servo_id, INST_READ, [addr, length])
        reply = self._recv(length)
        if reply is None:
            return None
        return reply[1][:length]

    def read_byte(self, servo_id, addr):
        data = self.read(servo_id, addr, 1)
        return data[0] if data else None

    def read_word(self, servo_id, addr):
        data = self.read(servo_id, addr, 2)
        return (data[1] << 8) | data[0] if data else None

    def write(self, servo_id, addr, values):
        self._send(servo_id, INST_WRITE, [addr] + list(values))
        if servo_id == BROADCAST_ID:
            return True
        return self._recv(0) is not None

    def write_byte(self, servo_id, addr, value):
        return self.write(servo_id, addr, [value & 0xFF])

    # --- EEPROM ------------------------------------------------------------

    def unlock_eeprom(self, servo_id):
        return self.write_byte(servo_id, ADDR_LOCK, 0)

    def lock_eeprom(self, servo_id):
        return self.write_byte(servo_id, ADDR_LOCK, 1)

    # --- discovery ---------------------------------------------------------

    def scan(self, ids=range(0, 253), progress=False):
        """Return the sorted IDs that answered a ping."""
        found = []
        for servo_id in ids:
            if self.ping(servo_id):
                found.append(servo_id)
                if progress:
                    print(f"  found servo ID {servo_id}")
        return found


def scan_all_bauds(path, bauds=(1000000, 500000, 250000, 128000, 115200, 57600, 38400)):
    """Scan every common baud rate. Useful when you do not know how a servo is set."""
    results = {}
    for baud in bauds:
        bus = FeetechBus(path, baud)
        try:
            found = bus.scan()
            if found:
                results[baud] = found
        finally:
            bus.close()
    return results
