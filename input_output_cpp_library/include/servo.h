// Copyright 2026 Keen Technologies, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// servo.h
//
// One Feetech SMS/STS bus servo (e.g. STS3215), addressed by ID on a shared
// half-duplex serial bus.
//
// PORTED FROM DYNAMIXEL. Three differences from the original Dynamixel-based
// implementation are load-bearing and are the usual source of bugs:
//
//  1. PID GAINS LIVE IN EEPROM, NOT RAM, and are single bytes (0-254). The
//     Dynamixel equivalents were 2-byte RAM registers holding values like 1500
//     and 6000. The numbers published in the Physical Atari paper therefore do
//     NOT carry over -- neither the scale nor the semantics match, and writing
//     1500 into a one-byte register is meaningless. Retune from scratch.
//     Because these are EEPROM, every write costs a limited erase cycle, so
//     setPosition*Gain() reads the current value first and skips the write when
//     it already matches.
//
//  2. GOAL POSITION IS A 2-BYTE REGISTER (addr 42), not 4-byte. Code that
//     assumes a 32-bit write will scribble over neighbouring registers.
//
//  3. PRESENT CURRENT IS IN SERVO UNITS, NOT MILLIAMPS. On the STS3215 one
//     count is roughly 6.5 mA, so the paper's 1200 mA reflex threshold is about
//     185 counts. getPresentCurrent() returns raw counts; the threshold is a
//     configurable value in the same units. Do not port 1200 across literally.
//
// The bus (an SMS_STS instance owning the serial port) is shared by every servo
// and is NOT owned by this class.

#ifndef SERVO_H
#define SERVO_H

#include <string>

#include <scservo/SMS_STS.h>

// ---- SMS/STS register addresses used here (see scservo/SMS_STS.h) ----------
// EEPROM, read/write. Mode 0 == position/servo mode.
#define ADDR_MODE0_P_COEF SMS_STS_MODE0_P_COEF // 21, 1 byte
#define ADDR_MODE0_D_COEF SMS_STS_MODE0_D_COEF // 22, 1 byte
#define ADDR_MODE0_I_COEF SMS_STS_MODE0_I_COEF // 23, 1 byte
#define ADDR_SERVO_MODE SMS_STS_MODE           // 33, 1 byte
// SRAM, read/write.
#define ADDR_TORQUE_LIMIT SMS_STS_TORQUE_LIMIT_L // 48, 2 bytes, 0-1000

// Feetech position mode is 0 (Dynamixel's was 3).
#define FEETECH_MODE_POSITION SMS_STS_MODE_SERVO

// Highest legal value for a 1-byte PID coefficient.
#define FEETECH_MAX_PID_COEF 254

// Full-scale value for the torque limit register.
#define FEETECH_MAX_TORQUE_LIMIT 1000

// Approximate milliamps per count of present-current on the STS3215. Used only
// for human-readable logging; all thresholds are handled in raw counts.
#define STS3215_MA_PER_CURRENT_COUNT 6.5

class Servo
{
  public:
    // `bus` must outlive this object and is shared between all servos.
    Servo(int id, SMS_STS* bus);

    int id() const { return id_; }

    // --- configuration -----------------------------------------------------

    // Put the servo into position (servo) mode. Writes EEPROM, so it is
    // skipped when the servo is already in that mode.
    bool setModeToPosition();

    // PID coefficients, 0-254, stored in EEPROM. A negative value means "leave
    // whatever is already programmed alone" -- note that 0 is a legitimate
    // value to write (it is the factory default for I and D), which is why
    // "don't touch" cannot be signalled with 0 the way the Dynamixel code did.
    bool setPositionPGain(int gain);
    bool setPositionIGain(int gain);
    bool setPositionDGain(int gain);

    // Torque ceiling, 0-1000 (~0.1% each). SRAM, so it reverts on power cycle.
    bool setTorqueLimit(int limit);

    // --- motion ------------------------------------------------------------

    // Move to `position` (0-4095) with the given speed (steps/s) and
    // acceleration (units of 100 steps/s^2).
    bool setPosition(int position, int speed, int acc);

    bool enableTorque();
    bool disableTorque();

    // --- feedback ----------------------------------------------------------

    // Each of these is its own bus round trip. Prefer refreshTelemetry() plus
    // the cached* accessors when you want more than one value.
    int getPresentPosition();
    int getPresentCurrent(); // raw counts, signed; see note 3 above

    // Read position/speed/load/voltage/temperature/current in a single
    // transaction. The cached* accessors below then cost nothing.
    //
    // The underlying SDK keeps ONE shared cache buffer for the whole bus, so
    // the cached values belong to whichever servo most recently refreshed.
    // Always refresh and read one servo at a time.
    bool refreshTelemetry();

    int cachedCurrent();     // raw counts, signed
    int cachedPosition();    // 0-4095
    int cachedTemperature(); // degrees C
    int cachedVoltage();     // 0.1 V units
    int cachedLoad();        // +/-1000

  private:
    // Write a single EEPROM byte only if it differs from what is stored,
    // wrapping the write in the unlock/relock the SMS/STS protocol requires.
    bool writeEepromByteIfChanged(int addr, int value, const char* what);

    int id_;
    SMS_STS* bus_; // shared, not owned
};

#endif // SERVO_H
