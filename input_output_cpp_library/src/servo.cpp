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

#include "../include/servo.h"

#include <algorithm>
#include <iostream>

Servo::Servo(int id, SMS_STS* bus) : id_(id), bus_(bus) {}

Servo::~Servo()
{
    // Best effort: a failure here has nowhere useful to go, and the caller is
    // already shutting down. Safe because Robotroller declares its SMS_STS bus
    // before the servos, so the bus is destroyed last.
    disableTorque();
}

bool Servo::writeEepromByteIfChanged(int addr, int value, const char* what)
{
    // EEPROM cells wear out. The gains and mode are written on every startup,
    // so read first and skip the write when nothing would change -- otherwise a
    // few thousand restarts would burn through the cell's erase budget for no
    // reason at all.
    const int current = bus_->readByte(static_cast<u8>(id_), static_cast<u8>(addr));
    if (current == value)
    {
        return true;
    }
    if (current < 0)
    {
        std::cerr << "[Servo " << id_ << "] could not read " << what << " (addr " << addr
                  << ") - is the servo powered and on the bus?" << std::endl;
        return false;
    }

    if (bus_->unLockEeprom(static_cast<u8>(id_)) != 1)
    {
        std::cerr << "[Servo " << id_ << "] failed to unlock EEPROM to set " << what << std::endl;
        return false;
    }

    const int rc = bus_->writeByte(static_cast<u8>(id_), static_cast<u8>(addr),
                                   static_cast<u8>(value));

    // Re-lock even if the write failed: leaving EEPROM writable invites silent
    // corruption from any later stray packet on a shared bus.
    const int locked = bus_->LockEeprom(static_cast<u8>(id_));

    if (rc != 1)
    {
        std::cerr << "[Servo " << id_ << "] failed to write " << what << " = " << value << std::endl;
        return false;
    }
    if (locked != 1)
    {
        std::cerr << "[Servo " << id_ << "] WARNING: EEPROM left unlocked after setting " << what
                  << std::endl;
    }

    std::cout << "[Servo " << id_ << "] " << what << ": " << current << " -> " << value
              << " (EEPROM)" << std::endl;
    return true;
}

bool Servo::setModeToPosition()
{
    return writeEepromByteIfChanged(ADDR_SERVO_MODE, FEETECH_MODE_POSITION, "operating mode");
}

bool Servo::setPositionPGain(int gain)
{
    if (gain < 0)
    {
        return true; // leave whatever is programmed
    }
    if (gain > FEETECH_MAX_PID_COEF)
    {
        std::cerr << "[Servo " << id_ << "] P gain " << gain << " exceeds the 1-byte maximum "
                  << FEETECH_MAX_PID_COEF
                  << ". Feetech coefficients are NOT on the same scale as Dynamixel's - "
                     "the Physical Atari paper's values (1500/6000/1500) do not apply here."
                  << std::endl;
        return false;
    }
    return writeEepromByteIfChanged(ADDR_MODE0_P_COEF, gain, "P coefficient");
}

bool Servo::setPositionIGain(int gain)
{
    if (gain < 0)
    {
        return true;
    }
    if (gain > FEETECH_MAX_PID_COEF)
    {
        std::cerr << "[Servo " << id_ << "] I gain " << gain << " exceeds the 1-byte maximum "
                  << FEETECH_MAX_PID_COEF << std::endl;
        return false;
    }
    return writeEepromByteIfChanged(ADDR_MODE0_I_COEF, gain, "I coefficient");
}

bool Servo::setPositionDGain(int gain)
{
    if (gain < 0)
    {
        return true;
    }
    if (gain > FEETECH_MAX_PID_COEF)
    {
        std::cerr << "[Servo " << id_ << "] D gain " << gain << " exceeds the 1-byte maximum "
                  << FEETECH_MAX_PID_COEF << std::endl;
        return false;
    }
    return writeEepromByteIfChanged(ADDR_MODE0_D_COEF, gain, "D coefficient");
}

bool Servo::setTorqueLimit(int limit)
{
    if (limit < 0)
    {
        return true;
    }
    limit = std::min(limit, FEETECH_MAX_TORQUE_LIMIT);
    // SRAM, not EEPROM - safe to write unconditionally on every startup.
    if (bus_->writeWord(static_cast<u8>(id_), ADDR_TORQUE_LIMIT, static_cast<u16>(limit)) != 1)
    {
        std::cerr << "[Servo " << id_ << "] failed to set torque limit" << std::endl;
        return false;
    }
    return true;
}

bool Servo::setPosition(int position, int speed, int acc)
{
    return bus_->WritePosEx(static_cast<u8>(id_), static_cast<s16>(position),
                            static_cast<u16>(speed), static_cast<u8>(acc)) == 1;
}

bool Servo::enableTorque()
{
    return bus_->EnableTorque(static_cast<u8>(id_), 1) == 1;
}

bool Servo::disableTorque()
{
    return bus_->EnableTorque(static_cast<u8>(id_), 0) == 1;
}

int Servo::getPresentPosition()
{
    return bus_->ReadPos(id_);
}

int Servo::getPresentCurrent()
{
    return bus_->ReadCurrent(id_);
}

bool Servo::refreshTelemetry()
{
    // FeedBack() returns 1 on success and 0 on failure (NOT -1 -- several other
    // calls in this SDK use -1, so this is easy to get wrong). Getting it wrong
    // matters: on a failed read the cache still holds the PREVIOUS servo's
    // values, and the overcurrent reflex would then be deciding on stale data.
    return bus_->FeedBack(id_) == 1;
}

int Servo::cachedCurrent()
{
    return bus_->ReadCurrent(-1);
}

int Servo::cachedPosition()
{
    return bus_->ReadPos(-1);
}

int Servo::cachedTemperature()
{
    return bus_->ReadTemper(-1);
}

int Servo::cachedVoltage()
{
    return bus_->ReadVoltage(-1);
}

int Servo::cachedLoad()
{
    return bus_->ReadLoad(-1);
}
