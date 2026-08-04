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

// robotroller.h
//
// Drives the three bus servos that actuate an unmodified Atari CX40 joystick.
//
// PORTED FROM DYNAMIXEL TO FEETECH SMS/STS (STS3215). The public interface is
// unchanged -- setAction() plus a background thread that carries out the move
// and runs the high-current reflex -- so nothing above this file had to change
// beyond passing the new configuration values through.
//
// What changed underneath:
//   * One shared SMS_STS bus object replaces the Dynamixel port/packet handler
//     pair. SMS_STS *is* the serial port, so there is exactly one of them.
//   * All three servo positions are now written in a SINGLE SyncWritePosEx
//     packet instead of three sequential round trips. Lower and, more usefully,
//     far more consistent actuation latency, and less contention with the
//     reflex's telemetry reads on the same half-duplex bus.
//   * The reflex threshold is in raw current counts (~6.5 mA each on an
//     STS3215), not milliamps. See servo.h.
//   * Servo IDs stay at 50/51/52 to match the rest of the codebase and to avoid
//     colliding with the Feetech factory default of ID 1.

#ifndef ROBOTROLLER_H
#define ROBOTROLLER_H

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <scservo/SMS_STS.h>

#include "servo.h"

// Servo IDs on the bus.
#define ROBOTROLLER_FIRE_SERVO_ID 50
#define ROBOTROLLER_LEFT_RIGHT_SERVO_ID 51
#define ROBOTROLLER_UP_DOWN_SERVO_ID 52

class Robotroller
{
  public:
    // `position_*_gain` are Feetech 1-byte EEPROM coefficients (0-254); pass a
    // negative value to leave the servo's programmed value alone. These are NOT
    // the Dynamixel gains from the paper -- see servo.h.
    //
    // `overcurrent_counts` is the high-current reflex threshold in raw current
    // counts (~6.5 mA each). Calibrate it by stalling a servo by hand and
    // reading the reported value; do not assume the paper's 1200 mA.
    //
    // `goal_speed` is in steps/s and `goal_acc` in units of 100 steps/s^2.
    // These replace the Dynamixel profile registers and shape how hard the
    // joystick is thrown -- too aggressive wears out the controller, which is
    // exactly the failure the paper describes.
    Robotroller(std::string device_path, int baud_rate, int position_d_gain, int position_i_gain,
                int position_p_gain, int dpad_servo_default, int dpad_servo_right,
                int dpad_servo_left, int dpad_servo_up, int dpad_servo_down,
                int button_servo_default, int button_deflection, int goal_speed, int goal_acc,
                int torque_limit, int overcurrent_counts);
    ~Robotroller();

    Robotroller(const Robotroller&) = delete;
    Robotroller& operator=(const Robotroller&) = delete;

    // Hand the servo thread a new action (0-17). Non-blocking: if a write is
    // already in flight, or the action matches what is already commanded, the
    // request is dropped so the caller's real-time loop never stalls on serial
    // I/O. Some commanded actions therefore never reach the joystick.
    void setAction(int action);

  private:
    struct ServoPositions
    {
        int left_right;
        int up_down;
        int fire;
    };

    ServoPositions getPositionsForAction(int action);
    void sendCommandsToServos();
    void applyOvercurrentReflex();

    SMS_STS bus_;

    Servo* fire_servo_;
    Servo* left_right_servo_;
    Servo* up_down_servo_;
    std::vector<Servo*> list_of_servos_;

    int new_action_to_execute_;
    int last_action_executed_;
    std::mutex command_mutex_;
    std::atomic<bool> robotroller_thread_is_running_;
    std::thread robotroller_thread_;

    // Servo position parameters (encoder counts, 0-4095). Per-robot
    // calibration; the paper's values are for a different body and a different
    // servo family.
    int dpad_servo_default_;
    int dpad_servo_right_;
    int dpad_servo_left_;
    int dpad_servo_up_;
    int dpad_servo_down_;
    int button_servo_default_;
    int button_deflection_;

    // Motion profile and safety.
    int goal_speed_;
    int goal_acc_;
    int overcurrent_counts_;
};

#endif // ROBOTROLLER_H
