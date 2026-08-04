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

#include "../include/robotroller.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace
{
// The reflex loop polls telemetry continuously on the same half-duplex bus that
// carries position commands. A short sleep keeps it from monopolising the bus
// while still checking every servo several hundred times a second -- far faster
// than any mechanical failure develops.
constexpr auto kReflexPollInterval = std::chrono::microseconds(500);

// How long torque stays off while the reflex releases mechanical tension.
constexpr auto kReflexTorqueOffTime = std::chrono::milliseconds(1);
} // namespace

Robotroller::ServoPositions Robotroller::getPositionsForAction(int action)
{
    const ServoPositions positions[18] = {
        {dpad_servo_default_, dpad_servo_default_, button_servo_default_}, // 0: NOOP
        {dpad_servo_default_, dpad_servo_default_, button_deflection_},    // 1: FIRE
        {dpad_servo_default_, dpad_servo_up_, button_servo_default_},      // 2: UP
        {dpad_servo_right_, dpad_servo_default_, button_servo_default_},   // 3: RIGHT
        {dpad_servo_left_, dpad_servo_default_, button_servo_default_},    // 4: LEFT
        {dpad_servo_default_, dpad_servo_down_, button_servo_default_},    // 5: DOWN
        {dpad_servo_right_, dpad_servo_up_, button_servo_default_},        // 6: UPRIGHT
        {dpad_servo_left_, dpad_servo_up_, button_servo_default_},         // 7: UPLEFT
        {dpad_servo_right_, dpad_servo_down_, button_servo_default_},      // 8: DOWNRIGHT
        {dpad_servo_left_, dpad_servo_down_, button_servo_default_},       // 9: DOWNLEFT
        {dpad_servo_default_, dpad_servo_up_, button_deflection_},         // 10: UPFIRE
        {dpad_servo_right_, dpad_servo_default_, button_deflection_},      // 11: RIGHTFIRE
        {dpad_servo_left_, dpad_servo_default_, button_deflection_},       // 12: LEFTFIRE
        {dpad_servo_default_, dpad_servo_down_, button_deflection_},       // 13: DOWNFIRE
        {dpad_servo_right_, dpad_servo_up_, button_deflection_},           // 14: UPRIGHTFIRE
        {dpad_servo_left_, dpad_servo_up_, button_deflection_},            // 15: UPLEFTFIRE
        {dpad_servo_right_, dpad_servo_down_, button_deflection_},         // 16: DOWNRIGHTFIRE
        {dpad_servo_left_, dpad_servo_down_, button_deflection_}           // 17: DOWNLEFTFIRE
    };

    return positions[action];
}

Robotroller::Robotroller(std::string device_path, int baud_rate, int position_d_gain,
                         int position_i_gain, int position_p_gain, int dpad_servo_default,
                         int dpad_servo_right, int dpad_servo_left, int dpad_servo_up,
                         int dpad_servo_down, int button_servo_default, int button_deflection,
                         int goal_speed, int goal_acc, int torque_limit, int overcurrent_counts)
    : fire_servo_(nullptr),
      left_right_servo_(nullptr),
      up_down_servo_(nullptr),
      new_action_to_execute_(-1),
      last_action_executed_(0),
      robotroller_thread_is_running_(true),
      dpad_servo_default_(dpad_servo_default),
      dpad_servo_right_(dpad_servo_right),
      dpad_servo_left_(dpad_servo_left),
      dpad_servo_up_(dpad_servo_up),
      dpad_servo_down_(dpad_servo_down),
      button_servo_default_(button_servo_default),
      button_deflection_(button_deflection),
      goal_speed_(goal_speed),
      goal_acc_(goal_acc),
      overcurrent_counts_(overcurrent_counts)
{
    std::cout << "[Robotroller] Opening Feetech servo bus on " << device_path << " at " << baud_rate
              << " baud" << std::endl;

    // SMS_STS owns the serial port, so a single instance IS the bus. (The
    // Dynamixel version needed a separate port handler and packet handler.)
    if (!bus_.begin(baud_rate, device_path.c_str()))
    {
        std::cerr << "[Robotroller] Failed to open the serial port to the Feetech servos!\n"
                  << "  Check the device path (prefer a stable /dev/serial/by-id/... path),\n"
                  << "  that the Waveshare adapter is connected, and that you have permission\n"
                  << "  (add yourself to the 'dialout' group)." << std::endl;
        exit(1);
    }
    std::cout << "[Robotroller] Serial port opened" << std::endl;

    fire_servo_ = new Servo(ROBOTROLLER_FIRE_SERVO_ID, &bus_);
    left_right_servo_ = new Servo(ROBOTROLLER_LEFT_RIGHT_SERVO_ID, &bus_);
    up_down_servo_ = new Servo(ROBOTROLLER_UP_DOWN_SERVO_ID, &bus_);

    list_of_servos_.push_back(fire_servo_);
    list_of_servos_.push_back(left_right_servo_);
    list_of_servos_.push_back(up_down_servo_);

    // Confirm every servo answers before touching anything. Without this a
    // missing servo shows up much later as a joystick that only half moves.
    for (Servo* servo : list_of_servos_)
    {
        if (bus_.Ping(static_cast<u8>(servo->id())) == -1)
        {
            std::cerr << "[Robotroller] Servo ID " << servo->id() << " did not respond.\n"
                      << "  Expected IDs " << ROBOTROLLER_FIRE_SERVO_ID << " (fire), "
                      << ROBOTROLLER_LEFT_RIGHT_SERVO_ID << " (left/right), "
                      << ROBOTROLLER_UP_DOWN_SERVO_ID << " (up/down) at " << baud_rate << " baud.\n"
                      << "  Feetech servos ship as ID 1 - assign IDs one servo at a time on an\n"
                      << "  otherwise empty bus before using the Robotroller." << std::endl;
            exit(1);
        }
    }
    std::cout << "[Robotroller] All three servos responded" << std::endl;

    // Configure each servo with torque off: mode and the PID coefficients live
    // in EEPROM, and writing them while the motor is driving is asking for a
    // lurch.
    for (Servo* servo : list_of_servos_)
    {
        servo->disableTorque();
        servo->setModeToPosition();
        servo->setPositionPGain(position_p_gain);
        servo->setPositionIGain(position_i_gain);
        servo->setPositionDGain(position_d_gain);
        servo->setTorqueLimit(torque_limit);
        servo->enableTorque();
    }

    std::cout << "[Robotroller] Configured (speed " << goal_speed_ << ", acc " << goal_acc_
              << ", overcurrent reflex at " << overcurrent_counts_ << " counts ~= "
              << static_cast<int>(overcurrent_counts_ * STS3215_MA_PER_CURRENT_COUNT) << " mA)"
              << std::endl;

    robotroller_thread_ = std::thread(&Robotroller::sendCommandsToServos, this);
}

Robotroller::~Robotroller()
{
    robotroller_thread_is_running_ = false;

    if (robotroller_thread_.joinable())
    {
        robotroller_thread_.join();
    }

    // Torque off on every exit path, so the joystick is never left held.
    for (Servo* servo : list_of_servos_)
    {
        servo->disableTorque();
    }

    delete fire_servo_;
    delete left_right_servo_;
    delete up_down_servo_;

    bus_.end();
}

void Robotroller::applyOvercurrentReflex()
{
    // The high-current reflex (paper section 2.1 / appendix B.3): a stalled or
    // obstructed servo draws heavily, which both damages the gearbox and can
    // latch the servo into a fault needing a manual power cycle. Freezing the
    // goal at the present position and briefly cycling torque releases the
    // mechanical tension. This is what makes unattended multi-day runs safe --
    // the learning agent physically cannot destroy the hardware.
    for (Servo* servo : list_of_servos_)
    {
        // One round trip fetches position/load/voltage/temperature/current
        // together; the cached reads below are free.
        if (!servo->refreshTelemetry())
        {
            continue; // transient bus error; try again next pass
        }

        const int current = servo->cachedCurrent();
        if (std::abs(current) <= overcurrent_counts_)
        {
            continue;
        }

        const int position = servo->cachedPosition();
        std::cout << "[Robotroller] Overcurrent on servo " << servo->id() << ": " << current
                  << " counts (~" << static_cast<int>(std::abs(current) * STS3215_MA_PER_CURRENT_COUNT)
                  << " mA), temp " << servo->cachedTemperature() << "C - holding at " << position
                  << " and cycling torque" << std::endl;

        if (position >= 0)
        {
            servo->setPosition(position, goal_speed_, goal_acc_);
        }
        servo->disableTorque();
        std::this_thread::sleep_for(kReflexTorqueOffTime);
        servo->enableTorque();
    }
}

void Robotroller::sendCommandsToServos()
{
    while (robotroller_thread_is_running_)
    {
        int action = -1;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            action = new_action_to_execute_;
        }

        if (action != -1)
        {
            const ServoPositions pos = getPositionsForAction(action);

            // All three servos in ONE bus transaction. The Dynamixel version
            // issued three sequential writes, so the joystick's axes started
            // moving at measurably different times; a sync write removes that
            // skew and frees bus time for the reflex's telemetry reads.
            u8 ids[3] = {static_cast<u8>(left_right_servo_->id()),
                         static_cast<u8>(up_down_servo_->id()),
                         static_cast<u8>(fire_servo_->id())};
            s16 targets[3] = {static_cast<s16>(pos.left_right), static_cast<s16>(pos.up_down),
                              static_cast<s16>(pos.fire)};
            u16 speeds[3] = {static_cast<u16>(goal_speed_), static_cast<u16>(goal_speed_),
                             static_cast<u16>(goal_speed_)};
            u8 accs[3] = {static_cast<u8>(goal_acc_), static_cast<u8>(goal_acc_),
                          static_cast<u8>(goal_acc_)};

            bus_.SyncWritePosEx(ids, 3, targets, speeds, accs);

            std::lock_guard<std::mutex> lock(command_mutex_);
            last_action_executed_ = action;
            new_action_to_execute_ = -1;
        }

        applyOvercurrentReflex();

        std::this_thread::sleep_for(kReflexPollInterval);
    }
}

void Robotroller::setAction(int action)
{
    std::lock_guard<std::mutex> lock(command_mutex_);

    // Already commanded, or already the servos' current pose - nothing to do.
    if (action == new_action_to_execute_ ||
        (new_action_to_execute_ == -1 && action == last_action_executed_))
    {
        return;
    }

    // A write is already in flight. Drop this one rather than block the
    // caller's real-time loop on serial I/O.
    if (new_action_to_execute_ != -1)
    {
        return;
    }

    new_action_to_execute_ = action;
}
