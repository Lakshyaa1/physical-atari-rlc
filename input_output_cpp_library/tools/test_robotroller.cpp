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

// test_robotroller.cpp
//
// Drives the real Robotroller against tools/fake_feetech_bus.py, which emulates
// three STS3215 servos on a pseudo-terminal. Verifies the driver end to end --
// startup ping, EEPROM configuration, sync-write actuation, telemetry, and
// torque-off on shutdown -- without any hardware.
//
// Build (from input_output_cpp_library/):
//   g++ -std=c++17 -I ../../SCServo_Linux/include tools/test_robotroller.cpp \
//       src/servo.cpp src/robotroller.cpp \
//       -L ../../SCServo_Linux/build -lSCServo -lpthread -o /tmp/test_robotroller
//
// Usage: test_robotroller <pty-path> [action ...]

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "../include/robotroller.h"

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: " << argv[0] << " <serial-device> [action ...]" << std::endl;
        return 2;
    }
    const std::string device = argv[1];

    // Deliberately distinct values per axis so the sync-write payload can be
    // attributed to the right servo when the log is checked.
    const int kDpadDefault = 2048;
    const int kDpadRight = 2130;
    const int kDpadLeft = 1925;
    const int kDpadUp = 2180;
    const int kDpadDown = 1960;
    const int kButtonDefault = 2000;
    const int kButtonPressed = 1932;

    std::cout << "--- constructing Robotroller ---" << std::endl;
    {
        Robotroller robot(device, 1000000,
                          /*d*/ 5, /*i*/ 10, /*p*/ 40,
                          kDpadDefault, kDpadRight, kDpadLeft, kDpadUp, kDpadDown,
                          kButtonDefault, kButtonPressed,
                          /*goal_speed*/ 2400, /*goal_acc*/ 0,
                          /*torque_limit*/ 500, /*overcurrent_counts*/ 185);

        std::cout << "--- issuing actions ---" << std::endl;
        for (int i = 2; i < argc; ++i)
        {
            const int action = std::atoi(argv[i]);
            std::cout << "setAction(" << action << ")" << std::endl;
            robot.setAction(action);
            // Let the servo thread pick the action up and write it.
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }

        std::cout << "--- destructing (expect torque off) ---" << std::endl;
    }
    std::cout << "--- done ---" << std::endl;
    return 0;
}
