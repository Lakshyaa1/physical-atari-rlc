# ESP-IDF project for the CX40 -> devbox joystick bridge.
#
#   . ~/.espressif/tools/activate_idf_v6.0.sh
#   export PATH="$IDF_PATH/tools:$PATH"
#   idf.py set-target esp32
#   idf.py build
#   idf.py -p /dev/ttyUSB0 flash
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(joystick_bridge)
