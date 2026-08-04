cmake_minimum_required(VERSION 3.21)
project(robotroller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Threads package (needed for std::thread on Linux)
find_package(Threads REQUIRED)

# Find OpenCV package (needed for camera capture)
find_package(OpenCV REQUIRED)

# Find pybind11 package (needed for Python bindings)
find_package(pybind11 REQUIRED)

# Find apriltag package (needed for AprilTag detection)
find_package(apriltag REQUIRED)

# ---------------------------------------------------------------------------
# Feetech SMS/STS servo SDK (SCServo_Linux)
#
# This project originally used the ROBOTIS DynamixelSDK. It now drives Feetech
# STS3215 servos over a Waveshare bus servo adapter, so it links SCServo_Linux
# instead: https://github.com/adityakamath/SCServo_Linux
#
# Resolution order:
#   1. -DSCSERVO_ROOT=/path/to/SCServo_Linux  (explicit, wins)
#   2. a sibling/parent checkout, which is how this repo is laid out
#   3. an installed copy found via find_package/find_library
# ---------------------------------------------------------------------------
set(SCSERVO_ROOT "" CACHE PATH "Path to a SCServo_Linux source checkout")

if(NOT SCSERVO_ROOT)
    foreach(_candidate
            "${CMAKE_CURRENT_SOURCE_DIR}/../SCServo_Linux"
            "${CMAKE_CURRENT_SOURCE_DIR}/../../SCServo_Linux"
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party/SCServo_Linux")
        if(EXISTS "${_candidate}/include/scservo/SMS_STS.h")
            get_filename_component(SCSERVO_ROOT "${_candidate}" ABSOLUTE)
            break()
        endif()
    endforeach()
endif()

if(SCSERVO_ROOT AND EXISTS "${SCSERVO_ROOT}/include/scservo/SMS_STS.h")
    message(STATUS "Using SCServo_Linux source at ${SCSERVO_ROOT}")
    # EXCLUDE_FROM_ALL keeps SCServo's ~40 example binaries out of our build.
    add_subdirectory("${SCSERVO_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/scservo" EXCLUDE_FROM_ALL)
    set(SCSERVO_TARGET SCServo)
else()
    find_library(SCSERVO_LIB NAMES SCServo scservo)
    find_path(SCSERVO_INCLUDE_DIR scservo/SMS_STS.h)
    if(NOT SCSERVO_LIB OR NOT SCSERVO_INCLUDE_DIR)
        message(FATAL_ERROR
            "SCServo_Linux not found.\n"
            "  Clone it next to this repository:\n"
            "    git clone https://github.com/adityakamath/SCServo_Linux\n"
            "  or point cmake at an existing checkout:\n"
            "    cmake .. -DSCSERVO_ROOT=/path/to/SCServo_Linux")
    endif()
    message(STATUS "Using installed SCServo: ${SCSERVO_LIB}")
    add_library(SCServo_imported UNKNOWN IMPORTED)
    set_target_properties(SCServo_imported PROPERTIES
        IMPORTED_LOCATION "${SCSERVO_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${SCSERVO_INCLUDE_DIR}")
    set(SCSERVO_TARGET SCServo_imported)
endif()

# Build the Python module
pybind11_add_module(robotroller bindings.cpp src/servo.cpp src/robotroller.cpp src/camera.cpp src/apriltag_detector.cpp src/physical_atari_env.cpp)

# Link Python module against required libraries
target_link_libraries(robotroller PRIVATE Threads::Threads)
target_link_libraries(robotroller PRIVATE ${OpenCV_LIBS})
target_link_libraries(robotroller PRIVATE apriltag)
target_link_libraries(robotroller PRIVATE ${SCSERVO_TARGET})

# Install the Python module to the active Python environment
execute_process(
    COMMAND python -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

install(TARGETS robotroller
        LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES})

# Print installation instructions
message(STATUS "")
message(STATUS "To build and install the Python module:")
message(STATUS "  mkdir -p build && cd build")
message(STATUS "  cmake ..")
message(STATUS "  make")
message(STATUS "  sudo make install")
message(STATUS "")
