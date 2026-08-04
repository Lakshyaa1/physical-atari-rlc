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

cmake_minimum_required(VERSION 3.21)

project(devbox_ale)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)

find_package(SDL2 REQUIRED COMPONENTS SDL2)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

# libgpiod is only needed for the original Raspberry Pi devbox, which reads the
# CX40 switches straight off the GPIO header. On any other host the switches
# arrive over serial from an MCU instead (--input=serial), so GPIO support is
# optional: we compile it in when a suitable libgpiod is present and quietly
# leave it out otherwise. Force either way with -DDEVBOX_USE_GPIOD=ON/OFF.
# Tri-state (AUTO/ON/OFF), so it must be a STRING cache entry rather than an
# option() — option() is a BOOL and would silently coerce AUTO to OFF.
set(DEVBOX_USE_GPIOD "AUTO" CACHE STRING
    "Build the Raspberry Pi GPIO joystick input path (AUTO/ON/OFF)")
set_property(CACHE DEVBOX_USE_GPIOD PROPERTY STRINGS AUTO ON OFF)

if(DEVBOX_USE_GPIOD STREQUAL "AUTO")
    pkg_check_modules(GPIOD QUIET libgpiod)
    if(GPIOD_FOUND AND NOT GPIOD_VERSION VERSION_LESS "2.0")
        set(DEVBOX_USE_GPIOD ON)
        message(STATUS "libgpiod ${GPIOD_VERSION} found - enabling GPIO input")
    else()
        set(DEVBOX_USE_GPIOD OFF)
        if(GPIOD_FOUND)
            message(STATUS "libgpiod ${GPIOD_VERSION} found but >= 2.0 required "
                           "- GPIO input disabled (use --input=serial)")
        else()
            message(STATUS "libgpiod not found - GPIO input disabled "
                           "(use --input=serial)")
        endif()
    endif()
elseif(DEVBOX_USE_GPIOD)
    pkg_check_modules(GPIOD REQUIRED libgpiod)
    if(GPIOD_VERSION VERSION_LESS "2.0")
        message(FATAL_ERROR "libgpiod >= 2.0 required (found ${GPIOD_VERSION})")
    endif()
endif()

include_directories(${SDL2_INCLUDE_DIRS})

set(IMGUI
        imgui/imgui_theme.cpp
        imgui/imgui.cpp imgui/imgui_draw.cpp imgui/imgui_tables.cpp
        imgui/imgui_widgets.cpp
        imgui/imconfig.h imgui/imgui.h imgui/imgui_internal.h
        imgui/imstb_rectpack.h imgui/imstb_textedit.h imgui/imstb_truetype.h
        imgui/imgui_impl_sdl2.cpp imgui/imgui_impl_sdlrenderer2.cpp
)


add_executable(PhysicalALE main.cpp ${IMGUI} include/json.hpp include/joystick_input.h)
add_subdirectory(src)

target_link_libraries(PhysicalALE PRIVATE SDL2::SDL2
        Threads::Threads ${OpenCV_LIBS}
        ZLIB::ZLIB Threads::Threads pthread Atari)

if(DEVBOX_USE_GPIOD)
    target_compile_definitions(PhysicalALE PRIVATE DEVBOX_HAVE_GPIOD)
    target_include_directories(PhysicalALE PRIVATE ${GPIOD_INCLUDE_DIRS})
    target_link_libraries(PhysicalALE PRIVATE ${GPIOD_LIBRARIES})
endif()

# Install the binary to /usr/local/bin
install(TARGETS PhysicalALE
        RUNTIME DESTINATION bin
        COMPONENT Runtime)