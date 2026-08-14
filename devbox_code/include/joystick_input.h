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

// joystick_input.h
//
// Reads the state of the CX40 controller's five switches (up/down/left/right/
// fire) for the devbox.
//
// The original Raspberry Pi devbox read those five wires directly on GPIO pins
// via libgpiod. That is not portable to hardware without usable GPIO — an x86
// laptop exposes /dev/gpiochip0 (the chipset's own controller) but no pins you
// can land a wire on. This header abstracts the read behind an interface with
// three implementations:
//
//   SerialInput   — an MCU (e.g. ESP32) reads the five wires and streams the
//                   state over USB CDC. This is the portable production path.
//   GpioInput     — libgpiod, for an actual Raspberry Pi devbox. Compiled only
//                   when DEVBOX_HAVE_GPIOD is defined.
//   KeyboardInput — arrow keys + space. No robot hardware at all; for bringing
//                   up the emulator, tags, and rendering on their own.
//
// Wire protocol for SerialInput — one byte per sample:
//
//     bit 7 : always 1 (frame marker)
//     bit 4 : fire        bit 3 : right      bit 2 : left
//     bit 1 : down        bit 0 : up
//     (1 = pressed)
//
// The payload occupies only bits 0-4, so a payload byte can never set bit 7.
// That makes the stream self-synchronising: any byte with the high bit set is a
// complete, valid frame, and a reader that has lost sync recovers on the very
// next byte. There is no framing state machine and nothing to get stuck in.
//
// The MCU should transmit continuously (~1 kHz is ample) rather than only on
// change. The devbox drains whatever has arrived and keeps the newest frame, so
// a free-running stream means the reader never blocks and a dropped byte costs
// one sample rather than desynchronising the link.

#ifndef JOYSTICK_INPUT_H
#define JOYSTICK_INPUT_H

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <SDL2/SDL.h>

#ifdef DEVBOX_HAVE_GPIOD
extern "C" {
#include <gpiod.h>
}
#endif

// State of the five CX40 switches. 1 = pressed/closed.
struct ButtonState
{
    int up = 0;
    int down = 0;
    int left = 0;
    int right = 0;
    int fire = 0;
};

class InputSource
{
  public:
    virtual ~InputSource() = default;

    // Refresh `state` with the most recent reading. Must not block: the devbox
    // calls this once per 60 Hz emulator step and any stall here directly
    // inflates end-to-end latency.
    virtual void poll(ButtonState& state) = 0;

    virtual const char* name() const = 0;
};

// ---------------------------------------------------------------------------
// SerialInput — MCU streams switch state over USB CDC
// ---------------------------------------------------------------------------

class SerialInput : public InputSource
{
  public:
    SerialInput() = default;

    ~SerialInput() override
    {
        if (fd_ >= 0)
        {
            tcsetattr(fd_, TCSANOW, &original_);
            close(fd_);
        }
    }

    SerialInput(const SerialInput&) = delete;
    SerialInput& operator=(const SerialInput&) = delete;

    bool open_port(const std::string& path, int baud, bool announce = true)
    {
        fd_ = open(path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            if (announce)
                std::fprintf(stderr, "[joystick] failed to open %s: %s\n", path.c_str(), std::strerror(errno));
            return false;
        }

        if (tcgetattr(fd_, &original_) != 0)
        {
            std::fprintf(stderr, "[joystick] tcgetattr failed on %s: %s\n", path.c_str(), std::strerror(errno));
            close(fd_);
            fd_ = -1;
            return false;
        }

        struct termios opt = original_;
        cfmakeraw(&opt);
        opt.c_cflag |= (CLOCAL | CREAD);
        opt.c_cflag &= ~CRTSCTS;
        // Fully non-blocking: return whatever is buffered, never wait.
        opt.c_cc[VMIN] = 0;
        opt.c_cc[VTIME] = 0;

        const speed_t speed = baud_constant(baud);
        cfsetispeed(&opt, speed);
        cfsetospeed(&opt, speed);

        if (tcsetattr(fd_, TCSANOW, &opt) != 0)
        {
            std::fprintf(stderr, "[joystick] tcsetattr failed on %s: %s\n", path.c_str(), std::strerror(errno));
            close(fd_);
            fd_ = -1;
            return false;
        }

        tcflush(fd_, TCIFLUSH);
        path_ = path;
        baud_ = baud;
        std::printf("[joystick] serial input on %s @ %d baud\n", path.c_str(), baud);
        return true;
    }

    void poll(ButtonState& state) override
    {
        if (fd_ < 0)
        {
            // Retry about once a second. Reopening by the ORIGINAL path matters:
            // a /dev/serial/by-id/ symlink follows the device across a
            // re-enumeration, where a bare /dev/ttyUSBn does not.
            if (++reopen_countdown_ < 60)
            {
                state = ButtonState{};
                return;
            }
            reopen_countdown_ = 0;
            // Retry quietly. A device unplugged for an hour would otherwise
            // write 3600 identical error lines into the run log and bury
            // everything worth reading; say it once, then hourly.
            const bool announce = (reopen_failures_ == 0) ||
                                  (reopen_failures_ % 3600 == 0);
            if (path_.empty() || !open_port(path_, baud_, announce))
            {
                ++reopen_failures_;
                state = ButtonState{};
                return;
            }
            reopen_failures_ = 0;
            std::printf("[joystick] reconnected to %s\n", path_.c_str());
            last_data_time_ = std::chrono::steady_clock::now();
            std::fflush(stdout);
            have_seen_data_ = false;
        }

        // Drain everything buffered and keep only the newest framed byte. The
        // MCU free-runs faster than we poll, so the buffer normally holds
        // several samples and all but the last are already stale.
        unsigned char buf[256];
        int newest = -1;
        ssize_t n;
        while ((n = read(fd_, buf, sizeof(buf))) > 0)
        {
            for (ssize_t i = n - 1; i >= 0; --i)
            {
                if (buf[i] & 0x80)
                {
                    newest = buf[i];
                    break;
                }
            }
            if (n < static_cast<ssize_t>(sizeof(buf)))
            {
                break; // drained
            }
        }

        // Detect a vanished device by SILENCE, not by an error code. With
        // VMIN=0/VTIME=0 a non-blocking tty returns 0 both for "no data right
        // now" and for a device that has been unplugged, and an fd left over
        // from a re-enumeration (ttyUSB0 -> ttyUSB1) keeps returning 0 forever
        // while /proc shows it as "(deleted)". Checking errno therefore never
        // fires. The bridge free-runs at 1 kHz, so a second of total silence
        // means the link is gone, not that nothing is pressed.
        const auto now = std::chrono::steady_clock::now();
        if (newest >= 0)
        {
            last_data_time_ = now;
        }
        else if (have_seen_data_ &&
                 now - last_data_time_ > std::chrono::milliseconds(1000))
        {
            std::fprintf(stderr, "[joystick] no data from %s for 1s -- releasing "
                                 "all buttons and reopening\n", path_.c_str());
            close(fd_);
            fd_ = -1;
            last_ = 0x80;          // marker set, every switch released
            have_seen_data_ = false;
            state = ButtonState{};
            return;
        }

        if (newest >= 0)
        {
            last_ = static_cast<unsigned char>(newest);
            have_seen_data_ = true;
        }
        else if (!have_seen_data_)
        {
            // Nothing has ever arrived — leave everything unpressed rather than
            // feeding the emulator a stuck input.
            state = ButtonState{};
            return;
        }

        state.up = (last_ >> 0) & 1;
        state.down = (last_ >> 1) & 1;
        state.left = (last_ >> 2) & 1;
        state.right = (last_ >> 3) & 1;
        state.fire = (last_ >> 4) & 1;
    }

    const char* name() const override { return "serial"; }

  private:
    int baud_ = 115200;
    int reopen_countdown_ = 0;
    long reopen_failures_ = 0;
    std::chrono::steady_clock::time_point last_data_time_ = std::chrono::steady_clock::now();

    static speed_t baud_constant(int baud)
    {
        switch (baud)
        {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            std::fprintf(stderr, "[joystick] unsupported baud %d, using 115200\n", baud);
            return B115200;
        }
    }

    int fd_ = -1;
    std::string path_;
    struct termios original_ {};  // restored on close
    unsigned char last_ = 0x80;   // nothing pressed
    bool have_seen_data_ = false;
};

// ---------------------------------------------------------------------------
// KeyboardInput — no robot hardware
// ---------------------------------------------------------------------------

class KeyboardInput : public InputSource
{
  public:
    void poll(ButtonState& state) override
    {
        const Uint8* keys = SDL_GetKeyboardState(nullptr);
        state.up = keys[SDL_SCANCODE_UP];
        state.down = keys[SDL_SCANCODE_DOWN];
        state.left = keys[SDL_SCANCODE_LEFT];
        state.right = keys[SDL_SCANCODE_RIGHT];
        state.fire = keys[SDL_SCANCODE_SPACE];
    }

    const char* name() const override { return "keyboard"; }
};

// ---------------------------------------------------------------------------
// GpioInput — original Raspberry Pi path (libgpiod >= 2.0)
// ---------------------------------------------------------------------------

#ifdef DEVBOX_HAVE_GPIOD
class GpioInput : public InputSource
{
  public:
    // BCM pin assignment from the paper: Up 17, Down 27, Left 22, Right 24,
    // Fire 23. Internal pull-ups are enabled, so a closed switch pulls the line
    // to ground and reads as logic 0 — hence the inversion in poll().
    static constexpr unsigned int kUp = 17;
    static constexpr unsigned int kDown = 27;
    static constexpr unsigned int kLeft = 22;
    static constexpr unsigned int kRight = 24;
    static constexpr unsigned int kFire = 23;

    ~GpioInput() override
    {
        if (request_ != nullptr)
        {
            gpiod_line_request_release(request_);
        }
    }

    bool open_chip(const char* chip_path)
    {
        static const unsigned int offsets[] = {kUp, kDown, kLeft, kRight, kFire};

        gpiod_chip* chip = gpiod_chip_open(chip_path);
        if (!chip)
        {
            std::fprintf(stderr, "[joystick] gpiod_chip_open(%s) failed: %s\n", chip_path, std::strerror(errno));
            return false;
        }

        gpiod_line_settings* settings = gpiod_line_settings_new();
        if (!settings)
        {
            gpiod_chip_close(chip);
            return false;
        }
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

        gpiod_line_config* line_cfg = gpiod_line_config_new();
        if (!line_cfg)
        {
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip);
            return false;
        }
        for (unsigned int offset : offsets)
        {
            if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) < 0)
            {
                gpiod_line_config_free(line_cfg);
                gpiod_line_settings_free(settings);
                gpiod_chip_close(chip);
                return false;
            }
        }

        gpiod_request_config* req_cfg = gpiod_request_config_new();
        if (req_cfg)
        {
            gpiod_request_config_set_consumer(req_cfg, "PhysicalAtariEnvironment");
        }

        request_ = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

        gpiod_request_config_free(req_cfg);
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);

        if (!request_)
        {
            std::fprintf(stderr, "[joystick] gpiod_chip_request_lines failed: %s\n", std::strerror(errno));
            return false;
        }
        std::printf("[joystick] GPIO input on %s\n", chip_path);
        return true;
    }

    void poll(ButtonState& state) override
    {
        if (!request_)
        {
            return;
        }
        state.up = pressed(kUp);
        state.down = pressed(kDown);
        state.left = pressed(kLeft);
        state.right = pressed(kRight);
        state.fire = pressed(kFire);
    }

    const char* name() const override { return "gpio"; }

  private:
    int pressed(unsigned int offset) const
    {
        return gpiod_line_request_get_value(request_, offset) == GPIOD_LINE_VALUE_ACTIVE ? 0 : 1;
    }

    gpiod_line_request* request_ = nullptr;
};
#endif // DEVBOX_HAVE_GPIOD

#endif // JOYSTICK_INPUT_H
