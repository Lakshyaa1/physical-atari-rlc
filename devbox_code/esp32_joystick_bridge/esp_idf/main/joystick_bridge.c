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

// joystick_bridge.c - ESP-IDF port of esp32_joystick_bridge.ino
//
// Reads the five switch lines of an Atari CX40 controller and streams their
// state to the devbox over USB serial. Identical wire protocol to the Arduino
// sketch; pick whichever toolchain you already have.
//
// WIRING (matches the pins below)
// -------------------------------
// The CX40 is five normally-open switches sharing a common return. Closing a
// switch connects its line to common.
//
//     DB9 pin 1  Up     -> GPIO 14        DB9 pin 4  Right -> GPIO 26
//     DB9 pin 2  Down   -> GPIO 13        DB9 pin 6  Fire  -> GPIO 27
//     DB9 pin 3  Left   -> GPIO 25        DB9 pin 8  Common -> GND
//
// Internal pull-ups hold each line high when open, so a closed switch reads
// LOW -- hence the inversion below. No external resistors are needed.
//
// These pins avoid the strapping pins (0/2/5/12/15), the SPI flash pins
// (6-11, which brick the boot if driven), and the input-only pins (34-39,
// which have no internal pull-up for this to rely on). GPIO 13/14 are the
// JTAG MTCK/MTMS lines, which are free unless you are actively debugging over
// JTAG.

#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_UP GPIO_NUM_14
#define PIN_DOWN GPIO_NUM_13
#define PIN_LEFT GPIO_NUM_25
#define PIN_RIGHT GPIO_NUM_26
#define PIN_FIRE GPIO_NUM_27

// WIRE PROTOCOL
// -------------
// Must match SerialInput::poll() in devbox_code/include/joystick_input.h.
// One byte per sample:
//
//     bit 7 : always 1 (frame marker)
//     bit 4 : fire     bit 3 : right    bit 2 : left
//     bit 1 : down     bit 0 : up
//     (1 = pressed)
//
// The payload uses only bits 0-4, so it can never set bit 7. Any byte with the
// high bit set is therefore a complete, valid frame, and a reader that loses
// sync recovers on the very next byte -- no framing state machine.
//
// This is why a debug sketch that prints "W"/"A"/"S"/"D" on press cannot drive
// the devbox, even though it looks right in a serial monitor:
//
//   1. The devbox accepts a byte only if bit 7 is set. ASCII is all < 0x80, so
//      every byte is dropped, the devbox concludes it has never seen data, and
//      it holds the emulator at "nothing pressed" forever -- silently.
//   2. Printing only while a button is down carries no RELEASE. Here every
//      sample carries the full five-switch state, so letting go is implicit in
//      the next byte and no direction can stick on.
//   3. One byte holds all five switches, so diagonals (UPRIGHT) and the *FIRE
//      combinations survive. 12 of the agent's 18 actions are combinations;
//      separate per-key lines cannot express "both down in this instant".
#define BIT_UP 0
#define BIT_DOWN 1
#define BIT_LEFT 2
#define BIT_RIGHT 3
#define BIT_FIRE 4
#define FRAME_MARKER 0x80u

// Transmit continuously rather than on change. The devbox drains its buffer and
// keeps the newest framed byte, so a free-running stream means it never blocks
// waiting for an edge, and a dropped byte costs one sample instead of
// desynchronising the link.
//
// 1 kHz, not the 10 Hz a 100 ms delay would give: the whole perception-to-action
// budget is ~165 ms, and a 100 ms sampling period would spend most of it before
// the emulator even hears the press.
#define SAMPLE_HZ 1000

// Mechanical switches bounce for a few milliseconds on make and break. A bit
// must read the same for this many consecutive samples (~ms at 1 kHz) before
// the reported state follows it. Three samples is far below the ~100 ms the
// agent holds an action, so it costs nothing real, and it stops a bounce from
// being the one sample the devbox happens to latch. Set to 1 for raw levels.
//
// An asymmetric variant (press instantly, debounce only the release) was tried
// on 2026-08-07 to rescue a CX40 whose LEFT contact had gone intermittent. It
// made no difference -- the pin simply never read LOW at that deflection -- so
// this is back to the symmetric version the successful training run used.
#define DEBOUNCE_SAMPLES 3

static const gpio_num_t kPins[] = {PIN_UP, PIN_DOWN, PIN_LEFT, PIN_RIGHT, PIN_FIRE};
static const uint8_t kBits[] = {BIT_UP, BIT_DOWN, BIT_LEFT, BIT_RIGHT, BIT_FIRE};
#define NUM_PINS (sizeof(kPins) / sizeof(kPins[0]))

void app_main(void)
{
    for (size_t i = 0; i < NUM_PINS; i++)
    {
        gpio_reset_pin(kPins[i]);
        const gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << kPins[i],
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&cfg));
    }

    // Deliberately no ESP_LOGx anywhere in this file, and nothing else may
    // write to stdout: the stream is binary, and a log line in the middle of it
    // is indistinguishable from joystick data to a reader that resyncs on the
    // high bit. The boot log is harmless -- it is ASCII, so every byte of it
    // fails the bit-7 test and is discarded. To confirm the link, watch for the
    // devbox's own "[joystick] serial input on ..." line.

    bool stable[NUM_PINS] = {false};       // debounced, reported state
    uint8_t agree[NUM_PINS] = {0};         // consecutive samples disagreeing with `stable`

    const TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_HZ) ? pdMS_TO_TICKS(1000 / SAMPLE_HZ) : 1;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        uint8_t frame = FRAME_MARKER;

        for (size_t i = 0; i < NUM_PINS; i++)
        {
            // Pull-ups hold an open switch HIGH, so a closed (pressed) switch
            // reads 0 -- hence the inversion.
            const bool raw = (gpio_get_level(kPins[i]) == 0);

            if (raw == stable[i])
            {
                agree[i] = 0;
            }
            else if (++agree[i] >= DEBOUNCE_SAMPLES)
            {
                stable[i] = raw;
                agree[i] = 0;
            }

            if (stable[i])
            {
                frame |= (uint8_t)(1u << kBits[i]);
            }
        }

        // fwrite, not printf: this is one byte, not text. Frame bytes are always
        // >= 0x80, so they can never collide with '\n' and trip the console's
        // newline translation.
        fwrite(&frame, 1, 1, stdout);
        fflush(stdout);

        vTaskDelayUntil(&last_wake, period);
    }
}
