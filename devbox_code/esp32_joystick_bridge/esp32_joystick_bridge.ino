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

// esp32_joystick_bridge.ino
//
// Reads the five switch lines of an Atari CX40 controller and streams their
// state to the devbox over USB serial.
//
// WHY THIS EXISTS
// ---------------
// The original Physical Atari devbox is a Raspberry Pi, which reads the CX40's
// switches directly on its GPIO header. On a devbox without usable GPIO -- an
// x86 laptop, say -- that is impossible, so this MCU stands in for the header.
// It performs no logic: it samples five pins and reports them. The agent still
// closes its loop through physical servo motion, exactly as before.
//
// WIRING
// ------
// The CX40 is five normally-open switches sharing a common return. Closing a
// switch connects its line to common.
//
//     DB9 pin 1  Up          DB9 pin 3  Left      DB9 pin 6  Fire
//     DB9 pin 2  Down        DB9 pin 4  Right     DB9 pin 8  Common / GND
//
// Wire DB9 pin 8 to an ESP32 GND, and each signal pin to the GPIO named below.
// Internal pull-ups hold each line high when open, so a closed switch reads LOW
// -- hence the inversion in readButtons(). No external resistors are needed.
//
// >>> SET THESE FIVE TO MATCH YOUR WIRING <<<
// Defaults avoid the ESP32's strapping pins (0/2/5/12/15), the flash pins
// (6-11), and the input-only pins (34-39, which have no internal pull-up).
static const int PIN_UP = 32;
static const int PIN_DOWN = 33;
static const int PIN_LEFT = 25;
static const int PIN_RIGHT = 26;
static const int PIN_FIRE = 27;

// WIRE PROTOCOL
// -------------
// One byte per sample:
//
//     bit 7 : always 1 (frame marker)
//     bit 4 : fire     bit 3 : right    bit 2 : left
//     bit 1 : down     bit 0 : up
//     (1 = pressed)
//
// The payload uses only bits 0-4, so it can never set bit 7. Any byte with the
// high bit set is therefore a complete, valid frame, and a reader that loses
// sync recovers on the very next byte -- no framing state machine, nothing to
// get stuck in.
//
// We transmit continuously rather than only on change. The devbox drains the
// buffer and keeps the newest frame, so a free-running stream means it never
// blocks and a dropped byte costs one sample instead of desynchronising.

static const uint32_t BAUD = 115200;
static const uint32_t SAMPLE_HZ = 1000;
static const uint32_t SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_HZ;

// Require this many consecutive identical samples before a change is reported.
// At 1 kHz that is ~3 ms of debounce -- far longer than mechanical contact
// bounce, and negligible against the platform's ~165 ms end-to-end latency.
static const uint8_t DEBOUNCE_SAMPLES = 3;

static uint8_t stable_state = 0;   // last debounced payload
static uint8_t candidate_state = 0;
static uint8_t candidate_count = 0;
static bool debug_mode = false;

static uint8_t readButtons()
{
	// Pull-ups mean LOW == closed == pressed.
	uint8_t s = 0;
	if (digitalRead(PIN_UP) == LOW) s |= (1 << 0);
	if (digitalRead(PIN_DOWN) == LOW) s |= (1 << 1);
	if (digitalRead(PIN_LEFT) == LOW) s |= (1 << 2);
	if (digitalRead(PIN_RIGHT) == LOW) s |= (1 << 3);
	if (digitalRead(PIN_FIRE) == LOW) s |= (1 << 4);
	return s;
}

void setup()
{
	Serial.begin(BAUD);

	pinMode(PIN_UP, INPUT_PULLUP);
	pinMode(PIN_DOWN, INPUT_PULLUP);
	pinMode(PIN_LEFT, INPUT_PULLUP);
	pinMode(PIN_RIGHT, INPUT_PULLUP);
	pinMode(PIN_FIRE, INPUT_PULLUP);

	// Settle the pull-ups before the first sample.
	delay(50);
	stable_state = readButtons();
	candidate_state = stable_state;
}

void loop()
{
	static uint32_t next_sample_us = 0;

	// Host commands, for bring-up only. The devbox opens the port read-only
	// and never writes, so these cannot fire during a real run.
	//   'd' - toggle human-readable debug output
	//   '?' - print one human-readable status line
	while (Serial.available() > 0) {
		const int c = Serial.read();
		if (c == 'd') {
			debug_mode = !debug_mode;
			Serial.printf("\r\n[bridge] debug %s\r\n", debug_mode ? "ON" : "OFF");
		} else if (c == '?') {
			const uint8_t s = stable_state;
			Serial.printf("\r\n[bridge] up=%d down=%d left=%d right=%d fire=%d (0x%02X)\r\n",
			              (s >> 0) & 1, (s >> 1) & 1, (s >> 2) & 1, (s >> 3) & 1, (s >> 4) & 1,
			              0x80 | s);
		}
	}

	const uint32_t now = micros();
	if (static_cast<int32_t>(now - next_sample_us) < 0) {
		return;
	}
	next_sample_us = now + SAMPLE_INTERVAL_US;

	const uint8_t raw = readButtons();
	if (raw == candidate_state) {
		if (candidate_count < DEBOUNCE_SAMPLES) {
			candidate_count++;
			if (candidate_count >= DEBOUNCE_SAMPLES) {
				stable_state = candidate_state;
			}
		}
	} else {
		candidate_state = raw;
		candidate_count = 0;
	}

	if (debug_mode) {
		static uint8_t last_printed = 0xFF;
		if (stable_state != last_printed) {
			last_printed = stable_state;
			const uint8_t s = stable_state;
			Serial.printf("up=%d down=%d left=%d right=%d fire=%d\r\n", (s >> 0) & 1,
			              (s >> 1) & 1, (s >> 2) & 1, (s >> 3) & 1, (s >> 4) & 1);
		}
		return;
	}

	Serial.write(static_cast<uint8_t>(0x80 | stable_state));
}
