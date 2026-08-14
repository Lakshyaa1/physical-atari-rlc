# ESP-IDF joystick bridge

ESP-IDF port of `../esp32_joystick_bridge.ino`. Same wire protocol; pick whichever
toolchain you have. Built and flashed on hardware 2026-08-06 against an
**ESP32-D0WD-V3** with **ESP-IDF v6.0**.

## Wiring

The CX40 is five normally-open switches sharing a common return. Internal
pull-ups hold each line high, so a closed switch reads LOW.

| DB9 pin | Function | GPIO |
|---|---|---|
| 1 | Up | 14 |
| 2 | Down | 13 |
| 3 | Left | 25 |
| 4 | Right | 26 |
| 6 | Fire | 27 |
| 8 | Common | GND |

These avoid the strapping pins (0/2/5/12/15), the SPI flash pins (6–11, which
break the boot), and the input-only pins (34–39, which have no internal pull-up).
GPIO 13/14 are JTAG MTCK/MTMS, free unless you are debugging over JTAG.

## Build and flash

`$IDF_PATH/export.sh` does **not** work on this machine — it looks for the venv
under `~/.espressif/python_env/`, but this install keeps it under
`~/.espressif/tools/python/`. Use the activate script, and add `$IDF_PATH/tools`
to PATH yourself (it holds `idf.py` and the activate script does not add it):

```bash
. ~/.espressif/tools/activate_idf_v6.0.sh
export PATH="$IDF_PATH/tools:$PATH"

idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

## Why a "print W/A/S/D on press" sketch cannot work

Worth stating plainly, because such a sketch looks perfect in a serial monitor
and still leaves the emulator completely dead:

1. **The devbox accepts a byte only if bit 7 is set** (`if (buf[i] & 0x80)` in
   `SerialInput::poll`). ASCII is all < 0x80, so every byte is dropped, the
   devbox concludes it has never seen data, and it holds the emulator at
   "nothing pressed" — silently, with no error.
2. **Printing only while a button is down carries no release.** Measured on the
   ASCII build: 0 bytes in 4 seconds while idle. Here every sample carries the
   full five-switch state, so release is implicit and nothing can stick on.
3. **One byte holds all five switches**, so diagonals and the `*FIRE`
   combinations survive. 12 of the agent's 18 actions are combinations.

## The FreeRTOS tick trap

`pdMS_TO_TICKS(1)` is **0** at the default `CONFIG_FREERTOS_HZ=100`. The sample
loop then runs at one tick — 10 ms — so the bridge silently samples at 100 Hz
instead of 1 kHz, and the 3-sample debounce stretches from 3 ms to 30 ms, long
enough to swallow a quick tap. Measured before the fix: exactly 100 frames/s.
`sdkconfig.defaults` sets `CONFIG_FREERTOS_HZ=1000`; measured after: exactly
1000 frames/s.

## Verified on hardware

- **1000 frames/s, 100% with bit 7 set**, idle byte `0x80`.
- Driving the servos from the agent machine produced exactly the right states:
  `0x81` up, `0x82` down, `0x84` left, `0x88` right, `0x90` fire, and **`0x89`
  up+right** — a real diagonal, which the ASCII build could not express.
- With the devbox on `--input=serial`, a random-action agent scored **280**
  against a **60** baseline with no input connected, and the AprilTag reward
  channel returned **14 rewards over 60 moves**.
