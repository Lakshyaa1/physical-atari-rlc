# Robotroller calibration — this robot

Measured on hardware **2026-08-05**. Every value here was obtained by driving the
servo and confirming the joystick actually clicked; none are from the paper.

The live copy of these numbers is
`agent_code/physical_atari_configs/physical_atari_sample_config.json`, which is
**gitignored** (per-robot calibration should not be committed). This file is the
recoverable record — if that config is lost, rebuild it from here.

**The servo bus lives on the agent machine (the RTX 4090), as of 2026-08-06.**
Every tool below must be run *there*, not on the devbox laptop. The
`/dev/serial/by-id/…5B14110727-if00` path is derived from the adapter's own
serial number, so it is identical on either machine — moving the USB cable
needed no config change, and all 18 actions re-verified unchanged from the 4090.
Because the config is gitignored, `scp` it whenever it changes; the two copies
otherwise diverge silently.

---

## Results

| Servo | Role | Neutral | Deflections | EEPROM angle limits |
|---|---|---|---|---|
| **50** | fire | 2188 | pressed **2588** (+400) | 951–3145 |
| **51** | left / right | **2564** | left **2064** ⚠ / right **2804** | **2020–2884** ⚠ |
| **52** | up / down | 2142 | down **1902** / up **2382** (±240) | 1068–3028 |

⚠ **Servo 51's left value and min limit changed 2026-08-07** (from left 2324,
min 2244) to work around a mechanical fault — see "Left throw regression" below.
**Run 001 used the original left 2324**, which worked correctly at the time.
`RUN_001_PONG.md` is the authoritative record of what that run actually used.

Other config values: `overcurrent_counts` **140** (see below — 50 was wrong),
`baud_rate` 1000000,
`serial_port` `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110727-if00`.

Sign conventions, verified by watching the stick:

- servo 51 — **lower encoder = left**
- servo 52 — **lower encoder = down**
- servo 50 — the button presses toward **higher** encoder values

Switch actuation was bracketed to **180–210 ticks** (180 did not click; 210 and
240 did). 240 is used for margin.

**Static holding** current is very low: **2–5 counts** on the stick axes, **6–10**
while holding fire (~6.5 mA per count). The fire press settles with a steady
~13-tick position error at 6–10 counts; that is the button spring holding against
the servo, not a stall.

⚠ Those holding figures are **not** what `overcurrent_counts` should be set from —
the reflex sees the *peak during the move*, which is ~10× higher. See
"overcurrent_counts was wrong" below.

---

## The three things that make this robot different

### 1. A mounted servo has no self-establishing neutral

The joystick's centring spring **cannot back-drive the STS3215 gearbox**. Once
coupled, a servo simply stays wherever it was last left.

So *present position is not a neutral reference*. Any tool that reads present
position and calls it "neutral" will re-baseline onto the previous run's drift —
during this session that silently accumulated ~100 ticks before it was caught.
**Always pass an absolute neutral.** `calibrate_deflection.py` takes `--neutral`
for exactly this reason.

This is also why servo 51's neutral is 2564 and not the 2466 measured earlier:
2466 was taken while the servos were uncoupled and free to spring back.

Corollary: the neutral is defined by **where the joystick looks centred**, which
has to be established by eye, not by letting the servo rest.

### 2. Angle limits are centred on whatever the previous build's neutral was

Servo 51 shipped with limits 1844–2700, centred on 2272 — but its true neutral is
2564. That left 720 ticks of travel one way and only **136** the other, which is
not enough to reach the switch.

The firmware **silently clamps every `goal_position` write** to this range,
regardless of torque. There is no error, no status flag: the servo just stops
short. They were rewritten to 2244–2884, centred on the real neutral.

### 2b. The fire servo does not hold position with torque off

The stick axes stay exactly where they are left when torque is cut, but servo 50
consistently springs back from its released position 2188 to **~1935**, about 250
ticks, every time — observed on separate runs on both machines.

So the fire linkage's true mechanical rest is ~1935, and the calibrated
"released" 2188 is already 250 ticks into the press direction, taking up slack.
That is still far short of the switch, which actuates 180–210 ticks *above* 2188,
so the button is genuinely not pressed at rest.

Two consequences: the servo holds against the button spring continuously while
torque is on (measured at only 6–10 current counts, so this is cheap), and on
every torque-on it must travel ~250 ticks before it is at "released". The tools
handle that by energising at the *current* position and then travelling to
neutral, which is why they must never take present position as the neutral.

### 3. EEPROM write acknowledgements cannot be trusted

Both the ID change and the angle-limit write returned a *failure* status while
the value landed correctly. The status packet is frequently lost. **Always verify
by reading the register back** — `set_angle_limits.py` and `change_id.py` do.

---

## Tools

All are dry-run by default, refuse to run with torque already enabled, and
disable torque on every exit path including `SIGTERM` (so `timeout` cannot leave
a servo energised).

| Script | Purpose |
|---|---|
| `setup_scripts/identify_servos.py` | Which servo ID is which joint. Read-only; watches `present_position` while each is moved by hand. **Only works before the servos are coupled.** |
| `setup_scripts/set_angle_limits.py` | Rewrite EEPROM min/max angle limits (addr 9/11), with unlock → write → relock → verify. |
| `setup_scripts/calibrate_deflection.py` | Hold a servo at a series of deflections so a human can see which one actuates the switch. Takes `--neutral`. |
| `setup_scripts/creep_calibrate.py` | Walk outward from neutral in small steps, stopping on current, position error, temperature, or a hard excursion cap. Use when the mechanical limit is unknown. |
| `setup_scripts/verify_actions_hw.py` | Drive the real joystick through all 18 ALE actions and check each pose against this table. |
| `setup_scripts/measure_throw_time.py` | Time the throw at several goal speeds; the latency work that needs no camera. |
| `setup_scripts/training_status.py` | Steps, measured rate, ETA and reward split of a running (or finished) training job. |
| `devbox_code/tools/monitor_joystick.py` | Watch the ESP32 bridge live on the terminal; prints one line per state change plus a frame-rate heartbeat. Devbox must be stopped. |
| `setup_scripts/camera_stream.py` | Serve the camera as MJPEG over HTTP with an AprilTag overlay and per-tag hit rate — this is how the camera gets aimed. |
| `input_output_cpp_library/tools/verify_actions.py` | The same 18-action check against an emulated bus, exercising the real compiled driver. Shares its action table with `verify_actions_hw.py`. |

### Writing a motion command by hand

Anything driving the servos directly must write the profile as **one block at
addr 41**, exactly as `SyncWritePosEx` does:

    [acc, pos_L, pos_H, time_L, time_H, speed_L, speed_H]

Writing the position register alone leaves the neighbouring `goal_time` and
`goal_speed` cells holding whatever was there before, which produces wildly
inconsistent motion — a 240-tick throw measured anywhere from 184 ms to 1064 ms
across runs with identical settings, until the full block was written.

### Procedure that produced the table above

```bash
SERIAL=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110727-if00

# 0. Establish the true neutral: centre the stick by eye, torque off, read position.

# 1. Bracket the actuation point. Two deflections per run, watch for the click.
python3 setup_scripts/calibrate_deflection.py --servo 51 --direction -1 \
        --neutral 2564 --levels 180,240 --hold 5

# 2. If a direction cannot reach the switch, the angle limits are the cause.
python3 setup_scripts/set_angle_limits.py --servo 51 --min 2244 --max 2884   # dry run
python3 setup_scripts/set_angle_limits.py --servo 51 --min 2244 --max 2884 --execute

# 3. Confirm repeatability before believing a number.
python3 setup_scripts/calibrate_deflection.py --servo 52 --direction 1 \
        --neutral 2142 --levels 240,240,240,240 --hold 4
```

Repeating the same deflection four times is worth the extra minute: it separates
a real mechanical limit (identical position every time) from noise, and it is far
easier for a human to observe than a single transient.

---

## Measured throw timing (2026-08-06)

`setup_scripts/measure_throw_time.py` on the real servos. Reproducible to ±1 ms
once the profile is written the way the driver writes it (see below).

**The throw is torque-limited, not speed-limited.** `goal_speed` 1200, 2400 and
3400 all produce exactly the same times, because the servo never reaches even
1200 steps/s against the joystick spring. `torque_limit` is the lever:

| `torque_limit` | 240-tick throw | effective speed |
|---|---|---|
| 300 | 280 ms | ~1070 steps/s |
| **500** (config) | **205 ms** | ~1590 steps/s |
| 1000 (max) | 184 ms | ~1830 steps/s |

This corrects the earlier estimate of ~100 ms, which came from dividing 240
ticks by `goal_speed` 2400 — a speed the servo does not actually reach.

P gain trades settling against overshoot, at `torque_limit` 500:

| P | dead time | throw | overshoot | settled error |
|---|---|---|---|---|
| **32** (factory, current) | 55 ms | 205 ms | 8.8 | 4.0 |
| 64 | 45 ms | 190 ms | 15.2 | 3.0 |
| 128 | 38 ms | 180 ms | 17.6 | 1.0 |

P was restored to 32 after measuring; nothing was left changed. Note the ~53 ms
of **dead time before any motion at all**, which P reduces but does not remove.

Even the best case (184 ms) exceeds the paper's entire ~165 ms end-to-end
budget, so this cannot be tuned away — see below.

## Left throw regression (2026-08-07) — diagnosis method worth reusing

Servo 51's left throw stopped reaching the switch. The method that found it
generalises: **map every servo position against the switch states** rather than
sweeping deflections, because a deflection sweep assumes the neutral is still
correct and silently misleads if it is not.

- **RIGHT closes at 2724** — 160 ticks from neutral, clean wide window
- **LEFT never closes above 2060** — 500 ticks from neutral

The servo reaches the joystick's mechanical left stop near **2130**, and even
pressed against it the contact is only ~58% reliable. Deeper only stalls the
servo: position error grows 31 → 179 ticks with no gain in contact.

Ruled out by direct measurement, in order:

| suspected | test | verdict |
|---|---|---|
| ESP32 / GPIO / wiring / switch | hand press; bridge streaming | fine — hand press works, 1000 frames/s clean |
| servo horn slipping | current draw | fine — left 448 mA peak vs right 390 mA |
| not travelling far enough | deeper sweep | **ruled out** — deeper is *worse*: 320→22.4%, 360→9.9%, 400→1.3%, 440→0.0% |

Degradation across the day: **1876 → 959 → 181 → 0** frames of contact, which
points at something loosening rather than a sudden break. Needs mechanical
attention; the config change is a workaround.

**A single symptom that hits all five switches at once is the common return**
(DB9 pin 8 → GND) — that happened earlier the same day and presented as every
switch failing while the bridge streamed perfectly.

## Camera — Lenovo FHD Webcam (2026-08-06)

USB `17ef:4831`, `/dev/video0` on the **agent machine**. `sra` had to be added to
the `video` group; the udev ACL only grants the desktop-session user.

**It cannot do uncompressed 720p at a usable rate.** Measured:

| format | measured fps |
|---|---|
| YUYV 1280×720 | **10.0** |
| YUYV 640×480 | 24.8 |
| MJPG 1280×720 | 24.8 |
| MJPG 1920×1080 | 24.8 |

`camera.cpp` used to force YUYV unconditionally, which at the configured
1280×720 would have run the whole agent at 10 fps. The pixel format is now a
config field (`camera.fourcc`, default `YUYV` so nothing else changes), and this
robot uses **MJPG 1280×720** — keeping resolution on the AprilTags matters more
here than avoiding JPEG artefacts. The paper used a Razer Kiyo Pro, which does
uncompressed at rate; this webcam does not.

Three further corrections, all measured rather than assumed:

- **`focus` and `zoom` do nothing.** This camera is fixed-focus and exposes
  neither control — OpenCV reports `-1` for both. `camera_focus_sweep.py` is
  useless on this hardware.
- **Manual exposure was never being applied.** `CAP_PROP_AUTO_EXPOSURE 0.25` is
  the *old* OpenCV encoding; OpenCV 5's V4L2 backend passes the enum through, so
  0.25 was silently ignored, the camera stayed on aperture-priority auto, and
  every subsequent exposure write failed. Now set to `1` with a readback and a
  fallback. This matters because auto-exposure hunts as the game screen changes,
  which moves the goalposts for the tag detector.
- **`brightness`/`contrast` were out of range.** 128/128 against actual ranges of
  −64..64 and 0..64; they simply clamped. Set to this camera's own defaults.

`exposure` was 20 (from the paper's camera), which is **black** on this sensor —
mean pixel 6/255. Set to 250, which works against the aimed screen.

### Tag detection vs. camera framing

`quad_decimate` is really a proxy for how large the tags are in frame, and this
rig demonstrated both sides of it.

**First framing** — camera well back, screen filling about half the frame, tag
edges **44–58 camera px**. `quad_decimate=2.0` halves that to ~22–29 px, under
3 px per tag cell for tag36h11. Measured over 40 live frames:

| tag | decimate 2.0 | decimate 1.5 |
|---|---|---|
| 0 (corner) | 98% | 100% |
| **1 (corner)** | **12%** | 100% |
| 2 (corner) | 100% | 100% |
| 3 (corner) | 95% | 100% |
| 11 (reward) | 80% | 100% |
| 17 (reward) | 100% | 100% |

A corner tag found in 12% of frames breaks the homography, which needs all four.
`quad_decimate` is now a config field (`camera.apriltag_quad_decimate`, default
2.0), and 1.5 was used as a stopgap.

**Second framing (current)** — camera re-aimed so the screen fills the frame,
tag edges **79–98 px**. At `quad_decimate=2.0` all four corners and the reward
tag read **100%**, so the config is back to the upstream default. Framing, not
the parameter, was the real fix.

Reading the change indicator's rate needs care: it shows as ids 15/16/17 at
roughly 32/32/35%, which sums to ~99%. That is the **period-3 cycle, one id at a
time**, not a detection failure.

The detector itself was never wrong: on the devbox's own `--dump-frame` render,
tag36h11 at 2.0 finds all six tags. Always check the pristine render before
suspecting the pipeline.

**Nothing may overlap the screen.** A terminal window straying over a corner
occludes that tag and kills the homography just as effectively as bad framing —
observed live, with corner tag 2 covered, failing identically at 1.5 and 2.0.

Give the game a monitor of its own. `PhysicalALE` now takes **`--display=N`**:
plain `SDL_WINDOWPOS_CENTERED` always resolves to display 0, and
`SDL_VIDEO_FULLSCREEN_DISPLAY` is an SDL **1.2** variable that SDL2 silently
ignores, so on a two-monitor desk the game lands on the wrong screen with no
message. Run the game on the monitor the camera watches and keep terminals on
the other one:

```bash
./PhysicalALE ../games ms_pacman --input=keyboard --fps=60 --display=1
```

Mirroring the displays does not work: the game and your terminals then share a
screen, and any window that strays over a corner tag breaks the run.

### overcurrent_counts was wrong — 50 → 140

The 2–10 counts recorded at calibration were **static holding current**, read
after the servo had settled. The reflex sees the **peak during the move**, which
is an order of magnitude higher. Measured at `torque_limit` 500:

| move | peak counts | peak mA |
|---|---|---|
| L/R left | 69 | 448 |
| L/R right | 62 | 403 |
| U/D up | 68 | 442 |
| U/D down | 62 | 403 |
| fire press | 59 | 384 |

At a threshold of 50 the reflex fired on nearly every action, cycling torque
mid-move — visible immediately in the first end-to-end run. Now **140** (~910 mA):
2× the worst observed peak, still well under a stall, and below the paper's 185.

### End-to-end verified 2026-08-06

Final configuration: camera aimed so the screen fills the frame, game alone on
its own monitor via `--display=1`, `quad_decimate` at the stock 2.0.

**All six tags detect in 100% of 50 consecutive frames**, tag edges 79–98 px.

Full `PhysicalAtariEnv` on the agent machine — camera → AprilTags → homography →
observation, with the servos driven — produces observations at the correct
`(210, 160, 3)` with real content and no transformation-matrix warnings, and the
overcurrent reflex stays silent for a whole run. The **reward channel decodes**:
the change indicator cycles 15 → 16 → 17 → 15 and each transition yields +1,
confirming the period-3 behaviour.

### The loop is CLOSED (2026-08-06)

The ESP32 bridge is built, flashed and verified, so the full physical path now
works: agent → servos → CX40 → ESP32 → devbox → screen → camera → agent.

- The bridge streams **1000 frames/s, 100% with bit 7 set**, idle `0x80`.
- Driving the servos produced exactly the right switch states, including
  **`0x89` = up+right**, a real diagonal.
- With `--input=serial`, a random-action agent scored **280** against the **60**
  baseline seen with no input connected, and the AprilTag reward channel
  returned **14 rewards over 60 moves**.

Firmware and the full write-up:
`devbox_code/esp32_joystick_bridge/esp_idf/` (ESP-IDF v6.0, ESP32-D0WD-V3).

Run it with the stable by-id path, never `/dev/ttyUSB0`:

```bash
./PhysicalALE ../games ms_pacman --display=1 --fps=60 \
  --input=serial:/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

## Not yet done

- **The throw is ~205 ms at the config's settings**, against a ~165 ms
  end-to-end budget. Tuning buys at most ~25 ms. The real fix is **mechanical**
  — a longer servo horn arm needs less servo rotation per unit of stick travel —
  not shrinking the throw below the 180–210 tick actuation point.
- **`torque_limit` 500 is confirmed the right value**, resolving the earlier
  open question. Calibration ran at 300, which is 75 ms slower; 1000 buys only
  another 21 ms and pushes harder against the button stop.
- **PID is still Feetech factory 32/0/0.** P=128 is measurably better on every
  axis except overshoot (which doubles). Left at 32 pending a decision, since
  the paper traced controller wear to an over-aggressive profile.
- **Deflections were calibrated with a human watching for clicks.** Once the
  ESP32 bridge is wired, redo this against real switch closures, which also gives
  a principled way to trim the throw and recover latency.
