# Robotroller calibration — this robot

Measured on hardware **2026-08-05**. Every value here was obtained by driving the
servo and confirming the joystick actually clicked; none are from the paper.

The live copy of these numbers is
`agent_code/physical_atari_configs/physical_atari_sample_config.json`, which is
**gitignored** (per-robot calibration should not be committed). This file is the
recoverable record — if that config is lost, rebuild it from here.

---

## Results

| Servo | Role | Neutral | Deflections | EEPROM angle limits |
|---|---|---|---|---|
| **50** | fire | 2188 | pressed **2588** (+400) | 951–3145 |
| **51** | left / right | **2564** | left **2324** / right **2804** (±240) | **2244–2884** |
| **52** | up / down | 2142 | down **1902** / up **2382** (±240) | 1068–3028 |

Other config values: `overcurrent_counts` **50**, `baud_rate` 1000000,
`serial_port` `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110727-if00`.

Sign conventions, verified by watching the stick:

- servo 51 — **lower encoder = left**
- servo 52 — **lower encoder = down**
- servo 50 — the button presses toward **higher** encoder values

Switch actuation was bracketed to **180–210 ticks** (180 did not click; 210 and
240 did). 240 is used for margin.

Measured currents in normal operation are very low: **2–5 counts** on the stick
axes, **6–10** while holding fire (~6.5 mA per count). `overcurrent_counts` is
therefore 50 (~325 mA) — about 5× the worst observed, and far below the paper's
185, which nothing on this robot would ever reach. The fire press settles with a
steady ~13-tick position error at 6–10 counts; that is the button spring holding
against the servo, not a stall.

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

## Not yet done

- **The throw costs ~100 ms of a ~165 ms end-to-end budget.** 240 ticks at
  `goal_speed` 2400 is roughly double the paper's ~82–132 tick throw (~55 ms).
  Fix this **mechanically** — a longer servo horn arm needs less servo rotation
  per unit of stick travel — not by shrinking the throw below the 180–210 tick
  actuation point.
- **`torque_limit` is 500 in the config, but calibration ran at 300.** The fire
  press holds against a stop; at 500 it pushes harder. Consider 300.
- **PID is still Feetech factory 32/0/0**, giving 6–13 ticks of steady-state lag.
  Acceptable — the switch actuates well before that matters — but untuned.
- **Deflections were calibrated with a human watching for clicks.** Once the
  ESP32 bridge is wired, redo this against real switch closures, which also gives
  a principled way to trim the throw and recover latency.
