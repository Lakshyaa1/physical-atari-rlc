# Run 001 — Pong, 2026-08-06/07

The first completed training run on this robot. A real Atari 2600 emulator drives
a screen; a webcam watches it; an RL agent turns those pixels into servo commands
that physically move a CX40 joystick; the joystick's switches feed the emulator.
Nothing in the loop is simulated and there is **no network path** between agent
and emulator — the loop closes through light and mechanics.

**Result: the agent learned to win.** In evaluation it took 150 points to the
opponent's 87 — a 63.3% win rate — after 648,000 training steps over 12h 54m.

---

## 1. Result

| phase | points won | points lost | win rate |
|---|---|---|---|
| training (648,000 steps) | 1069 | 7749 | 12.12% |
| last 10% of training | — | — | 25.61% |
| **evaluation (50,000 steps, greedy)** | **150** | **87** | **63.29%** |

Net evaluation score **+63**. `eval_avg_reward_per_step` = **+0.00128**, against a
training lifetime average of **−0.0103**.

Evaluation scores far above training for two reasons: it runs pure greedy on the
final policy, whereas training carries ε-exploration and a policy still moving;
and the training average is dragged down by the early hours when the agent was
near-random.

### Learning curve

From `results.json` → `training_avg_reward_history` (one sample per 100 steps),
averaged by decile. Monotonic across the entire run:

| decile | steps | mean reward/step |
|---|---|---|
| 1 | 100 – 64,700 | −0.0237 |
| 2 | 64,800 – 129,400 | −0.0223 |
| 3 | 129,500 – 194,100 | −0.0211 |
| 4 | 194,200 – 258,800 | −0.0190 |
| 5 | 258,900 – 323,500 | −0.0168 |
| 6 | 323,600 – 388,200 | −0.0150 |
| 7 | 388,300 – 452,900 | −0.0135 |
| 8 | 453,000 – 517,600 | −0.0123 |
| 9 | 517,700 – 582,300 | −0.0114 |
| 10 | 582,400 – 647,900 | **−0.0107** |

A 55% reduction in loss rate, still improving at the end — **the run was stopped
by the step budget, not by convergence.**

Measured a second way, as share of points won, across six equal windows of the
reward log: **0.97% → 2.62% → 4.86% → 5.44% → 11.76% → 18.85%**. The last two
windows are the steepest, confirming it had not plateaued.

---

## 2. System

| role | hardware |
|---|---|
| Devbox (emulator + screen) | Laptop, Ubuntu 22.04, driving a 31.5" Acer monitor as a **dedicated display** |
| Agent (camera + servos + RL) | RTX 4090, 24 GB (`sra@omen`, LAN) |
| Camera | Lenovo FHD Webcam, USB `17ef:4831`, on the **4090** |
| Servos | 3 × Feetech STS3215 (7.4 V), Waveshare bus adapter, on the **4090** |
| Joystick readout | Atari CX40 → **ESP32** (ESP32-D0WD-V3) → USB CDC → **devbox** |

```
   ┌──────────── LAPTOP (devbox) ────────────┐
   │  PhysicalALE: game + AprilTags on screen│
   └────┬──────────────────────────▲─────────┘
        │ light                    │ USB CDC 115200, 5 switch bits
        ▼                          │
    ╔═══════╗                 ┌─────────┐
    ║ CAMERA║                 │  ESP32  │
    ╚═══╤═══╝                 └────▲────┘
        │ USB                      │ 5 wires + common GND
        ▼                          │
   ┌─── RTX 4090 (agent) ───┐  ┌────┴────┐
   │ camera → AprilTags →   │  │  CX40   │
   │ homography → policy →  │  │joystick │
   │ servo commands         │  └────▲────┘
   └──────────┬─────────────┘       │ servo horns
              └────► 3× STS3215 ────┘
```

**The CX40's switch output must go to the devbox, never the agent.** Routing it
back to the agent would mean the agent reads its own commands and the emulator
receives nothing — it looks broken while every component works.

---

## 3. Calibration (as used for Run 001)

Every value measured on this robot. The paper's Dynamixel numbers do not transfer.

| servo | role | neutral | deflections | EEPROM angle limits |
|---|---|---|---|---|
| 50 | fire | 2188 | pressed 2588 (+400) | 951–3145 |
| 51 | left/right | 2564 | left **2324** / right 2804 (±240) | 2244–2884 |
| 52 | up/down | 2142 | down 1902 / up 2382 (±240) | 1068–3028 |

Sign conventions: on 51 lower encoder = left; on 52 lower encoder = down; on 50
the button presses toward higher encoder values.

Other robot settings: `baud_rate` 1,000,000 · `P/I/D` 32/0/0 (Feetech factory) ·
`goal_speed` 2400 · `goal_acc` 0 · `torque_limit` 500 · `overcurrent_counts` 140.

Camera: MJPG **1280×720**, manual exposure **250**, brightness 0, contrast 32,
`apriltag_quad_decimate` **2.0**.

> Servo 51's left value was changed to **2064** on 2026-08-07 *after* this run,
> to work around a mechanical fault. Run 001 used **2324**. See §7.

---

## 4. Agent configuration

`agent_code/experiment_configs/agent_action_input_real.json`, agent
`agents.agent_action_input`.

| parameter | value | note |
|---|---|---|
| `steps` | 648,000 | + 50,000 eval |
| `ring_size` | 200,000 | 9.16 GiB. The default 400,000 needs 18.31 GiB and OOMs a 24 GB card |
| `action_set` | **[0, 3, 4]** | NOOP/RIGHT/LEFT — see §5 |
| `explore_log2` | −6 | ≈1.6% exploration |
| `obs_size` / `obs_channels` | 128 / 3 | |
| `base_channels` | 24 | |
| `lr_log2` / `lr_linear_log2` | −14 / −17 | |
| `train_batch` / `train_reps` | 16 / 1 | |
| `multisteps` / `discount` | 12 / 0.99 | |
| `policy_skip` / `train_skip` | 2 / 4 | |
| `target_model_log2` | 5 | |
| `max_frames_without_reward` | 18,000 | |

Checkpoint: `agent_code/checkpoints/pong__agents.agent_action_input__0__20260806-192731/`
— `weights.pkl` (28.3 MB), `agent_state.pkl`, `last_observation.pkl`, `results.json`.

---

## 5. The single change that made learning possible

**Pong is controlled by LEFT/RIGHT, not up/down.** The paddle appears to move
vertically, but the ROM reads the joystick's horizontal axis. Measured with
`ale_py`:

```
pong minimal_action_set: [NOOP, FIRE, RIGHT, LEFT, RIGHTFIRE, LEFTFIRE]
random UP/DOWN only    : total_reward -41   (paddle never moves)
random LEFT/RIGHT only : total_reward -30   (paddle plays)
```

So **servo 51 is the critical actuator** for Pong and servo 52 is unused.

The first attempt used the stock 6-action minimal set and scored **+0 points over
4,600 steps**. The reason was action aliasing: this robot's fire linkage was
mechanically disconnected, so

- `FIRE` behaved identically to `NOOP`
- `RIGHTFIRE` behaved identically to `RIGHT`
- `LEFTFIRE` behaved identically to `LEFT`

Six actions with only **three distinct physical consequences**. The agent had to
learn its way out of a distinction that did not exist, while the fire servo moved
on half of all steps for nothing.

Restricting `action_set` to `[0, 3, 4]` took the run from **+0** to visible
learning within the first few thousand steps. **An action the hardware cannot
perform is worse than a missing action — it is a duplicate that costs a servo
throw.**

---

## 6. Throughput: the robot is the bottleneck

Measured **~15 steps/s** against a configured 30, so the run took **12h 54m**
rather than the ~6h the config assumes.

The cause is mechanical. A 240-tick joystick throw is **torque-limited, not
speed-limited** — `goal_speed` 1200, 2400 and 3400 all produce identical times,
because the servo never reaches even 1200 steps/s against the joystick's spring.
`torque_limit` is the real lever:

| `torque_limit` | 240-tick throw | effective speed |
|---|---|---|
| 300 | 280 ms | ~1070 steps/s |
| **500** (used) | **205 ms** | ~1590 steps/s |
| 1000 | 184 ms | ~1830 steps/s |

There is a further ~53 ms of dead time before any motion, which P gain reduces
(55 → 38 ms at P=128) but does not remove. Even the best case exceeds the paper's
entire ~165 ms perception-to-action budget, so it **cannot be tuned away** — it
needs a longer servo horn arm, which trades angle for linear travel.

Consequence: the RTX 4090 sits at **0% GPU utilisation between steps**. Nothing
about this workload is compute-bound.

---

## 7. Caveats — read before citing these numbers

1. **Three-action Pong.** The fire linkage was mechanically disconnected, so the
   agent never had FIRE, RIGHTFIRE or LEFTFIRE. Not comparable to the paper's
   full action set.
2. **`ring_size` 200,000, not 400,000.** A 24 GB card cannot hold the paper's
   replay ring. This halves replay diversity — 1.85 h of stored experience
   instead of 3.70 h — and is a real algorithmic difference.
3. **Episode counts are zero.** `training_total_episodes` and
   `eval_total_episodes` are both 0: game-over boundaries are never detected
   through the camera. The reward channel works, but there are no episode-level
   metrics, so average-reward-per-step is the only aggregate available.
4. **Not yet reproduced.** A post-hoc evaluation run was attempted on 2026-08-07
   but abandoned: servo 51's left throw had by then developed a mechanical fault
   (§8), which would have understated the policy badly.

---

## 8. Post-run hardware regression (2026-08-07)

Servo 51's left throw stopped reaching the switch. Documented because the
diagnosis method generalises.

Mapping **every servo position** against the switch states — rather than sweeping
deflections, which assumes the neutral is still correct — gave:

- **RIGHT closes at 2724** — 160 ticks from neutral, a clean wide window
- **LEFT never closes above 2060** — 500 ticks from neutral

The servo reaches the joystick's **mechanical left stop near 2130**, and even
pressed against it the left contact is only ~58% reliable. Pushing deeper merely
stalls the servo — position error grows 31 → 179 ticks with no gain in contact.

Ruled out along the way, each by direct measurement:

- **ESP32, GPIO, wiring, and the switch itself** — a hand press works perfectly,
  and the bridge streams a clean 1000 frames/s
- **Horn slipping** — left draws 448 mA peak vs right's 390 mA, so it is doing
  real work
- **Insufficient travel** — deeper makes it *worse*: 320 ticks → 22.4%,
  360 → 9.9%, 400 → 1.3%, 440 → 0.0%

Workaround applied: `dpad_servo_left` 2324 → **2064**, servo 51 min angle limit
2244 → **2020**. This is a workaround, not a fix; the coupling needs mechanical
attention. It held solid for 1876 frames the morning of the run and degraded
through the day (1876 → 959 → 181 → 0), which points at something loosening.

---

## 9. Reproducing

```bash
# 1. Devbox (laptop) — game on its own monitor, joystick from the ESP32
cd devbox_code/build
./PhysicalALE ../games pong --display=1 --fps=60 --log-input \
  --input=serial:/dev/serial/by-id/usb-Silicon_Labs_CP2102_...-if00-port0

# 2. Agent (4090) — verify before committing hours to a run
python3 setup_scripts/preflight_agent.py --config <robot config>
python3 setup_scripts/verify_actions_hw.py --config <robot config> --execute

# 3. Train
cd agent_code
setsid nohup python3 -u learn_policy.py \
  --config experiment_configs/agent_action_input_real.json --run 0 \
  > ~/train_pong.log 2>&1 &

# 4. Watch
python3 setup_scripts/training_status.py --watch

# 5. Evaluate a saved policy
python3 evaluate_policy.py --checkpoint-dir ./checkpoints/<run_name> \
  --env real --device cuda --eval_steps 20000 \
  --robotroller-config ./physical_atari_configs/<robot config>
```

Always launch long runs **detached** (`setsid nohup`). An ssh drop killed a
monitoring wrapper mid-run; the detached training was unaffected.

---

## 10. What this run taught

Ordered by how much time each cost to learn.

1. **A silent input failure looks exactly like a bad policy.** The ESP32
   re-enumerated `ttyUSB0` → `ttyUSB1` mid-run; the devbox kept a stale file
   descriptor, read nothing forever, and fed the emulator "nothing pressed" with
   no error anywhere. Servos moved, rewards were a uniform stream of −1, and
   every log looked healthy. **A `/dev/serial/by-id/` path only protects you at
   `open()` — it does nothing once an already-open device re-enumerates.**

2. **Detect a dead serial link by silence, not by an error code.** With
   `VMIN=0/VTIME=0` a non-blocking tty returns 0 both for "no data right now" and
   for a device that has been unplugged, so checking `errno` never fires. The
   bridge free-runs at 1 kHz, so one second of silence means the link is gone.
   On disconnect, **release every button** — a stuck direction is worse than no
   input.

3. **Verify the whole chain end to end before every long run.** Two separate
   "0 tags detected" results turned out to be "no tags were being displayed" —
   the fullscreen game window had been buried by a terminal, while the process
   kept running and logging episodes. Wayland blocks screenshots, so the camera
   is the only ground truth for what is on screen.

4. **Give the game a monitor of its own.** `SDL_VIDEO_FULLSCREEN_DISPLAY` is an
   SDL **1.2** variable that SDL2 ignores, and `SDL_WINDOWPOS_CENTERED` always
   resolves to display 0 — so the game silently lands on the wrong screen. Added
   `--display=N`. Mirroring displays is worse than useless: any window straying
   over a corner AprilTag kills the homography.

5. **Camera parameters from a different camera are worse than no parameters.**
   The paper's exposure of 20 is black on this sensor (mean pixel 6/255);
   brightness/contrast of 128 are out of range (actual −64..64 and 0..64) and
   silently clamp; focus and zoom do not exist at all (fixed-focus, OpenCV
   returns −1). Also `CAP_PROP_AUTO_EXPOSURE 0.25` is the pre-OpenCV-5 encoding
   and is **silently ignored** — use `1`, and read the property back.

6. **`quad_decimate` is really a proxy for camera framing.** With the screen at
   half the frame (tag edges 44–58 px), the stock 2.0 decoded one corner tag in
   **12%** of frames — fatal, since the homography needs all four. Re-aiming so
   the screen fills the frame (edges 79–98 px) made 2.0 correct again.

7. **Set current thresholds from the peak during motion, not from holding
   current.** Holding draws 2–10 counts; the same move peaks at 59–69. A
   threshold set from the holding figure fired the overcurrent reflex on nearly
   every action and cycled torque mid-move.

8. **Checkpoint periodically, and on interrupt.** As shipped, the only save
   happens after training *and* evaluation complete — ~12.9 h in — and both
   `KeyboardInterrupt` handlers re-raise without saving. A Ctrl-C at hour 11
   discards everything. Now saves every 10,000 steps and on interrupt.

9. **`pkill` on a training run leaves the servos energised**, because the
   Robotroller destructor never runs. Use SIGINT.

10. **Config sweepers take the cartesian product of every list-valued key.**
    `"action_set": [0,3,4]` silently becomes three experiments with
    `action_set` = 0, then 3, then 4. It must be double-wrapped: `[[0,3,4]]`.
    Check the printed `Total experiment configurations: 1`.
