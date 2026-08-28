# NUC + remote GPU setup

Replaces the laptop-as-Devbox setup with a single NUC handling all physical
I/O (Devbox render/input *and* agent camera/servos), while heavy compute
(`learn_policy.py`, torch/CUDA) runs on a separate GPU box on the same LAN.

## Progress log (2026-08-28)

NUC at `192.168.0.135`, user `sra`. `tmux` session `devbox` on the NUC holds
everything below and survives SSH disconnects -- reattach with
`ssh sra@192.168.0.135` then `tmux attach -t devbox`.

Done:
- `PhysicalALE` built and installed (`/usr/local/bin`), ROMs in `devbox_code/games`.
- `robotroller` Python module built (`input_output_cpp_library` + `SCServo_Linux`,
  cloned to `~/physical-atari-rlc/SCServo_Linux`, untracked -- reclone if missing)
  and installed into the system `python3`.
- `agent_code` deps installed (torch cpu, opencv, ale-py, gymnasium). Full sim
  pipeline verified end-to-end: `python3 learn_policy.py --config
  experiment_configs/agent_random_sim.json --run 0` runs clean.
- Camera (Kreo Owl Camera, `0c45:636d`, `/dev/video0`) detected and streaming.
  Exposure fixed from a copied-over bad default (`20`, near-black on this sensor)
  to the camera's own default (`157`); brightness/contrast also needed lowering
  from the old camera's `128`/`128` since this sensor's real ranges are
  `-64..64`/`0..64` and were silently clamping to `64`/`64`.
  `physical_atari_configs/physical_atari_sample_config.json` on the NUC has the
  live values (gitignored, per-machine -- not in this repo).
- `PhysicalALE` running fullscreen (not `--windowed`) at 1920x1080 -- windowed
  mode added window-manager chrome that pushed the bottom of the 1920x1080
  render off-screen, clipping the two lower AprilTags. Fullscreen fixed this
  because the render's `frac 1.5` math is sized to exactly fill the display.
- Two real repo bugs found and fixed on this branch: `input_output_cpp_library`
  hardcoded `python` instead of `python3` (breaks on plain Ubuntu 24.04), and
  `agent_random_sim.json`/`agent_random_real.json` were both missing
  `checkpoint_dir`, which `learn_policy.py` requires unconditionally.
- Also worked around (NUC-local, not a repo fix): Ubuntu 24.04's
  `libapriltag-dev` package ships a CMake config with a doubled `lib/lib` path
  bug -- symlinked `/usr/lib/lib/x86_64-linux-gnu/*` and `/usr/lib/lib/include`
  to the real locations. Needed again if this NUC's `libapriltag-dev` is
  ever reinstalled.

Pending / resume here tomorrow:
- **Motor driver (Waveshare bus servo adapter) still not confirmed on the NUC.**
  Root-caused to a bad USB cable (confirmed on a laptop test: nothing enumerated
  at all with the original cable; swapping to a known-good data cable made it
  show up instantly as `/dev/ttyACM0`, `1a86:55d3`). Needs a new USB-C cable.
- **ESP32 was unplugged/lost connection** during testing -- recheck
  `/dev/serial/by-id/` once physically reconnected.
- Once both serial devices are confirmed on the NUC: fill in
  `robot.serial_port` in the sample config (currently a placeholder), re-run
  `setup_scripts/camera_stream.py --tags` to confirm all 6 AprilTags detect
  reliably now that fullscreen is fixed (last check before disconnecting
  showed 3/6 and flaky, but that was pre-fullscreen-fix and not re-verified),
  then run `agent_random_real.json` for the full physical-loop sanity check.
- usbip export to the GPU box (Part 4 below) not started yet.

## Architecture

```
NUC (all physical I/O)                         GPU box (same LAN)
├── Monitor (HDMI/DP)                          └── learn_policy.py, --device cuda
│     PhysicalALE renders game + AprilTags
├── ESP32 (USB, wired to CX40 joystick
│     switches: up/down/left/right/fire)
│     -> feeds PhysicalALE --input=serial
│     -> stays local to the NUC, not exported
├── Camera (USB)
│     watches the screen for AprilTags
└── Servo bus adapter "motor driver" (USB)
      -> serial bus -> 3x Feetech STS3215
         servos (IDs 50/51/52) pressing
         the joystick
```

The ESP32 is only needed for the `--input=serial` debug path (bypassing
camera+servos). The real physical loop (camera -> AprilTag -> servo) closes
through the physical world, not through any software link between
`PhysicalALE` and `agent_code` — both processes run on the NUC.

Camera + servo adapter are exported to the GPU box over the network via
`usbip`, so `learn_policy.py` can run with CUDA while still reading/driving
hardware that's physically on the NUC.

## Part 1 — NUC: build the Devbox side

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y libsdl2-dev cmake build-essential pkg-config git

git clone <this-repo-url> physical-atari-rlc
cd physical-atari-rlc/devbox_code
mkdir -p build && cd build
cmake ..        # libgpiod not found is expected on a NUC (not a Pi) - GPIO input disabled
make -j"$(nproc)"
sudo make install
```

ROMs:
```bash
pip install ale-py
mkdir -p ~/physical-atari-rlc/devbox_code/games
python3 -c "from ale_py import roms; import shutil; from pathlib import Path; d=Path('games'); [shutil.copy(p, d/f'{n}.bin') for n in roms.get_all_rom_ids() if (p:=roms.get_rom_path(n))]"
```

## Part 2 — ESP32: wire it to the NUC

Plug it into the NUC via USB and find its port:
```bash
ls /dev/serial/by-id/
```

If it needs (re)flashing, follow `devbox_code/esp32_joystick_bridge/esp_idf/README.md`
from the NUC. Then run the emulator against it:
```bash
cd ~/physical-atari-rlc/devbox_code
PhysicalALE ./games/ pong --input=serial:/dev/serial/by-id/<esp32-device>
```
Confirm the screen renders and switches register before moving on.

## Part 3 — NUC: agent-side hardware (camera + servos)

```bash
cd ~/physical-atari-rlc
pip install -r setup_scripts/requirements.txt
```

Plug in the camera and the Waveshare servo bus adapter, then:
```bash
ls /dev/serial/by-id/
```

Servos already calibrated (IDs 50/51/52) keep their IDs when moved to the
NUC's adapter — no need to redo `change_id.py`/`change_baud_rate.py`.

```bash
cd input_output_cpp_library && <build/install per its README>
cd ../agent_code
pip install -r requirements.txt
cp physical_atari_configs/robotroller.default.json physical_atari_configs/physical_atari_sample_config.json
```

Edit `physical_atari_sample_config.json`:
- `camera.device` — check with `v4l2-ctl --list-devices` (likely `/dev/video0`)
- `robot.serial_port` — the adapter's `/dev/serial/by-id/...` path

Sanity-check the whole physical loop locally on the NUC before adding the
network hop:
```bash
python learn_policy.py --config experiment_configs/agent_random_real.json --run 0
```

## Part 4 — usbip: export camera + servo adapter to the GPU box

On the **NUC**:
```bash
sudo apt install -y linux-tools-generic linux-tools-$(uname -r)
sudo modprobe usbip-host
sudo usbipd -D

usbip list -l           # find bus IDs for camera + servo adapter
sudo usbip bind -b <camera-busid>
sudo usbip bind -b <servo-adapter-busid>
```

On the **GPU box**:
```bash
sudo apt install -y linux-tools-generic linux-tools-$(uname -r)
sudo modprobe vhci-hcd

usbip list -r <nuc-ip>
sudo usbip attach -r <nuc-ip> -b <camera-busid>
sudo usbip attach -r <nuc-ip> -b <servo-adapter-busid>

v4l2-ctl --list-devices   # confirm camera shows up
ls /dev/serial/by-id/     # confirm servo adapter shows up
```

**Before trusting this for a real run**, check the camera didn't degrade over
the tunnel. UVC cameras stream over isochronous USB transfers, which don't
tunnel over usbip (TCP-wrapped) as cleanly as the servo adapter's bulk/serial
transfers do:
```bash
v4l2-ctl --stream-mmap --stream-count=100   # compare fps to a local-NUC baseline
```

If fps holds steady, proceed. If it stutters/drops, fall back to a hybrid:
keep the camera + AprilTag decode on the NUC, export only the servo adapter,
and send just the decoded tag state to the GPU box over a socket instead of
raw video.

## Part 5 — GPU box: run training

```bash
cd physical-atari-rlc/agent_code
pip install -r requirements.txt   # + torch with CUDA
cp physical_atari_configs/robotroller.default.json physical_atari_configs/physical_atari_sample_config.json
# edit camera.device / robot.serial_port to match the usbip-attached paths
```

Launch the game on the NUC's `PhysicalALE`, then from the GPU box:
```bash
python learn_policy.py --config experiment_configs/agent_action_input_real.json --run 0 --gpu 0
```
`device: cuda` is already the default in that config.
