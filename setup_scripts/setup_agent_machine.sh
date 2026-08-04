#!/usr/bin/env bash
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
#
# setup_agent_machine.sh - install everything the agent machine needs.
#
# The agent machine is the Linux box with the GPU and the camera. It runs the RL
# agent, watches the Devbox screen, and drives the Robotroller's servos.
#
#   ./setup_scripts/setup_agent_machine.sh            # everything
#   ./setup_scripts/setup_agent_machine.sh --no-apt   # skip sudo apt steps
#
# Afterwards, run setup_scripts/preflight_agent.py to confirm the machine is
# actually ready.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCSERVO_DIR="${SCSERVO_DIR:-$(dirname "$REPO_ROOT")/SCServo_Linux}"
DO_APT=1

for arg in "$@"; do
    case "$arg" in
        --no-apt) DO_APT=0 ;;
        -h|--help) sed -n '16,26p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 1 ;;
    esac
done

say() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------------------
say "1/6  System packages"
# ---------------------------------------------------------------------------
# libapriltag-dev is required HERE (the agent detects the tags); the devbox does
# not need it. pybind11 and OpenCV are for the robotroller Python module.
if [ "$DO_APT" -eq 1 ]; then
    sudo apt-get update
    sudo apt-get install -y \
        build-essential cmake git pkg-config \
        libopencv-dev python3-dev python3-pip python3-pybind11 \
        libapriltag-dev v4l-utils
else
    echo "skipped (--no-apt)"
fi

# ---------------------------------------------------------------------------
say "2/6  Serial port permission"
# ---------------------------------------------------------------------------
# Without this every servo command fails with a permission error that looks like
# a wiring fault.
if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    echo "$USER is already in the 'dialout' group"
else
    echo "Adding $USER to the 'dialout' group (needed to open the servo bus)"
    sudo usermod -aG dialout "$USER"
    echo "!! Log out and back in for this to take effect."
fi

# ---------------------------------------------------------------------------
say "3/6  SCServo_Linux (Feetech servo SDK)"
# ---------------------------------------------------------------------------
if [ ! -d "$SCSERVO_DIR" ]; then
    echo "Cloning SCServo_Linux into $SCSERVO_DIR"
    git clone https://github.com/adityakamath/SCServo_Linux "$SCSERVO_DIR"
fi
cmake -S "$SCSERVO_DIR" -B "$SCSERVO_DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$SCSERVO_DIR/build" -j"$(nproc)" --target SCServo
echo "SCServo built at $SCSERVO_DIR/build"

# ---------------------------------------------------------------------------
say "4/6  Python packages"
# ---------------------------------------------------------------------------
# Install torch FIRST and separately: the CUDA build is large and picking the
# wrong index here is the usual cause of a CPU-only torch on a GPU machine.
python3 -m pip install --upgrade pip
if python3 -c "import torch" 2>/dev/null; then
    echo "torch already installed: $(python3 -c 'import torch;print(torch.__version__)')"
else
    echo "Installing torch (CUDA build)."
    echo "If this picks the wrong CUDA version, install it manually from"
    echo "https://pytorch.org/get-started/locally/ and re-run with --no-apt."
    python3 -m pip install torch
fi
python3 -m pip install -r "$REPO_ROOT/agent_code/requirements.txt"
python3 -m pip install -r "$REPO_ROOT/setup_scripts/requirements.txt"

# ---------------------------------------------------------------------------
say "5/6  robotroller Python module (camera + AprilTags + servos)"
# ---------------------------------------------------------------------------
# Built against the interpreter that is active now, because cmake installs it
# into that interpreter's site-packages. Build inside your venv/conda env.
cmake -S "$REPO_ROOT/input_output_cpp_library" \
      -B "$REPO_ROOT/input_output_cpp_library/build" \
      -DSCSERVO_ROOT="$SCSERVO_DIR"
cmake --build "$REPO_ROOT/input_output_cpp_library/build" -j"$(nproc)"
echo "Installing the module (needs sudo unless you are in a venv):"
sudo cmake --install "$REPO_ROOT/input_output_cpp_library/build" \
    || cmake --install "$REPO_ROOT/input_output_cpp_library/build"

# ---------------------------------------------------------------------------
say "6/6  Robot configuration file"
# ---------------------------------------------------------------------------
CFG_DIR="$REPO_ROOT/agent_code/physical_atari_configs"
if [ -f "$CFG_DIR/physical_atari_sample_config.json" ]; then
    echo "$CFG_DIR/physical_atari_sample_config.json already exists - not overwriting"
else
    cp "$CFG_DIR/robotroller.default.json" "$CFG_DIR/physical_atari_sample_config.json"
    echo "Created $CFG_DIR/physical_atari_sample_config.json"
fi

cat <<'EOF'

=============================================================================
Installed. Three things still need doing by hand:

 1. Edit agent_code/physical_atari_configs/physical_atari_sample_config.json
      robot.serial_port  <- ls /dev/serial/by-id/   (it ships as a placeholder
                             on purpose, so it fails loudly rather than
                             targeting the wrong device)
      camera.device      <- v4l2-ctl --list-devices
      servo positions    <- calibrate on YOUR robot; the shipped values are
                            from a different build and will not fit yours
      P/I/D gains        <- Feetech 1-byte coefficients (0-254), NOT the
                            paper's Dynamixel values

 2. Assign servo IDs 50/51/52, ONE SERVO AT A TIME on an otherwise empty bus:
      python3 setup_scripts/change_id.py --path "$SERIAL" --new_id 52 --execute

 3. Verify the machine:
      python3 setup_scripts/preflight_agent.py
=============================================================================
EOF
