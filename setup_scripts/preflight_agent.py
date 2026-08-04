#!/usr/bin/env python3
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

"""
preflight_agent.py - is the agent machine actually ready to run?

Checks everything that silently ruins a run if it is wrong: a CPU-only torch on
a GPU box, a replay buffer larger than the card, a missing camera, an
unconfigured serial port, servos that do not answer.

Catching these here costs seconds. Catching them six hours into an unattended
training run costs the run.

    python3 setup_scripts/preflight_agent.py
    python3 setup_scripts/preflight_agent.py --config agent_code/physical_atari_configs/physical_atari_sample_config.json
"""

import argparse
import json
import os
import shutil
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PASS, WARN, FAIL = "PASS", "WARN", "FAIL"
results = []


def check(name, status, detail=""):
    results.append((name, status, detail))
    colour = {PASS: "\033[32m", WARN: "\033[33m", FAIL: "\033[31m"}[status]
    print(f"  {colour}{status}\033[0m  {name}" + (f"\n         {detail}" if detail else ""))


def bytes_gib(n):
    return n / (2 ** 30)


def check_python_packages():
    print("\nPython packages")
    for mod, why in [("numpy", "arrays"), ("cv2", "camera/vision"),
                     ("torch", "the agent"), ("ale_py", "the simulator"),
                     ("serial", "servo setup scripts")]:
        try:
            __import__(mod)
            check(f"import {mod}", PASS)
        except ImportError:
            check(f"import {mod}", FAIL, f"needed for {why}; run setup_agent_machine.sh")


def check_torch_gpu():
    print("\nGPU / torch")
    try:
        import torch
    except ImportError:
        check("torch", FAIL, "not installed")
        return None
    check(f"torch {torch.__version__}", PASS)

    if not torch.cuda.is_available():
        check("CUDA available", FAIL,
              "torch cannot see a GPU. A CPU-only build will 'work' but run far too "
              "slowly to keep up with the camera. Reinstall the CUDA build from "
              "https://pytorch.org/get-started/locally/")
        return None

    name = torch.cuda.get_device_name(0)
    total = torch.cuda.get_device_properties(0).total_memory
    check(f"GPU: {name}", PASS, f"{bytes_gib(total):.1f} GiB")
    return total


def check_ring_size(total_vram):
    print("\nReplay buffer vs GPU memory")
    cfg_path = os.path.join(REPO, "agent_code/experiment_configs/agent_action_input_real.json")
    try:
        with open(cfg_path) as f:
            cfg = json.load(f)
    except OSError:
        check("real experiment config", WARN, f"not found at {cfg_path}")
        return

    def first(key, default):
        v = cfg.get(key, default)
        return v[0] if isinstance(v, list) else v

    ring = first("ring_size", 400000)
    obs = first("obs_size", 128)
    ch = first("obs_channels", 3)
    need = ring * ch * obs * obs
    detail = (f"ring_size={ring:,} -> {bytes_gib(need):.2f} GiB on the training device "
              f"({ring / 30 / 3600:.2f} h of experience at 30 fps)")

    if total_vram is None:
        check("replay buffer size", WARN, detail + " (no GPU detected to compare against)")
        return
    # Leave headroom for model, activations, optimizer state, CUDA context.
    headroom = total_vram - need
    if headroom < 3 * 2 ** 30:
        check("replay buffer fits", FAIL, detail +
              f"\n         only {bytes_gib(headroom):.1f} GiB would remain - expect an OOM. "
              f"Lower ring_size in {os.path.relpath(cfg_path, REPO)}.")
    elif headroom < 6 * 2 ** 30:
        check("replay buffer fits", WARN, detail +
              f"\n         {bytes_gib(headroom):.1f} GiB headroom - tight but probably workable")
    else:
        check("replay buffer fits", PASS, detail +
              f"\n         {bytes_gib(headroom):.1f} GiB headroom")


def check_robotroller_module():
    print("\nrobotroller module (camera + AprilTags + servos)")
    try:
        import robotroller
        have = [a for a in ("PhysicalAtariEnv", "Camera", "Robotroller") if hasattr(robotroller, a)]
        check("import robotroller", PASS, f"exposes {', '.join(have)}")
    except ImportError as e:
        check("import robotroller", FAIL,
              f"{e}\n         build it: cmake -S input_output_cpp_library -B input_output_cpp_library/build "
              "&& cmake --build ... && sudo cmake --install ...")


def check_roms():
    print("\nAtari ROMs")
    try:
        from ale_py import roms
        p = roms.get_rom_path("ms_pacman")
        check("ale-py ROMs", PASS if p else FAIL, str(p) if p else "ms_pacman not found")
    except Exception as e:
        check("ale-py ROMs", FAIL, str(e))


def check_camera():
    print("\nCamera")
    devs = sorted(d for d in os.listdir("/dev") if d.startswith("video"))
    if not devs:
        check("video device", FAIL, "no /dev/video* found - is the camera plugged in?")
        return
    check("video devices", PASS, ", ".join("/dev/" + d for d in devs))
    if shutil.which("v4l2-ctl") is None:
        check("v4l2-ctl", WARN, "not installed; `sudo apt install v4l-utils` to inspect formats")


def check_serial_and_servos(config_path):
    print("\nServo bus")
    by_id = "/dev/serial/by-id"
    if os.path.isdir(by_id):
        entries = os.listdir(by_id)
        check("stable serial paths", PASS if entries else WARN,
              "\n         ".join(os.path.join(by_id, e) for e in entries) or
              "directory exists but is empty - adapter not connected?")
    else:
        check("stable serial paths", WARN,
              f"{by_id} does not exist - no USB serial adapter connected")

    import grp
    try:
        in_dialout = os.getlogin() in grp.getgrnam("dialout").gr_mem
    except Exception:
        in_dialout = "dialout" in [grp.getgrgid(g).gr_name for g in os.getgroups()]
    check("user in 'dialout' group", PASS if in_dialout else FAIL,
          "" if in_dialout else "servo bus will fail to open; "
                                "sudo usermod -aG dialout $USER, then log out and back in")

    if not config_path or not os.path.isfile(config_path):
        check("robot config", WARN,
              "no config given; copy robotroller.default.json and edit it "
              "(pass --config to check the servos too)")
        return

    with open(config_path) as f:
        cfg = json.load(f)
    port = cfg.get("robot", {}).get("serial_port", "")
    if "CHANGE-ME" in port or not port:
        check("robot.serial_port configured", FAIL,
              f"still the placeholder ({port!r}) - set it from /dev/serial/by-id/")
        return
    if not os.path.exists(port):
        check("robot.serial_port exists", FAIL, f"{port} not found")
        return
    check("robot.serial_port exists", PASS, port)

    try:
        sys.path.insert(0, os.path.join(REPO, "setup_scripts"))
        from feetech_protocol import FeetechBus
        bus = FeetechBus(port, cfg["robot"].get("baud_rate", 1000000))
        try:
            found = bus.scan(range(0, 80))
        finally:
            bus.close()
        want = {50, 51, 52}
        if want.issubset(set(found)):
            check("servos 50/51/52 respond", PASS, f"found {found}")
        elif found:
            check("servos 50/51/52 respond", FAIL,
                  f"found {found}; missing {sorted(want - set(found))}. "
                  "Assign IDs with setup_scripts/change_id.py, one servo at a time.")
        else:
            check("servos respond", FAIL,
                  "nothing answered. Check the EXTERNAL power supply (USB alone does "
                  "not power the servos), the baud rate, and the data wiring.")
    except SystemExit as e:
        check("servo bus", FAIL, str(e))
    except Exception as e:
        check("servo bus", WARN, f"could not scan: {e}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(
        REPO, "agent_code/physical_atari_configs/physical_atari_sample_config.json"),
        help="robot config to validate (and use to probe the servos)")
    args = ap.parse_args()

    print("Physical Atari - agent machine preflight")
    print("=" * 60)
    check_python_packages()
    vram = check_torch_gpu()
    check_ring_size(vram)
    check_robotroller_module()
    check_roms()
    check_camera()
    check_serial_and_servos(args.config)

    n_fail = sum(1 for _, s, _ in results if s == FAIL)
    n_warn = sum(1 for _, s, _ in results if s == WARN)
    print("\n" + "=" * 60)
    print(f"{len(results)} checks: {len(results) - n_fail - n_warn} pass, {n_warn} warn, {n_fail} fail")
    if n_fail:
        print("\nFix the failures above before starting a training run.")
        return 1
    print("\nReady. Smoke-test the stack next:")
    print("  cd agent_code && python3 learn_policy.py \\")
    print("      --config experiment_configs/agent_random_real.json --run 0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
