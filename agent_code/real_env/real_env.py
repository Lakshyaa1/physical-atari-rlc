#!/usr/bin/python
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

# real_env.py
#
# Physical Atari environment wrapper that mirrors sim_env.py interface
# Uses C++ PhysicalAtariEnv which integrates camera, AprilTags, and RoboTroller
#
# Physical runs require the user to launch PhysicalALE on the devbox manually
# before training (e.g. PhysicalALE ./games/ pong).

import json
import numpy as np
import os
from ale_py import ALEInterface, roms


class RealEnv:
    """
    Physical Atari environment wrapper that provides a clean interface matching sim_env.py.
    This is now a thin wrapper around the C++ PhysicalAtariEnv class.
    """

    def __init__(
        self,
        game,
        config_path,
        seed=0,
        latency_model=None,
        max_frames_without_reward=18_000,
        exp_name="",
        use_reduced_action_set=False,
        action_set=None
    ):
        """
        Initialize the physical Atari environment.

        Args:
            game (str): Name of the Atari game (e.g., "pong", "breakout")
            config_path (str): Path to robotroller config with camera and robot settings
            seed (int): Random seed for reproducibility
            latency_model: Optional latency model (not used in physical env, kept for interface compatibility)
            max_frames_without_reward (int): Truncate episode after this many frames without reward
            exp_name (str): Experiment name (kept for interface compatibility)
            use_reduced_action_set (bool): If True, use minimal action set for the game (default: False)
            action_set (list[int]|None): explicit ALE action values to allow,
                overriding use_reduced_action_set. e.g. [0, 3, 4] for Pong's
                paddle without the fire button.
        """
        self.game = game
        self.seed = seed
        self.latency_model = latency_model  # Not used in physical env, but kept for compatibility
        self.exp_name = exp_name
        self.use_reduced_action_set = use_reduced_action_set

        # Load configuration
        if not os.path.isfile(config_path):
            print(f"Error: robotroller config not found at {config_path}")
            print("Copy physical_atari_configs/robotroller.default.json to your config path, or set robotroller_config_path in your experiment config.")
            raise FileNotFoundError(config_path)

        with open(config_path, "r") as f:
            config = json.load(f)

        # Extract camera configuration
        camera_device_str = config["camera"]["device"]
        # Extract camera index from device string (e.g., "/dev/video0" -> 0)
        if isinstance(camera_device_str, str) and "/dev/video" in camera_device_str:
            camera_device = int(camera_device_str.split("/dev/video")[-1])
        elif isinstance(camera_device_str, int):
            camera_device = camera_device_str
        else:
            camera_device = int(camera_device_str)
        
        camera_focus = config["camera"]["focus"]
        camera_zoom = config["camera"].get("zoom", 100)
        camera_width = config["camera"]["width"]
        camera_height = config["camera"]["height"]
        camera_fps = config["camera"].get("fps", 30)
        camera_exposure = config["camera"].get("exposure", 20)
        camera_brightness = config["camera"].get("brightness", 128)
        camera_contrast = config["camera"].get("contrast", 128)
        # Pixel format. Not every camera does every format at every resolution:
        # a webcam that manages 30 fps YUYV at 640x480 can drop to 10 fps at
        # 720p, where MJPG still holds 30. Defaults to the uncompressed YUYV the
        # code used unconditionally before this became configurable.
        camera_fourcc = config["camera"].get("fourcc", "YUYV")
        # AprilTag detector decimation. 2.0 is the fast default, but it needs
        # the tags to be large in frame; if the screen does not fill the camera
        # view, corner tags stop decoding and the homography fails silently.
        quad_decimate = float(config["camera"].get("apriltag_quad_decimate", 2.0))

        # Extract robot configuration.
        #
        # NOTE: these servos are Feetech STS3215, not the Dynamixel XC-330 used
        # in the paper. The PID values here are Feetech's 1-byte EEPROM
        # coefficients (0-254), NOT Dynamixel's 2-byte RAM registers -- the
        # paper's P=1500/I=6000/D=1500 are meaningless on this hardware. A
        # negative value leaves whatever is programmed in the servo alone.
        serial_port = config["robot"]["serial_port"]
        position_d_gain = config["robot"]["D_gain"]
        position_i_gain = config["robot"]["I_gain"]
        position_p_gain = config["robot"]["P_gain"]
        baud_rate = config["robot"]["baud_rate"]

        # Motion profile and safety. goal_speed is steps/s, goal_acc is in units
        # of 100 steps/s^2, torque_limit is 0-1000, and overcurrent_counts is
        # the high-current reflex threshold in RAW CURRENT COUNTS (~6.5 mA each
        # on an STS3215) -- not milliamps.
        goal_speed = config["robot"].get("goal_speed", 2400)
        goal_acc = config["robot"].get("goal_acc", 0)
        torque_limit = config["robot"].get("torque_limit", 500)
        overcurrent_counts = config["robot"].get("overcurrent_counts", 185)
        
        # Extract servo position configuration
        dpad_lr_default = config["robot"]["dpad_lr_default"]
        dpad_ud_default = config["robot"]["dpad_ud_default"]
        dpad_servo_right = config["robot"]["dpad_servo_right"]
        dpad_servo_left = config["robot"]["dpad_servo_left"]
        dpad_servo_up = config["robot"]["dpad_servo_up"]
        dpad_servo_down = config["robot"]["dpad_servo_down"]
        button_servo_default = config["robot"]["button_servo_default"]
        button_deflection = config["robot"]["button_deflection"]

        # An explicit action_set overrides everything below. Use it when part of
        # the robot cannot actuate: an action the hardware cannot perform does
        # not just waste a step, it ALIASES onto another action (a dead fire
        # button makes LEFTFIRE physically identical to LEFT), so the agent sees
        # two actions with identical consequences and has to learn its way out
        # of a distinction that does not exist.
        if action_set is not None:
            self.action_set = list(action_set)
            print(f"[RealEnv] Using EXPLICIT action set with "
                  f"{len(self.action_set)} actions: {self.action_set}")
        # Initialize ALE to get minimal action set if needed
        elif use_reduced_action_set:
            print(f"[RealEnv] Initializing ALE to get minimal action set for {game}")
            temp_ale = ALEInterface()
            game_path = roms.get_rom_path(game)
            temp_ale.loadROM(game_path)
            minimal_action_set = temp_ale.getMinimalActionSet()
            # Convert Action enums to ints
            self.action_set = [action.value for action in minimal_action_set]
            print(f"[RealEnv] Using minimal action set with {len(self.action_set)} actions: {self.action_set}")
        else:
            # Use full action set: all 18 actions (NOOP through DOWNLEFTFIRE)
            self.action_set = list(range(18))
            print(f"[RealEnv] Using legal action set with {len(self.action_set)} actions")
        
        # Initialize C++ PhysicalAtariEnv (does all the heavy lifting)
        import robotroller

        print(f"[RealEnv] Initializing C++ PhysicalAtariEnv for game: {game}")
        self.env = robotroller.PhysicalAtariEnv(
            game,
            seed,
            camera_device,
            camera_width,
            camera_height,
            camera_focus,
            camera_zoom,
            camera_fps,
            camera_exposure,
            camera_brightness,
            camera_contrast,
            serial_port,
            position_d_gain,
            position_i_gain,
            position_p_gain,
            baud_rate,
            dpad_lr_default,
            dpad_ud_default,
            dpad_servo_right,
            dpad_servo_left,
            dpad_servo_up,
            dpad_servo_down,
            button_servo_default,
            button_deflection,
            goal_speed,
            goal_acc,
            torque_limit,
            overcurrent_counts,
            camera_fourcc,
            quad_decimate
        )
        
        # Cache for last step results
        self._observation = None
        self._reward = 0
        self._terminated = False
        self._truncated = False
        
        print(f"[RealEnv] Environment initialized successfully")

    def step(self, action_index):
        """
        Execute one step in the environment.

        Args:
            action_index (int): Index into the action_set
        """
        # Map action index to the actual action from action_set
        actual_action = self.action_set[action_index]
        # Delegate to C++ implementation
        self.env.step(actual_action)
    

    def perceive(self):
        """Perceive the current state of the environment."""
        self._observation = self.env.getObservation()
        self._reward = self.env.getReward()
        self._terminated = self.env.getTerminated()
        self._truncated = self.env.getTruncated()

    def get_observation(self):
        """Returns the current RGB observation (210x160x3)."""
        return self._observation

    def get_reward(self):
        """Returns the reward from the last step (score change)."""
        return self._reward

    def get_terminated(self):
        """Returns True if episode ended (game over)."""
        return self._terminated

    def get_truncated(self):
        """Returns True if episode was truncated (max frames without reward)."""
        return self._truncated

    def reset(self):
        """Reset the environment for a new episode."""
        self.env.reset()
        self._observation = self.env.getObservation()
        self._reward = 0
        self._terminated = False
        self._truncated = False

    def get_num_actions(self):
        """Returns the number of valid actions in the action space."""
        return len(self.action_set)

    def get_number_of_actions(self):
        """Returns the number of actions in the action space.
        
        Returns the size of the minimal action set if use_reduced_action_set was True
        in the constructor, otherwise returns the size of the legal action set.
        """
        return len(self.action_set)

    def get_ram(self):
        """Returns the current RAM state (not available for physical Atari)."""
        # RAM is not accessible from physical hardware
        return np.zeros(128, dtype=np.uint8)

    def shutdown(self):
        """Clean up resources."""
        pass

