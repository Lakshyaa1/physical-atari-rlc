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
verify_frame.py - check a devbox frame against the agent's vision contract.

The devbox and the agent are coupled only through what a camera sees. That
contract is easy to break silently: change the render scale, the tag size, or
the game rect on the devbox and nothing fails loudly -- the agent just starts
receiving a mis-cropped screen or stops seeing rewards.

This script closes that loop offline. Point it at a frame dumped by
`PhysicalALE --dump-frame=...` and it reproduces exactly what
input_output_cpp_library/src/apriltag_detector.cpp and physical_atari_env.cpp do:

  1. detect the tag36h11 AprilTags
  2. require the four corner tags (IDs 0-3) and the two reward tags
  3. build the same homography (tag corners -> a 12px inset of 1280x720)
  4. warp, then crop the hardcoded game ROI (193, 44, 930, 647)
  5. resize to ALE's native 160x210

If every check passes, a real camera pointed at this screen will produce a
correctly framed observation -- no robot, no agent machine, no camera required.

Usage:
    python3 tools/verify_frame.py frame.bmp [--out-dir DIR]
"""

import argparse
import sys

import cv2
import numpy as np

# Must match input_output_cpp_library/src/physical_atari_env.cpp
OUTPUT_W, OUTPUT_H = 1280, 720
CROP_X, CROP_Y, CROP_W, CROP_H = 193, 44, 930, 647
ALE_W, ALE_H = 160, 210
INSET = 12  # apriltag_detector.cpp maps tag corners to this inset

CORNER_IDS = (0, 1, 2, 3)
VALUE_TAG_IDS = (10, 11, 12)  # 10 = zero/init, 11 = +1, 12 = -1
CHANGE_TAG_IDS = (15, 16, 17)  # cycles once per non-zero reward


def detect_tags(gray):
    """Detect tag36h11 markers, tolerating the OpenCV 4.7 API change."""
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    else:  # OpenCV < 4.7
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)

    found = {}
    if ids is not None:
        for tag_id, quad in zip(ids.flatten(), corners):
            found[int(tag_id)] = quad.reshape(4, 2).astype(np.float64)
    return found


def pick_corner(quad, which):
    """Mirror find_corner() in apriltag_detector.cpp (order-independent)."""
    x, y = quad[:, 0], quad[:, 1]
    if which == "top_left":
        return quad[np.argmin(x + y)]
    if which == "top_right":
        return quad[np.argmax(x - y)]
    if which == "bottom_left":
        return quad[np.argmax(y - x)]
    if which == "bottom_right":
        return quad[np.argmax(x + y)]
    raise ValueError(which)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("frame", help="image dumped by PhysicalALE --dump-frame=")
    parser.add_argument("--out-dir", default=None,
                        help="also write rectified/crop/observation images here")
    args = parser.parse_args()

    frame = cv2.imread(args.frame, cv2.IMREAD_COLOR)
    if frame is None:
        print(f"FAIL: could not read {args.frame}", file=sys.stderr)
        return 1
    print(f"frame          : {frame.shape[1]}x{frame.shape[0]}")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detect_tags(gray)
    print(f"tags detected  : {sorted(tags)}")

    failures = []

    missing = [t for t in CORNER_IDS if t not in tags]
    if missing:
        failures.append(f"missing corner tag(s) {missing} - the agent cannot rectify the screen")
    else:
        print("corner tags    : 0,1,2,3 all present")

    value_seen = [t for t in VALUE_TAG_IDS if t in tags]
    change_seen = [t for t in CHANGE_TAG_IDS if t in tags]
    if len(value_seen) != 1:
        failures.append(f"expected exactly one value tag from {VALUE_TAG_IDS}, saw {value_seen}")
    if len(change_seen) != 1:
        failures.append(f"expected exactly one change tag from {CHANGE_TAG_IDS}, saw {change_seen}")
    if len(value_seen) == 1 and len(change_seen) == 1:
        print(f"reward tags    : value={value_seen[0]} change={change_seen[0]}")
        # The decoder sorts reward tags by vertical position and treats the
        # topmost as the value tag. Confirm that ordering actually holds.
        v_y = tags[value_seen[0]][:, 1].mean()
        c_y = tags[change_seen[0]][:, 1].mean()
        if v_y >= c_y:
            failures.append(
                f"value tag (y={v_y:.0f}) is not above change tag (y={c_y:.0f}); "
                "processRewardTags() would swap them and decode rewards wrongly")
        else:
            print(f"reward order   : value above change ({v_y:.0f} < {c_y:.0f}) OK")

    if failures:
        for f in failures:
            print(f"FAIL: {f}")
        return 1

    src = np.array([
        pick_corner(tags[0], "top_left"),
        pick_corner(tags[1], "top_right"),
        pick_corner(tags[2], "bottom_left"),
        pick_corner(tags[3], "bottom_right"),
    ], dtype=np.float32)
    dst = np.array([
        [INSET, INSET],
        [OUTPUT_W - INSET, INSET],
        [INSET, OUTPUT_H - INSET],
        [OUTPUT_W - INSET, OUTPUT_H - INSET],
    ], dtype=np.float32)

    matrix = cv2.getPerspectiveTransform(src, dst)
    rectified = cv2.warpPerspective(frame, matrix, (OUTPUT_W, OUTPUT_H))
    crop = rectified[CROP_Y:CROP_Y + CROP_H, CROP_X:CROP_X + CROP_W]
    observation = cv2.resize(crop, (ALE_W, ALE_H))

    print(f"rectified      : {rectified.shape[1]}x{rectified.shape[0]}")
    print(f"game crop      : ({CROP_X},{CROP_Y}) {crop.shape[1]}x{crop.shape[0]}")
    print(f"observation    : {observation.shape[1]}x{observation.shape[0]} (ALE native)")

    # A correct crop is dominated by the game picture. An all-black or
    # near-uniform crop means the ROI slid off the game area.
    spread = float(observation.std())
    nonblack = float((observation.max(axis=2) > 24).mean())
    print(f"crop std dev   : {spread:.1f}")
    print(f"crop non-black : {nonblack * 100:.1f}%")
    if spread < 5.0:
        failures.append("crop is nearly uniform - the game ROI is probably off-screen")
    if nonblack < 0.02:
        failures.append("crop is essentially black - the game ROI is probably off-screen")

    # No AprilTag should survive into the observation: the crop is meant to
    # exclude them, and a tag leaking in would feed the agent a spurious cue.
    leaked = detect_tags(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY))
    if leaked:
        failures.append(f"tag(s) {sorted(leaked)} leaked into the game crop")
    else:
        print("tag leakage    : none in crop OK")

    if args.out_dir:
        cv2.imwrite(f"{args.out_dir}/rectified.png", rectified)
        cv2.imwrite(f"{args.out_dir}/crop.png", crop)
        cv2.imwrite(f"{args.out_dir}/observation.png", observation)
        print(f"wrote rectified.png, crop.png, observation.png to {args.out_dir}")

    if failures:
        for f in failures:
            print(f"FAIL: {f}")
        return 1

    print("\nPASS - this frame satisfies the agent's vision contract.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
