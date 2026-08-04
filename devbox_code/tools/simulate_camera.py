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
simulate_camera.py - how much abuse can the AprilTag link survive?

verify_frame.py checks a pixel-exact screenshot. A real camera is not that: it
sees the screen at an angle, from a distance, through a lens, at a lower
resolution, with noise and imperfect focus. This script takes a screenshot from
`PhysicalALE --dump-frame` and degrades it the way a camera would, then runs the
REAL apriltag detector -- the same library and the same tuning the C++ uses in
input_output_cpp_library/src/apriltag_detector.cpp:

    nthreads=1  quad_decimate=2.0  quad_sigma=0.0
    refine_edges=0  decode_sharpening=0.25  family=tag36h11

quad_decimate=2.0 matters most: the detector halves the image before looking for
quads, so a tag needs roughly twice the pixels you would naively expect.

The useful output is the margin: how small the screen can get in frame, and how
far off-axis the camera can sit, before tags stop resolving. Use it to choose
camera placement before building the rig, not after.

    python3 tools/simulate_camera.py frame.bmp
    python3 tools/simulate_camera.py frame.bmp --sweep
"""

import argparse
import sys

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except ImportError:
    raise SystemExit("pip install pupil-apriltags")

CAM_W, CAM_H = 1280, 720          # matches robotroller config camera.width/height
CORNER_IDS = (0, 1, 2, 3)
REWARD_IDS = (10, 11, 12, 15, 16, 17)


def make_detector():
    """Exactly the settings in apriltag_detector.cpp."""
    return Detector(families="tag36h11", nthreads=1, quad_decimate=2.0,
                    quad_sigma=0.0, refine_edges=0, decode_sharpening=0.25)


def simulate(frame, fill=0.8, yaw_deg=0.0, pitch_deg=0.0, blur=0, noise=0):
    """Render `frame` as a camera at `fill` of the sensor would see it."""
    h, w = frame.shape[:2]

    # Screen corners in the camera image: scaled by `fill`, then skewed to fake
    # an off-axis viewpoint (yaw = camera to one side, pitch = above/below).
    tw, th = CAM_W * fill, CAM_H * fill
    cx, cy = CAM_W / 2.0, CAM_H / 2.0
    yaw = np.tan(np.radians(yaw_deg))
    pitch = np.tan(np.radians(pitch_deg))

    dst = np.float32([
        [cx - tw / 2 * (1 - yaw), cy - th / 2 * (1 - pitch)],
        [cx + tw / 2 * (1 + yaw), cy - th / 2 * (1 + pitch)],
        [cx - tw / 2 * (1 - yaw), cy + th / 2 * (1 + pitch)],
        [cx + tw / 2 * (1 + yaw), cy + th / 2 * (1 - pitch)],
    ])
    src = np.float32([[0, 0], [w, 0], [0, h], [w, h]])

    out = cv2.warpPerspective(frame, cv2.getPerspectiveTransform(src, dst),
                              (CAM_W, CAM_H), flags=cv2.INTER_AREA)
    if blur > 0:
        k = 2 * blur + 1
        out = cv2.GaussianBlur(out, (k, k), 0)
    if noise > 0:
        out = np.clip(out.astype(np.int16) +
                      np.random.normal(0, noise, out.shape).astype(np.int16), 0, 255
                      ).astype(np.uint8)
    return out


def check(img, detector):
    """Return (ok, found_ids, corner_tag_pixel_size)."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dets = detector.detect(gray)
    found = {d.tag_id: d for d in dets}
    corners_ok = all(t in found for t in CORNER_IDS)
    reward_ok = sum(1 for t in REWARD_IDS if t in found) >= 2
    size = 0.0
    if 0 in found:
        c = found[0].corners
        size = float(np.linalg.norm(c[0] - c[1]))
    return (corners_ok and reward_ok), sorted(found), size


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("frame")
    p.add_argument("--sweep", action="store_true", help="explore the robustness envelope")
    p.add_argument("--out", default=None, help="write the simulated camera view here")
    args = p.parse_args()

    frame = cv2.imread(args.frame)
    if frame is None:
        print(f"cannot read {args.frame}", file=sys.stderr)
        return 1
    det = make_detector()

    if not args.sweep:
        img = simulate(frame, fill=0.8)
        ok, found, size = check(img, det)
        print(f"nominal (screen fills 80% of frame, head on)")
        print(f"  tags: {found}")
        print(f"  corner tag ~{size:.0f} px")
        print(f"  {'PASS' if ok else 'FAIL'}")
        if args.out:
            cv2.imwrite(args.out, img)
        return 0 if ok else 1

    print("Screen size in frame (head on) -- how far away can the camera be?")
    print(f"  {'fill':>6} {'tag px':>7}  result")
    for fill in (1.0, 0.8, 0.6, 0.5, 0.4, 0.3, 0.25, 0.2, 0.15):
        ok, _, size = check(simulate(frame, fill=fill), det)
        print(f"  {fill:>6.0%} {size:>7.0f}  {'PASS' if ok else 'FAIL'}")

    print("\nOff-axis angle (screen fills 60%) -- how far off centre can it sit?")
    print(f"  {'yaw':>6} {'pitch':>6}  result")
    for yaw, pitch in ((0, 0), (10, 0), (20, 0), (30, 0), (40, 0),
                       (0, 10), (0, 20), (0, 30), (20, 20), (30, 30)):
        ok, _, _ = check(simulate(frame, fill=0.6, yaw_deg=yaw, pitch_deg=pitch), det)
        print(f"  {yaw:>5}° {pitch:>5}°  {'PASS' if ok else 'FAIL'}")

    print("\nDefocus (screen fills 60%, head on) -- how soft can focus be?")
    for blur in (0, 1, 2, 3, 4, 6, 8):
        ok, _, _ = check(simulate(frame, fill=0.6, blur=blur), det)
        print(f"  blur radius {blur:>2}px  {'PASS' if ok else 'FAIL'}")

    print("\nSensor noise (screen fills 60%, head on):")
    for noise in (0, 5, 10, 20, 30, 45):
        ok, _, _ = check(simulate(frame, fill=0.6, noise=noise), det)
        print(f"  sigma {noise:>3}  {'PASS' if ok else 'FAIL'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
