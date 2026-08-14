#!/usr/bin/env python3
"""Serve the raw camera as MJPEG over HTTP, so it can be watched from another machine.

The camera lives on the agent machine, which is usually driven over ssh with no
display. `visualize_stream.py` records processed 160x210 agent observations to a
file and drives the servos to do it; this shows the plain camera image live, and
touches no hardware but the camera.

Its real job is aiming. With --tags it overlays AprilTag detections and prints a
running per-tag hit rate, so the camera can be positioned until all four corner
tags are solid -- the homography needs all four, and a corner that decodes
intermittently is the difference between a working agent and a dead one.

    python3 setup_scripts/camera_stream.py --config <config> --tags
    # then open http://<agent-ip>:8080/ in a browser

NOTE: this listens on all interfaces so another machine can reach it, which
means anyone on the network can view the camera while it runs. Stop it when
you are done aiming.
"""
import argparse
import json
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))

latest_jpeg = None
latest_lock = threading.Lock()
stats = {"frames": 0, "tags": {}, "fps": 0.0}


def capture_loop(args, cam):
    global latest_jpeg
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"ERROR: could not open camera index {args.device}", file=sys.stderr)
        os._exit(1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*cam.get("fourcc", "YUYV")))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
    # 1 = V4L2_EXPOSURE_MANUAL. OpenCV 5 passes the enum through; the old 0.25
    # encoding is silently ignored and leaves the camera on auto.
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, cam["exposure"])
    cap.set(cv2.CAP_PROP_BRIGHTNESS, cam["brightness"])
    cap.set(cv2.CAP_PROP_CONTRAST, cam["contrast"])

    det = None
    if args.tags:
        from pupil_apriltags import Detector
        det = Detector(families="tag36h11",
                       quad_decimate=float(cam.get("apriltag_quad_decimate", 2.0)),
                       refine_edges=0, decode_sharpening=0.25, nthreads=4)

    seen, n, t0 = {}, 0, time.perf_counter()
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue

        if det is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            hits = det.detect(gray)
            for h in hits:
                seen[h.tag_id] = seen.get(h.tag_id, 0) + 1
                c = h.corners.astype(int)
                cv2.polylines(frame, [c], True, (0, 255, 0), 2)
                edge = int(abs(h.corners[0][0] - h.corners[1][0]))
                cv2.putText(frame, f"{h.tag_id} ({edge}px)", tuple(c[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            n += 1
            rates = " ".join(f"{k}:{100*v/n:.0f}%" for k, v in sorted(seen.items()))
            cv2.putText(frame, f"tags {len(hits)}/6   {rates}", (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if len(hits) >= 6 else (0, 165, 255), 2)
            if n >= 60:   # rolling window, so moving the camera shows up quickly
                seen, n = {}, 0
            stats["tags"] = {k: round(100 * v / max(n, 1)) for k, v in seen.items()}

        stats["frames"] += 1
        if stats["frames"] % 30 == 0:
            now = time.perf_counter()
            stats["fps"] = 30.0 / (now - t0)
            t0 = now
        cv2.putText(frame, f"{stats['fps']:.1f} fps", (10, frame.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
        if ok:
            with latest_lock:
                globals()["latest_jpeg"] = buf.tobytes()


PAGE = b"""<!doctype html><meta charset=utf-8><title>camera</title>
<style>body{margin:0;background:#111;display:grid;place-items:center;height:100vh}
img{max-width:100vw;max-height:100vh}</style><img src="/stream.mjpg">"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # keep the console readable

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        try:
            while True:
                with latest_lock:
                    jpg = latest_jpeg
                if jpg is None:
                    time.sleep(0.02)
                    continue
                self.wfile.write(b"--FRAME\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n")
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / 30)
        except (BrokenPipeError, ConnectionResetError):
            pass  # viewer closed the tab


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True, help="robot config JSON (for camera settings)")
    ap.add_argument("--device", type=int, default=None, help="camera index (default: from config)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality for the stream")
    ap.add_argument("--tags", action="store_true",
                    help="overlay AprilTag detections and per-tag hit rate (needs pupil-apriltags)")
    args = ap.parse_args()

    cam = json.load(open(args.config))["camera"]
    if args.device is None:
        dev = cam["device"]
        args.device = int(str(dev).rsplit("video", 1)[-1]) if "video" in str(dev) else int(dev)

    threading.Thread(target=capture_loop, args=(args, cam), daemon=True).start()

    ip = socket.gethostbyname(socket.gethostname())
    print(f"camera {args.device} at {cam['width']}x{cam['height']} "
          f"{cam.get('fourcc', 'YUYV')}, exposure {cam['exposure']}")
    print(f"open  http://{ip}:{args.port}/   (or http://<this-host>:{args.port}/)")
    print("visible to anyone on this network -- stop it when you are done.")
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    raise SystemExit(main())
