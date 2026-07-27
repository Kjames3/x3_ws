#!/usr/bin/env python3
"""
astra_webrtc.py — standalone WebRTC PoC for the Orbbec Astra COLOR camera.

The Astra has no on-camera encoder, so we encode on the JETSON. This version
captures DIRECTLY with ffmpeg's V4L2 input (MJPEG) instead of OpenCV — measured
~17 fps vs OpenCV's ~10 fps on this unit — then encodes H.264 and publishes RTSP
to mediamtx, which serves low-latency WebRTC.

NOTE: the Astra Pro RGB caps at ~17 fps @ 640x480 (USB2/firmware). That's the
ceiling; WebRTC makes it smoother but can't exceed it.

Conflicts with server_x3.py for the Astra, so stop it first:
    sudo systemctl stop x3_server

Run:
    ./mediamtx mediamtx.yml                                  # terminal A
    python3 astra_webrtc.py                                  # terminal B
    open viewer.html#http://<jetson-ip>:8889/astra/whep      # laptop
"""
import glob
import os
import subprocess
import sys

# ----------------------------- knobs -----------------------------
WIDTH, HEIGHT, FPS = 640, 480, 30      # request 30; the Astra delivers ~17 here
BITRATE = "4M"
# "libx264" = CPU (ultrafast+zerolatency, cheap at this size)
# "h264_v4l2m2m" = Jetson HARDWARE encoder (near-zero CPU) — this ffmpeg build has it
ENCODER = "libx264"
RTSP_URL = "rtsp://localhost:8554/astra"


def find_astra_rgb():
    """The Astra RGB is USB product 0501 (it also exposes a second node); pick the first."""
    for v in sorted(glob.glob("/dev/video*")):
        n = os.path.basename(v)
        try:
            with open(f"/sys/class/video4linux/{n}/device/../idProduct") as f:
                if f.read().strip() == "0501":
                    return v
        except Exception:
            pass
    return None


def main():
    dev = find_astra_rgb()
    if dev is None:
        print("ERROR: Astra RGB camera (product ID 0501) not found.")
        sys.exit(1)
    if ENCODER == "h264_v4l2m2m":
        venc = ["-c:v", "h264_v4l2m2m", "-b:v", BITRATE]
    else:
        venc = ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-b:v", BITRATE]
    cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "warning",
        # capture straight from V4L2 as MJPEG (fast path). V4L2 provides its own
        # timestamps, so we do NOT force wallclock here (that added jitter buffer).
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-f", "v4l2", "-input_format", "mjpeg",
        "-video_size", f"{WIDTH}x{HEIGHT}", "-framerate", str(FPS), "-i", dev,
        # low-latency H.264, no B-frames, 1 s keyframes
        *venc, "-g", str(FPS), "-bf", "0", "-pix_fmt", "yuv420p",
        "-f", "rtsp", "-rtsp_transport", "tcp", RTSP_URL,
    ]
    print(f"Astra RGB: {dev}  ({WIDTH}x{HEIGHT}, {ENCODER} -> {RTSP_URL})")
    print("open viewer.html#http://<jetson-ip>:8889/astra/whep   |   Ctrl-C to stop\n")
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\nstopping...")
    except FileNotFoundError:
        print("ERROR: ffmpeg not found (sudo apt install -y ffmpeg)")
        sys.exit(1)


if __name__ == "__main__":
    main()
