#!/usr/bin/env python3
"""
oakd_webrtc_poc.py — standalone low-latency H.264 PoC for the OAK-D Lite.

Encodes the OAK color camera to H.264 ON THE DEVICE (hardware encoder, zero
B-frames, 1 s keyframes) and publishes it to a local mediamtx server over RTSP
using ffmpeg with `-c copy` (mux only, NO re-encode). mediamtx then serves it as
low-latency WebRTC to the browser.

Deliberately SEPARATE from server_x3.py so it can't destabilise the running
robot. It needs exclusive access to the OAK, so stop the main service first:

    sudo systemctl stop x3_server

Run (three steps):
    1) ./mediamtx mediamtx.yml                 # terminal A (this dir)
    2) python3 oakd_webrtc_poc.py              # terminal B (service stopped)
    3) open  http://<jetson-ip>:8889/oak       # laptop browser

Measure glass-to-glass latency: point the camera at a phone stopwatch, open the
browser view next to it, screenshot, compare. Tune knobs below + mediamtx.yml.
"""
import subprocess
import sys
import time

import depthai as dai

# ----------------------------- knobs -----------------------------
# NOTE: the OAK color sensor (IMX214) only supports 1080P / 4K / 12MP, and caps at
# ~35 fps @ 1080p. 720p is NOT valid for color — it silently falls back to 1080p.
# (For higher fps / lower latency you'd stream a MONO cam instead — grayscale, up to ~120 fps.)
RESOLUTION = "1080p"        # "1080p" | "4k"
FPS        = 35             # 1080p color max on the IMX214
BITRATE    = 4_000_000      # 4 Mbps CBR — sized for 5 GHz WiFi headroom
KEYFRAME_S = 1              # seconds between keyframes (lower = faster recovery, more bw)
RTSP_URL   = "rtsp://localhost:8554/oak"

_RES = {
    "1080p": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
    "4k":    dai.ColorCameraProperties.SensorResolution.THE_4_K,
}


def build_pipeline():
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(_RES[RESOLUTION])
    cam.setFps(FPS)
    cam.setInterleaved(False)

    enc = p.create(dai.node.VideoEncoder)
    enc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
    enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)
    enc.setBitrate(BITRATE)
    enc.setKeyframeFrequency(FPS * KEYFRAME_S)   # e.g. every 30 frames = 1 s
    enc.setNumBFrames(0)                          # no B-frames -> lowest encode latency

    xout = p.create(dai.node.XLinkOut)
    xout.setStreamName("h264")
    cam.video.link(enc.input)
    enc.bitstream.link(xout.input)
    return p


def start_ffmpeg():
    # -c copy: pass the OAK's H.264 straight through (no CPU re-encode).
    cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "warning",
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-use_wallclock_as_timestamps", "1",   # stamp packets (raw h264 carries no PTS)
        "-f", "h264", "-r", str(FPS), "-i", "pipe:0",
        "-c", "copy",
        "-f", "rtsp", "-rtsp_transport", "tcp", RTSP_URL,
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)


def main():
    print(f"OAK H.264 PoC -> {RTSP_URL}  ({RESOLUTION}@{FPS}, {BITRATE // 1000} kbps CBR, "
          f"no B-frames, keyframe/{KEYFRAME_S}s)")
    ff = start_ffmpeg()
    try:
        with dai.Device(build_pipeline(), maxUsbSpeed=dai.UsbSpeed.SUPER) as dev:
            print("OAK USB link:", dev.getUsbSpeed().name)
            if dev.getUsbSpeed().name not in ("SUPER", "SUPER_PLUS"):
                print("WARNING: not on USB3 — high-res H.264 may struggle. Check the cable orientation.")
            # Small blocking queue: never DROP encoded NALs (a drop corrupts video
            # until the next keyframe), but keep the buffer shallow for low latency.
            q = dev.getOutputQueue("h264", maxSize=8, blocking=True)
            print("Streaming... open http://<jetson-ip>:8889/oak in the browser. Ctrl-C to stop.")
            while True:
                pkt = q.get()
                data = pkt.getData().tobytes()
                try:
                    ff.stdin.write(data)
                except (BrokenPipeError, OSError):
                    print("ffmpeg exited; restarting it...")
                    time.sleep(0.5)
                    ff = start_ffmpeg()
    except KeyboardInterrupt:
        print("\nstopping...")
    except Exception as e:
        print(f"ERROR: {e!r}")
        print("If it's a device-busy error, stop the main server first: sudo systemctl stop x3_server")
        sys.exit(1)
    finally:
        try:
            ff.stdin.close()
        except Exception:
            pass
        ff.terminate()


if __name__ == "__main__":
    main()
