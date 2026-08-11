#!/bin/bash
# astra_ffmpeg.sh — capture the Astra RGB (MJPEG) and publish H.264 to mediamtx.
# Called by mediamtx's runOnDemand for the "astra" path (spawned only when a
# viewer is connected). Auto-detects the Astra RGB node (USB product 0501).
set -e

DEV=""
FOUND=0
for v in /dev/video*; do
  n=$(basename "$v")
  p=$(cat "/sys/class/video4linux/$n/device/../idProduct" 2>/dev/null || true)
  if [ "$p" = "0501" ]; then DEV="$v"; FOUND=1; break; fi
done

if [ "$FOUND" -eq 0 ] || [ -z "$DEV" ]; then
  echo "ERROR: Astra RGB camera (product ID 0501) not found!" >&2
  exit 1
fi

# ENCODER: libx264 (CPU) is low-latency and cheap at 640x480. Swap to
# h264_v4l2m2m for the Jetson hardware encoder to nearly eliminate the CPU cost.
# `nice -n 15` keeps this encoder from starving the server's control loop (which
# is single-threaded) — control input stays responsive while the camera streams.
#
# BITRATE: was 4M, which is what made the GUI feed go black.  The robot rides
# UCR-SECURE on 2437 MHz (2.4 GHz, ch 6) — a shared campus channel.  Measured from
# the laptop while a 4 Mbps stream was running: 56.7% packet loss, RTT avg 3.5 s,
# peaking at 15 s.  With the stream stopped, the same link went to 8.3% loss and
# ~1 s.  The robot-side pipeline was never the problem — a local reader held a
# rock-steady 31 fps for 3.5 minutes — the wireless link simply cannot carry 4 Mbps.
#
# 1.5 Mbps is ample for 640x480 H.264 and leaves headroom for control traffic.
# maxrate/bufsize cap the burstiness, which is what actually saturates a congested
# channel: without them x264 emits large I-frame spikes that blow the queue even
# when the average rate looks fine.  Override with X3_CAM_BITRATE if needed.
BITRATE="${X3_CAM_BITRATE:-1500k}"
MAXRATE="${X3_CAM_MAXRATE:-1800k}"
BUFSIZE="${X3_CAM_BUFSIZE:-900k}"

exec nice -n 15 ffmpeg -hide_banner -loglevel error -fflags nobuffer -flags low_delay \
  -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 30 -i "$DEV" \
  -c:v libx264 -preset ultrafast -tune zerolatency \
  -b:v "$BITRATE" -maxrate "$MAXRATE" -bufsize "$BUFSIZE" \
  -g 30 -bf 0 -pix_fmt yuv420p \
  -f rtsp -rtsp_transport tcp "rtsp://localhost:${RTSP_PORT:-8554}/astra"
