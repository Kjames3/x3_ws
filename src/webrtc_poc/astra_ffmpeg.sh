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
exec nice -n 15 ffmpeg -hide_banner -loglevel error -fflags nobuffer -flags low_delay \
  -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 30 -i "$DEV" \
  -c:v libx264 -preset ultrafast -tune zerolatency -b:v 4M -g 30 -bf 0 -pix_fmt yuv420p \
  -f rtsp -rtsp_transport tcp "rtsp://localhost:${RTSP_PORT:-8554}/astra"
