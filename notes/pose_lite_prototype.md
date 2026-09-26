# Laptop Pose Lite prototype

Standalone CPU inference using MediaPipe Pose Landmarker Lite. This does not
modify the robot, publish ROS messages, control motion, or drive GUI avatars.
The first goal is to evaluate joint visibility from the OAK's low viewpoint.

## Installed environment

- Interpreter: `.cache/pose-lite/venv/bin/python` (Python 3.12).
- Model: `.cache/pose-lite/pose_landmarker_lite.task` (float16 v1, CPU execution).
- Pinned packages: `scripts/requirements_pose_lite.txt`.
- Setup: `bash scripts/setup_pose_lite.sh` (uses uv, or an installed Python 3.12).
- No packages are installed into the system/robot Python environment.

The setup script downloads the official Google model and verifies its SHA-256.
Model and environment are cached locally and ignored by git; inference works
offline after setup.

## Use

Run from the workspace root. These commands open a local preview with joints
and confidence-filtered connecting lines. An image stays open until a keypress;
for video/camera, Q or Escape stops processing. Ctrl+C also stops a run.

```bash
# Cached Google sample image, already downloaded and tested on this laptop:
bash scripts/laptop_pose_lite.sh --source .cache/pose-lite/sample-person.jpg

# An OAK frame or recorded video, when available:
bash scripts/laptop_pose_lite.sh --source /path/to/oak-frame.jpg
bash scripts/laptop_pose_lite.sh --source /path/to/oak-video.mp4 --fps 10

# Optional laptop USB camera (only opened when you run this command):
bash scripts/laptop_pose_lite.sh --source 0 --fps 10

# A live video stream, when the robot is charged and its URL is confirmed:
bash scripts/laptop_pose_lite.sh --source rtsp://x3:8554/astra --fps 10
```

`astra` above is the repository's MediaMTX example path; the OAK proof-of-concept
also uses `oak`. Confirm the active path before testing. A WebRTC `/whep` URL or
robot control WebSocket URL cannot be passed to OpenCV as a video source. If
only those transports are available, use a recorded file first; direct WebRTC/
WebSocket ingestion is not implemented in this prototype.

Options:

- `--headless`: save results without opening a window.
- `--num-poses 2`: allow two people (default one; increases work).
- `--max-frames 100`: finish after 100 processed frames.
- `--visibility 0.5`: minimum visibility AND presence for drawing a joint.
- `--output /path/to/new-run`: explicit output directory; existing runs are
  never overwritten. Otherwise uses `.cache/pose-lite/runs/<unique timestamp>`.
- `--self-test --headless`: model inference on blank frames, no camera/network.

## Timing and output

Each run saves `metadata.json`, `poses.ndjson`, `summary.json`, and the last
annotated frame as `last-frame.jpg`. Each pose has 33 landmarks with confidence,
normalized image coordinates and separate hip-relative 3D estimates in meters.
The metadata lists landmark names in order.

**Hip-relative world landmarks are model predictions, not measured robot/map
coordinates.** `pose_index` is per-frame order, not a stable person ID. Mapping
poses to existing robot tracks, calibrated RGB/depth fusion, and humanoid
retargeting remain later integration work.

For recorded video, `--fps` samples its media timeline (frame index / source
FPS), processing as fast as inference allows. It does not force real-time replay.
For a camera/stream, a reader continuously drains video into a one-frame slot;
older frames are discarded and inference is limited to the requested rate.
Live timestamps are laptop decoded-frame receipt times, not camera exposure
times. Decoder/network buffering can still contribute latency. Timing stats
measure the detector call, not capture/network latency or full CPU utilization.

## Verified on this laptop, 2026-09-25

- Setup rerun and model hash verification succeeded.
- Blank-frame inference: 3 frames, no people.
- Official sample image: one pose with 33 image and 33 world landmarks; skeleton
  visually inspected. Sample: https://storage.googleapis.com/mediapipe-assets/pose.jpg
- A 2 s / 30 FPS video made by repeating that sample: 20 sampled frames at 10 Hz,
  20 poses. Detector median about 9.4 ms, p95 about 20.5 ms in this short run.
  This is a pipeline check on repeated imagery, **not an OAK accuracy test or
  a sustained live-performance benchmark**.
- OpenCV preview window opened successfully.
- Unit checks cover timestamp/sample selection, max-frames, dropping a live
  backlog, release of capture resources, and coordinate/identity export labels.

```bash
.cache/pose-lite/venv/bin/python tests/test_pose_lite_prototype.py
bash scripts/laptop_pose_lite.sh --self-test --headless
```

When charged, test full-body and partial-body views, sitting/crouching, crossings,
and frames with no person. Inspect low-confidence joints and measure latency
before feeding any estimates into the surroundings display. Existing mocap
animation remains unchanged until that integration is explicitly implemented.

Reference API: https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker/python

## Live connection check, 2026-09-26

Robot was running `--webrtc-camera`. The existing publisher on
`rtsp://x3:8554/astra` uses `/dev/video0`, identified through sysfs as
`Astra Pro HD Camera`. The `oak` RTSP path returned 404 (no publisher).
A headless 30-frame test of the Astra stream succeeded: no poses in that sample,
11.45 ms median detector time, 23.16 ms p95, 4.46 s including startup/capture.
This validates laptop inference against live video, **not OAK pose accuracy**.
No server restart, camera reconfiguration, or robot motion was performed.

To run a bounded interactive Astra test when a person is ready:

```bash
bash scripts/laptop_pose_lite.sh --source rtsp://x3:8554/astra --fps 10 --max-frames 600
```

This processes up to 600 frames (roughly a minute after stream startup). Start
with the robot stationary and the person's full body visible; then test turning,
arm movements and partial-body views. Q or Escape exits early.
