---
name: project_oak_rosbag_recording
description: How to record OAK-D Lite data to rosbags — requires --oak-ros-publish systemd drop-in; env-var and rclpy gotchas
metadata: 
  node_type: memory
  type: project
  originSessionId: f3e32cca-245e-4a1d-831e-794e6034c5c5
  modified: 2026-07-26T22:23:52.290Z
---

Added 2026-07-26. The OAK-D Lite is driven in-process by `src/oakd_driver.py` inside
`server_x3.py`, and DepthAI opens the device **exclusively** — so no separate node can
ever grab it, and by default the OAK streams are invisible to `ros2 bag record`.

`src/oakd_ros_publisher.py` (new) republishes them as sensor_msgs under `/oak/*`
(depth 16UC1 mm, `depth/camera_info`, left/right mono8, imu, detections as
vision_msgs/Detection3DArray). It is **off unless `server_x3.py` is started with
`--oak-ros-publish`** (also `--oak-ros-rate`, default 10 Hz, and `--oak-ros-no-stereo`).

To enable on the robot, add a systemd drop-in — the unit takes a `$SERVER_ARGS` env var,
and drop-ins apply alphabetically so `30-` wins over the existing `20-webrtc.conf`:
`/etc/systemd/system/x3_server.service.d/30-oak-record.conf` with
`Environment="SERVER_ARGS=--domain-id 42 --webrtc-camera --oak-ros-publish"`,
then `sudo systemctl daemon-reload && sudo systemctl restart x3_server`.
**Delete the drop-in after a recording session** — publishing costs real CPU.
Verify it actually took by checking `--oak-ros-publish` is in `/proc/<MainPID>/cmdline`;
`systemctl restart` succeeding does NOT prove the drop-in installed.

Gotchas that cost real time:
- `record_bag.sh` options are **environment variables, not arguments**:
  `RECORD_OAK_STEREO=true ./record_bag.sh` — NOT `./record_bag.sh RECORD_OAK_STEREO=true`,
  which silently creates an output directory named `RECORD_OAK_STEREO=true` with stereo OFF.
  Same for `RECORD_ASTRA_DEPTH`, `RECORD_DURATION` (auto-stop, for trial runs).
- rclpy `uint8[]` fields (e.g. `Image.data`): rosidl's generated setter early-returns ONLY
  for `array.array('B')`. Passing `bytes` OR a numpy array falls through to an
  `if __debug__` block that validates every element individually (~600k checks per
  640x480 image) — this throttled the publish loop from 10 Hz to ~1 Hz. Always
  `array.array('B', arr.tobytes())`. Cost 44 ms/tick -> ~0.
- Reading a zstd-compressed bag needs `rosbag2_py.SequentialCompressionReader`, not
  `SequentialReader`; it decompresses a `.db3` next to the `.zstd` that should be
  cleaned up afterward (the `.zstd` is what `metadata.yaml` references).

Measured rates on the Jetson: ~8.9 Hz achieved at 25 ms/publish, ~211 MB/min with stereo
(~6.3 GB per 30 min), roughly half that depth-only. See [[project_oakd_lite]],
[[project_robot_deploy]].

OAK NN spatial detections DO work (verified: `person` at 2.8-4.2 m), but fire on only
~12% of frames at conf 0.37-0.43 vs the 0.35 threshold — usable as a bonus label track,
not as reliable training ground truth.
