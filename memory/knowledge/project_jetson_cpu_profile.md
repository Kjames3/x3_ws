---
name: project_jetson_cpu_profile
description: "X3 server CPU profile on the Jetson Orin Nano — top consumers, the psutil fix, and where CPU actually goes"
metadata: 
  node_type: memory
  type: project
  originSessionId: 044e60b1-a709-4124-92fc-a3b52d246764
---

Profiled `server_x3.py` on the live robot (2026-07-12) with `py-spy` (install: `pip install --user py-spy`; needs `sudo -S` for ptrace, sudo pw is `1`). Robot is a **Jetson Orin Nano** (6-core) — reach via `ssh jetson@192.168.1.150` (mDNS `jetson-desktop.local` is flaky). See [[project_robot_deploy]].

**Key fact: the Orin Nano has NO hardware video encoder** (`h264_v4l2m2m` → "Could not find a valid device"; no `/dev/nvhost-msenc`). So the ~50% `ffmpeg libx264` process (WebRTC camera pipeline, launched by `_launch_mediamtx` under `--webrtc-camera`, encodes /dev/video0 640x480@30) CANNOT be GPU-offloaded — only tuned (lower fps/bitrate). NVDEC decode exists, NVENC does not.

**YOLO is already off-CPU:** it runs on the OAK-D Lite's Movidius VPU (blob `src/blobs/yolo26n/yolo26n.blob`, "spatial detection ON"), NOT the Jetson CPU/GPU. Server-side CPU YOLO (`detection_enabled`) defaults OFF. So `GR3D_FREQ 0%` (GPU idle) is EXPECTED and fine — there is no YOLO-to-GPU win to chase. torch is CPU-only (`2.10.0+cpu`).

**Fixed (2026-07-12):** `_is_standalone_test_running()` (server_x3.py ~2028) ran `psutil.process_iter()` scanning ~260 procs' /proc at 30 Hz on the event-loop MainThread (caught red-handed by py-spy in `motion_loop`). Added a 1 Hz result cache (`_standalone_test_cache`). Verified: MainThread no longer appears in psutil in profiles; now mostly idle when no client. This scan is UNRELATED to lidar/CBF avoidance — CBF runs inside `ROS2Bridge.move()` off `/scan`→`_scan_cb`→`_latest_obstacles`; the psutil check only gates whether move() is called during an external test. Applied to BOTH local `/home/kamren/x3_ws` and robot `/home/jetson/x3_ws` (patched in place, files diverge).

**After the fix, top CPU consumers in the python process (no client connected, ~110-126% of a core total):**
1. `oakd` driver thread (`oakd_driver.py:328 _run`) — ~50%. depthai host lib marshaling 37 fps mono depth + NN results over USB. `mono_fps=80` requested (server_x3.py:1049), USB-gated to ~37. LOWERING mono_fps is the biggest remaining lever.
2. `_inference_loop` (velocity_estimator.py) — ~17-30%. depth-centroid cv2/numpy + torch MLP @ `INFER_HZ=10`. Gate on active targets or drop to 5 Hz.
3. ROS2 rclpy spin + FastDDS threads — ~15-25% combined.
Load average dropped 4.19 → ~2.2 (1 min) after the psutil fix, but process total is dominated by #1/#2 which the fix doesn't touch.
