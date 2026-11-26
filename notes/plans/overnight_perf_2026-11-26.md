# Overnight perf + strafe diagnosis — 2026-11-26

Branch `overnight/perf-2026-11-26` (not merged). Raw captures in
`artifacts/overnight-perf-2026-11-26/`. Robot commit at capture time = ac47d9b (same as branch base).

**Caveat:** the robot was **idle, Nav2 off, parked**, one GUI client connected, profiled 03:57–04:02.
No driving load, no Nav2/costmap cost in these numbers. All benchmark numbers marked *laptop* are
from the i9-14900HX laptop, not the Orin.

## 1. Measured baseline (AGX Orin, 12 cores @ 2.2 GHz)

| Process | CPU (top) | Notes |
|---|---:|---|
| `server_x3.py` | **521 %** | 11 OpenBLAS pool threads ≈ 355 %; Python threads ≈ 165 % |
| `livox_scan_node` | 40 % | PointCloud2 setter 21 %, `np.unique` 13 %, display voxels 14 % of samples |
| `Mcnamu_driver_X3` | 29 % | mostly rclpy executor overhead; serial I/O small |
| `livox_imu_relay` | 20 % | rclpy overhead per IMU msg |
| `livox_ros_driver2` | 15 % | C++ |
| ekf / madgwick / foxglove | 6 / 4 / 2 % | |

System: tegrastats CPU ~55–65 % per core, GR3D 0–15 %, RAM 3.6/62.8 GB.
Topics: `/scan` 9.8 Hz, delay 27 ms mean (max 77); `/livox/obstacles` delay 50 ms mean (max 90);
`/cmd_vel_safe` 19–21 Hz (nominal 30) with gaps up to 0.20 s; journal shows 6 event-loop stalls of
0.15–0.79 s in ~75 s.

server_x3 Python threads (top -H): rclpy spin 20 %, Arducam `_capture_loop` 14 %, `oak_floor_obstacles` 13 %,
main loop 12 %, asyncio executor workers 4×~9 %.

**Root of the 5-core idle load:** gdb shows all 11 hot threads in `blas_thread_server` of numpy's bundled
`libopenblas64_p-r0.3.23`. No `OPENBLAS_NUM_THREADS` set. `oak_floor_obstacles.py:128/201` (N×3 matmuls on
thousands of points, 5 Hz) wake the pool; workers spin-yield between calls. Only 4/1731 native samples were
real BLAS compute → nothing needs threaded BLAS.

## 2. Ranked list

| # | Change | Gain (est.) | Effort | Risk | Evidence |
|---|---|---|---|---|---|
| 1 | `OPENBLAS_NUM_THREADS=1` for x3_server (prototype: `X3_BLAS_THREADS=1`, c2df2ed; systemd line commented) | **~3–3.5 cores**, fewer GIL stalls | trivial | low (small calls 1.3–1.6× slower: 3×3 solve 35→56 µs, laptop) | gdb bt, top -H; laptop bench 3027 %→5.5 % CPU with a 512² matmul load |
| 2 | OLED loop runs `iwgetid` on the event loop every 5 s although OLED is unplugged (`server_x3.py:6067`, `if oled:` true with `_device=None`) | removes 0.15 s stalls | trivial | none | journal stall #13 |
| 3 | psutil scan starts a new Thread every 1 s from the event loop (`server_x3.py:5824-5860`) — not a regression of 2b65bde, but `Thread.start()` stalls | ~3.5 % + 0.22 s stalls | small | low | stall #11 stack |
| 4 | Arducam `cap.read()` decodes 1280×800 MJPEG every frame; consumer needs 5 Hz (`drivers_x3.py:285`) → `grab()` + `retrieve()` at need, or camera FPS 10 | ~10 % | small | low | top -H 14 % |
| 5 | livox_scan_node fast cloud path (prototype `X3_LIVOX_FAST_CLOUD=1`, 469c53d) | ~0.12 core | done | low (byte-identical tests) | laptop: setter 6.8 ms→13 µs/frame, first-per-cell 9.0→2.6 ms |
| 6 | oak_floor: 3×3 normal equations instead of `lstsq`; packed-int64 `np.unique`; drop per-point Python loop (lines 49-52, 151, 192-211) | 6–8 % | medium | medium (re-check floor accuracy) | py-spy 11.5 % |
| 7 | rclpy spin: create `/odom_raw` sub only when `/odom` stale; replace TransformListener; fewer QoS-event handlers | 5–10 % | medium | medium | spin thread 20 %, mostly wait-set rebuild |
| 8 | Side detector: `cudaDeviceScheduleBlockingSync` context, person-column-only decode, pre-sized frames (`trt_detector.py:117`) | 3–5 % | small | low | executor samples |
| 9 | Left camera rotate only used frames; skip `get_frame` copy with no viewers | 3–8 % (client only) | small | low | |
| 10 | Cache `os.path.exists(CBF_OFF_FILE)` 1 s TTL (`server_x3.py:2067`) | ~0.7 % | trivial | none | |

Latency: VelocityEstimator polls with `time.sleep` at 10 Hz (`velocity_estimator.py:1186-1388`) → +~50 ms mean;
wait on a new-frame event instead. 30 Hz heartbeat `asyncio.sleep(0.033)` on the event loop explains 19–21 Hz
`/cmd_vel_safe`; items 1–3 remove the identified stalls (biggest tail-latency win). OAK capture→host ~267 ms
(earlier measurement, not re-measured).

## 3. C++ / numba verdicts

| Candidate | Verdict |
|---|---|
| `livox_scan_node` (0.40 core) | **No C++ now.** numpy fixes recover ~0.12 core (laptop-based estimate); rest is rclpy overhead. Revisit if Nav2-on profile shows >0.5 core. |
| `server_x3` CBF solve | No — ~0.5 ms/call (and never sector-prune, per past audit). |
| oak_floor plane fit | numpy rewrite (item 6) first; numba only if still >5 % after BLAS fix. |
| `Mcnamu_driver_X3`, `livox_imu_relay` (29 % / 20 %) | Mostly rclpy per-message overhead. A C++ relay for the IMU (simple, high rate) is the one cheap C++ port worth considering (~0.15–0.2 core); not prototyped. |
| CUDA/IPC offload | Not re-proposed (rejected before; no new evidence). |

## 4. Strafe bug — root cause and proposal (NOT deployed)

Diagnosis by Sonnet, independently reviewed by Codex against Humble Nav2 1.1.20 source
(`strafe_diagnosis_sonnet.md` in the artifacts dir).

1. **RotationShimController in front of MPPI** (`nav2_params_x3.yaml:153-167`, `angular_dist_threshold: 0.785`).
   For a side goal the sampled path point 0.15 m ahead is ~90° off → shim outputs pure rotation until within 45°,
   then MPPI sees a forward path. 1 Hz replanning (stock BT) re-arms it on every new path. Added in 4770360 when MPPI
   was DiffDrive. Raising only the threshold to 135° is *not* clean: Humble disengages at the same threshold and its
   collision lookahead uses the engage threshold.
2. **GUI plain click sends goal yaw 0 (map frame)** (`main.js:796-802`) → spin near goal (GoalAngleCritic, yaw tol 0.25).
3. **Possibly stale installed params.** Nav2 loads from `install/yahboomcar_nav/share/...`; the laptop's install copy
   still has `DiffDrive` and no smoother. Humble's smoother default lateral limits are 0 → a stale file alone reproduces
   the bug. **Verify first on the robot.**
- Ruled out: velocity_smoother wiring (both launch paths route cmd_vel_nav→smoother→cmd_vel, ±0.26 y), param
  namespace/spelling, driver (mixes vy). CBF keeps `linear.y` but, being a 2-D projection, can redirect near obstacles.
- `ax_max/ay_max/az_max` are not read by MPPI 1.1.20 (harmless, dead config).

Proposed diff:

```diff
--- a/src/yahboomcar_nav/params/nav2_params_x3.yaml
+++ b/src/yahboomcar_nav/params/nav2_params_x3.yaml
     FollowPath:
-      plugin: "nav2_rotation_shim_controller::RotationShimController"
-      primary_controller: "nav2_mppi_controller::MPPIController"
-      angular_dist_threshold: 0.785      # 45 deg
-      forward_sampling_distance: 0.15
-      rotate_to_heading_angular_vel: 1.0
-      max_angular_accel: 3.2
-      simulate_ahead_time: 1.0
+      # Omni MPPI tracks lateral paths directly; the shim was added while MPPI was DiffDrive.
+      plugin: "nav2_mppi_controller::MPPIController"
--- a/src/web/main.js
+++ b/src/web/main.js
         if (Math.hypot(d.ex - d.sx, d.ey - d.sy) >= DRAG_MIN_PX) {
             const w2 = canvasPxToWorld(d.ex, d.ey);
             if (w2) theta = Math.atan2(w2.y - d.wy, w2.x - d.wx);
+        } else if (state.nav.mapMode === 'navigate') {
+            const mapYaw = state.latestData.robotPoseMap?.theta;
+            if (Number.isFinite(mapYaw)) theta = mapYaw;
         }
```

Live test: `ros2 param get /controller_server FollowPath.plugin` / `FollowPath.motion_model`, `/velocity_smoother max_velocity`;
goal 0.8 m left with current yaw; echo `/plan`, `/cmd_vel_nav`, `/cmd_vel`, `/cmd_vel_safe`, expect sustained |vy| 0.2–0.26.
**Regression case:** goal behind the robot (the shim was added for an 18–35 s stall recorded under DiffDrive). If it
stalls under Omni, keep the shim with a high threshold and a matching lookahead instead. TwirlingCritic left out of
the first patch.

## 5. Not tried / needs the robot next time

- Apply `OPENBLAS_NUM_THREADS=1` (drop-in + restart) and re-measure server CPU, `/cmd_vel_safe` rate and stall count.
- Profile with **Nav2 running and driving** (controller_server, costmaps, lidar→costmap latency — not measured at all).
- `/tf`, `/odom_raw` rates; `ros2 topic delay` for camera/OAK topics; `/livox/lidar` hz (CustomMsg type not loadable in the CLI env).
- Jetson numbers for the Livox fast path; pidstat (not installed on the robot).
- Verify the robot's installed `nav2_params_x3.yaml` content before testing the strafe fix.
- Items 2–4, 6–10 are analysed only, not prototyped. Gemini CLI was unavailable (not authenticated).
