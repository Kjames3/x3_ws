# Lab / courtyard follow-up: perf flags and omni-direct Nav2 (from 2026-11-26)

Apartment results: `artifacts/overnight-perf-2026-11-26/robot_ab_2026-11-26/RESULTS.md` (perf flags)
and `artifacts/overnight-perf-2026-11-26/strafe_live_2026-11-26/RESULTS.md` (strafe + rear goals).
Those were idle or short runs. Everything below needs the lab or the courtyard outside it.

Robot config used on 2026-11-26 (`/etc/systemd/system/x3_server.service.d/zz-perf-test.conf`):
X3_BLAS_THREADS, X3_OLED_GUARD, X3_PSUTIL_SCAN_THREAD, X3_CBF_FLAG_CACHE, X3_TRT_FAST,
X3_OAK_FLOOR_FAST, X3_LIVOX_FAST_CLOUD, X3_RCLPY_LEAN, X3_CAM_DECODE_ON_DEMAND (all =1) and
X3_NAV2_PARAMS=omni_direct.

## 1. Group lab recordings (people walking around the robot)
- Record with the flags on: `/cmd_vel_safe` rate and max gap, event-loop stall count (journal
  `event loop blocked`), side-person births/confirms, lidar-person track continuity, CBF behaviour.
- Same session with only `X3_BLAS_THREADS=1` as the comparison, if time allows.
- The stall flags showed nothing at idle (no stalls in either baseline). This is where they must prove themselves.
- `X3_CBF_FLAG_CACHE`: toggle the CBF override both ways with wheels up; it may take up to 1 s.

## 2. Longer Nav2 runs (lab, then courtyard)
- 10-20 min multi-goal runs on omni_direct. Log controller_server CPU, costmap update rate,
  lidar→costmap latency (not measured yet), goal success rate, recoveries.
- Side goals at 0.3, 0.8, 1.5 m; rear goals at 1.0-1.3 m; doorway passes.
- Known tuning items to watch: 36-51° yaw during strafes, turning around instead of reversing on
  rear goals, slow creeping before the final rotation on yaw-changing goals. Next knob: a heading
  critic / TwirlingCritic in `nav2_params_x3_omni_direct.yaml`.
- Exercise the GUI path too: `GUI.html?navKeepYaw=1` plain clicks (not tested on the robot yet).

## 3. Not yet tried on the robot
- C++ IMU relay (`imu_relay_impl:=cpp`; package already built on the robot 2026-11-26).
- Whether to make omni_direct and the perf flags the defaults (today they are all opt-in).
