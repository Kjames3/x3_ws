# Overnight prototypes — 2026-11-26 (branch `overnight/perf-proto-2`)

Every item is OFF by default behind its own flag. Laptop numbers only; nothing ran on the robot.
Full suite: 1095 passed, 4 failed, 8 URDF errors (baseline 992/4/8). Item numbers refer to
`overnight_perf_2026-11-26.md`.

| Flag | Item | Laptop result | Check on the robot |
|---|---|---|---|
| `X3_BLAS_THREADS=1` (perf-2026-11-26 branch, included) | 1 OpenBLAS pool | 30 cores → 0.05 under a matmul load | server CPU, `/cmd_vel_safe` Hz, stall count |
| `X3_LIVOX_FAST_CLOUD=1` | 5 livox_scan_node | setter 6.8 ms → 13 µs, first-per-cell 9.0 → 2.6 ms | node CPU, identical clouds |
| `X3_OLED_GUARD=1` | 2 OLED iwgetid | 1.5 ms → 14 µs per loop turn | stall stacks gone |
| `X3_PSUTIL_SCAN_THREAD=1` | 3 psutil thread | 212 → 0.6 µs per check | standalone test detected on start, cleared on exit |
| `X3_CBF_FLAG_CACHE=1` | 10 CBF flag stat | 0.8 → 0.08 µs; override takes ≤1 s | both override transitions, wheels up |
| `X3_CAM_DECODE_ON_DEMAND=1` (+`X3_CAM_DECODE_HZ`, default 10) | 4/9 Arducam + left cam | decode 3x less at 10 Hz | V4L2 backend in log; saving only with no GUI open |
| `X3_TRT_FAST=1` | 8 side detector | pre+post 2.8 → 1.0 ms | replay frames, same boxes; p50/p95 latency with blocking sync |
| `X3_OAK_FLOOR_FAST=1` | 6 oak_floor | step 2.0 → 1.0 ms, same voxels | `/oak/obstacles` + floor-plane journal line on/off |
| `X3_RCLPY_LEAN=1` | 7 rclpy wakeups | spin CPU 6–11% → 3–3.6% (synthetic) | `/odom_raw` sub appears only when `/odom` stale |
| `X3_VEL_WAKE_ON_FRAME=1` | latency | 53 → 0.2 ms at 5 Hz depth; no gain ≥15 Hz | only worth it if OAK depth ≤10 fps (server opens at 80) |
| `X3_NAV2_PARAMS=omni_direct` + GUI `?navKeepYaw=1` | strafe fix | params differ only in FollowPath (tests) | follow `strafe_live_test_2026-11-26.md`; rear-goal regression |
| launch `imu_relay_impl:=cpp` (new pkg `yahboomcar_imu_relay_cpp`) | C++ IMU relay | 7.7% → 3.2% of a core at 200 Hz; parity over 2,400 msgs | needs `colcon build --packages-select yahboomcar_imu_relay_cpp yahboomcar_nav` |

Suggested order next robot session: enable BLAS alone and re-measure first (largest, cleanest A/B),
then the stall flags (2, 3, 10), then one flag at a time for the rest. The strafe test needs Nav2 and
space to drive.

Notes: with ROS sourced, `pytest tests/` collects differently (only the ROS parity test ran);
the 1095 figure is from an unsourced shell, like the baseline.
