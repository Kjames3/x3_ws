# Robot A/B, 2026-11-26 evening (Orin AGX, idle, parked, Nav2 off)

Each phase: restart x3_server, settle 120 s, measure 60 s (top every 5 s, tegrastats, ros2 topic hz).
Script: phase.sh. A separate sim job (agx_dryrun make_batch.py, ~1 core) was running during phases 1-3 and is excluded.

| Phase | Flags | server_x3 CPU | /cmd_vel_safe rate | max gap | stalls |
|---|---|---:|---:|---:|---:|
| p0 | none | 465 % | 27.6 Hz | 89 ms | 0 |
| p1 | BLAS_THREADS=1 | 98 % | 28.4 Hz | 42 ms | 0 |
| p2 | + OLED_GUARD, PSUTIL_SCAN_THREAD, CBF_FLAG_CACHE | 96 % | 28.4 Hz | 44 ms | 0 |
| p3 | + TRT_FAST, OAK_FLOOR_FAST, LIVOX_FAST_CLOUD, RCLPY_LEAN, CAM_DECODE_ON_DEMAND | 76 % | 28.6 Hz | 39 ms | 0 |

Other nodes unchanged (Mcnamu 28-30 %, livox_scan_node 20 %, imu_relay 20 %). /scan 9.98 Hz throughout.
With p3 on: /oak/obstacles 5.0 Hz, /livox/obstacles 10 Hz, /imu/data 200 Hz, /odom 50 Hz; side detectors 9-11 ms/detect.
No stalls appeared even in the baseline, so the stall flags could not be shown to help at idle.
