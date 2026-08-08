---
title: Software Architecture
description: Map of content — server, ROS2 graph, packages, control flow
tags: [moc, architecture]
---

# Software Architecture

Back to [[Home]]. Canonical reference is [[CLAUDE]] — this map adds the memory around it.

## Control flow

```
Browser (HTTP :8080 GUI files, WebSocket :8081 control)
    │
    └─ src/server_x3.py
         ├─ Direct serial ──────→ Rosmaster (/dev/ttyCH341USB0)
         └─ ROS2Bridge (default) → /cmd_vel pub
                                   /scan /odom /odom_raw /map /voltage
                                   /camera/image_raw subs
```

ROS2 bridge mode is the **default**; `--sim` disables it.

## Key files

| File | Role |
|---|---|
| `src/server_x3.py` | async WebSocket server, telemetry loop, watchdog queue (100 Hz), SLAM/Nav2 lifecycle |
| `src/drivers_x3.py` | hardware abstraction |
| `src/nav2_client.py` | Nav2 action client |
| `src/navigation_fsm.py` | YOLO-driven tracking FSM |
| `src/frontier_explorer.py` | frontier-based autonomous exploration |
| `src/trt_detector.py` | TensorRT YOLO wrapper |

## ROS2 packages

- `yahboomcar_base_node` (C++) — dead-reckoning odometry, publishes `/odom_raw` + TF
- `yahboomcar_msgs` — `Target`, `TargetArray`, `ImageMsg`, `PointArray`, `Position`
- `yahboomcar_bringup` (Python) — Mcnamu driver, calibration
- `yahboomcar_nav` (Python) — SLAM/Nav2/Gazebo launch files, params, EKF config
- `ydlidar_ros2_driver` (C++) — external submodule

## Web GUI

`src/web/GUI.html` served over HTTP :8080 — the HTTP server is **required** so the Web Worker
can load `lidar-worker.js`. Camera frames arrive as **binary** WebSocket messages, not base64
JSON; that alone saves 2–5 ms/frame. Lidar decoding runs off the main thread in the worker.

## Performance notes

See `PERFORMANCE_PLAN.md` and [[project_jetson_cpu_profile]]. Established facts:
`cv2.imencode` runs in a thread executor so it can't block the event loop; battery voltage
is cached at 1 Hz rather than read per-loop; lidar point conversion is vectorized numpy.

## Multi-domain ROS2

Default domain 0; scripts use **domain 42** for Jetson/laptop isolation. Server takes
`--domain-id N`. Cross-machine discovery has been the single biggest time sink —
→ [[Troubleshooting-DDS]], [[project_rviz_debugging]], [[project_robot_deploy]]

## Related

- [[Navigation-and-SLAM]], [[Perception]], [[Data-and-Bags]], [[Ideas-and-Planning]]
