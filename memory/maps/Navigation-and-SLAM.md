---
title: Navigation and SLAM
description: Map of content — odometry, EKF, SLAM Toolbox, Nav2, exploration
tags: [moc, navigation]
---

# Navigation and SLAM

Back to [[Home]].

## Odometry stack

```
Encoder ticks → base_node_X3        → /odom_raw
IMU           → Madgwick filter     → /imu/data
/odom_raw + /imu/data → robot_localization EKF → /odom + TF
```

Config: `src/yahboomcar_nav/params/ekf_x3.yaml`.

## Navigation

- **Nav2** — params in `src/yahboomcar_nav/params/nav2_params_x3.yaml` (DWA planner/controller).
  Client wrapper `src/nav2_client.py`: `navigate_to()`, `set_initial_pose()`, goal tracking
  with path sampling.
- **SLAM Toolbox** — `src/yahboomcar_nav/params/slam_toolbox_params.yaml`. Started from the
  GUI "Start SLAM" button; maps saved through the server's map-save handler.
- **Frontier exploration** — `src/frontier_explorer.py` finds free/unknown boundaries on the
  OccupancyGrid, clusters them, sends the nearest centroid to Nav2.
- **Target-tracking FSM** — `src/navigation_fsm.py`:
  IDLE → SEARCHING → APPROACHING → ARRIVED → AVOIDING → RETURNING.

## Maps

`.pgm` / `.yaml` pairs in `src/yahboomcar_nav/maps/`.

## Gotchas worth remembering

- The lidar sees the robot's own chassis, which pollutes costmaps and the CBF safety filter
  until the riser bracket lands. Self-returns cluster under ~0.45 m.
  → [[project_lidar_mount]], `LIDAR_MOUNT_PLAN.md`
- RViz on the laptop can list topics while no data ever arrives — a DDS discovery problem,
  not a navigation one. Don't debug the planner first. → [[Troubleshooting-DDS]]

## Related

- [[Hardware]], [[Software-Architecture]], [[Perception]]
