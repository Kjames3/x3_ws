---
name: project_lidar_mount
description: Lidar is mounted low and sees the chassis; temporary CBF filter workarounds are in place pending a bracket to raise it
metadata: 
  node_type: memory
  type: project
  originSessionId: 13c5b595-e4b2-4465-9fb3-fa495a23240a
---

The YDLidar X3 is mounted **low at the front** (`laser_joint` z=0.11m in
`src/yahboomcar_description/urdf/yahboomcar_X3.urdf`), so its rear/side beams hit the robot's
own body and two mount posts at +24°/−18°, ~0.26–0.35m. This makes the manual-drive CBF
filter (`src/cbf_filter.py` fed by `ROS2Bridge._scan_cb` in `server_x3.py`) either freeze
translation or go blind — no range/cone threshold can separate body from real obstacles.

**Current temporary state (drivable, weak obstacle stopping):** `_scan_cb` gate `0.33 < r < 1.0`,
front-180° cone `abs(angle) <= pi/2`; `HolonomicCBFFilter(safe_distance=0.30, gamma=1.0)`.
These are stop-gaps to roll back once the lidar is raised.

**Plan:** user will fabricate a bracket to raise the lidar above the chassis, then share the
mesh/dimensions/placement to update the URDF; afterward revert the filter params. Full plan
(URDF lines, revert table, frame/180°-yaw caveat, optional footprint-subtraction fallback,
validation) is in repo file **`LIDAR_MOUNT_PLAN.md`**.

Note: the Jetson's `server_x3.py` git checkout is diverged from local — mirror every edit to
both copies. See [[project_robot_deploy]].
