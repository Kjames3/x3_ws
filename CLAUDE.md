# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Yahboom X3 Mecanum Robot** workspace running on a **Jetson Orin Nano** (JetPack 6.2 / Ubuntu 22.04). It combines a Python WebSocket server, ROS2 Humble middleware, and a browser-based GUI for real-time control, SLAM, and autonomous navigation.

## Build Commands

### Build ROS2 packages (run from workspace root)
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select yahboomcar_msgs yahboomcar_description yahboomcar_base_node yahboomcar_bringup yahboomcar_nav ydlidar_ros2_driver
```

### Full build with dependency installation (includes YDLidar SDK)
```bash
bash scripts/build_ros2.sh
```

### Run tests
```bash
source install/setup.bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Connecting to the Robot

**Never ask for the robot's IP address, and never hardcode one.** The robot is on
campus wifi for the duration of the deployment and its lease moves between subnets,
so any address written down goes stale within days.

Use the alias, which resolves the address at connect time:

```bash
ssh x3                         # ~/.ssh/config resolves via scripts/x3-ip
scp file x3:~/                 # scp/rsync work through the same alias
"$(x3-ip)"                     # the bare address, when a command needs one
```

Diagnose or correct the address with:

```bash
x3-ip status                   # cached address, age, how it resolved, reachability
x3-ip set <IP>                 # run this instead of pasting an IP into a prompt
```

Resolution order is Tailscale → cache → `x3.lan` (bench LAN only) → mDNS; see
[scripts/robot_env.sh](scripts/robot_env.sh) for the details and env knobs.
`ROBOT_IP=<addr>` overrides everything for one command. Scripts should
`source scripts/robot_env.sh` rather than defining their own default.

Tailscale is the path that survives the robot changing buildings; set it up on
both machines with [scripts/setup_tailscale.sh](scripts/setup_tailscale.sh).

### Running ROS commands on the robot — use `rjet`

**Never hand-roll `ssh x3 'source ... && export ROS_DOMAIN_ID=42 && ros2 ...'`.**
A non-interactive ssh session on the robot has no ROS on PATH, and it still has the
FastDDS discovery-server variables exported from earlier experiments. Miss the
`unset` and **`ros2 topic list` returns empty with no error** — indistinguishable
from a dead robot, and the usual cause of a session spent debugging nothing.

[scripts/rjet](scripts/rjet) owns that preamble:

```bash
rjet 'ros2 topic list'                     # preamble + run on the robot
rjet -- ros2 topic hz /scan                # argv form, no quoting puzzles
rjet --raw 'ls ~/bags'                     # plain ssh, no ROS
rjet --sudo 'systemctl restart x3_server'  # sudo (password via ~/.x3_sudo, mode 600)
rjet --print 'ros2 node list'              # show the remote script, run nothing
```

The remote exit code propagates. `--domain N` overrides `ROS_DOMAIN_ID` (default 42).
`--sudo` sends the password on stdin, so it never reaches the robot's process list;
it is read from `$X3_SUDO_PASS` or `~/.x3_sudo` and is **never committed**.

## Running the System

ROS2 hardware bridge mode is the **default**. The `--sim` flag disables it for simulation.

### Option A — Physical robot with full ROS2 stack (default)
```bash
bash scripts/jetson_bringup.sh [DOMAIN_ID]          # default domain 42
python3 src/server_x3.py --domain-id 42
```

### Option B — Simulation on laptop (cross-machine)
```bash
bash scripts/laptop_sim.sh [DOMAIN_ID]              # on laptop
python3 src/server_x3.py --domain-id 42             # on Jetson
```

### Option C — Full simulation on one machine
```bash
python3 src/server_x3.py --sim
```

### Auto-start via systemd
```bash
# Service file: src/x3_server.service (runs with --domain-id 42 by default)
# Override mode via /etc/systemd/system/x3_server.service.d/override.conf
systemctl start x3_server
# View logs:
journalctl -u x3_server -f
```

## Architecture

### Control Flow
```
Browser (HTTP :8080 for GUI files, WebSocket :8081 for control)
    │
    └─ src/server_x3.py          # Main async WebSocket server
         ├─ Direct serial ──────→ Rosmaster controller (/dev/ttyCH341USB0)
         │                        └─ Motors, battery, OLED
         └─ ROS2Bridge (default) → /cmd_vel pub; /scan /odom /odom_raw
                                    /map /voltage /camera/image_raw subs
```

### Key Source Files
- **[src/server_x3.py](src/server_x3.py)** — Main WebSocket server. Handles browser connections, telemetry broadcast loop, ROS2Bridge initialization, YOLO inference dispatch, motion watchdog queue (100 Hz drain), SLAM/Nav2 lifecycle.
- **[src/drivers_x3.py](src/drivers_x3.py)** — Hardware abstraction: Rosmaster serial (motors, battery), YDLidar X3, Astra Pro camera (RGB/depth), OLED, mecanum kinematics.
- **[src/nav2_client.py](src/nav2_client.py)** — Nav2 action client: `navigate_to()`, `set_initial_pose()`, goal tracking with path sampling.
- **[src/navigation_fsm.py](src/navigation_fsm.py)** — YOLO-driven target tracking FSM (IDLE → SEARCHING → APPROACHING → ARRIVED → AVOIDING → RETURNING).
- **[src/frontier_explorer.py](src/frontier_explorer.py)** — Autonomous frontier-based exploration: finds free/unknown boundaries on the SLAM OccupancyGrid, clusters them, and sends nearest centroid to Nav2.
- **[src/trt_detector.py](src/trt_detector.py)** — TensorRT YOLO wrapper loading `.engine` files, mimics Ultralytics API.
- **[src/Rosmaster_Lib.py](src/Rosmaster_Lib.py)** — Yahboom official low-level serial protocol library (motors, IMU, battery).

YOLO models live under `src/yolo_models/` in subdirectories (`cans_models/`, `default/`). The active model is resolved by `find_model_path()` which recursively searches for `<name>.pt`.

### ROS2 Packages
- **yahboomcar_base_node** (C++) — Dead-reckoning odometry from encoder ticks; publishes `/odom_raw` and TF.
- **yahboomcar_msgs** — Custom message types: `Target`, `TargetArray`, `ImageMsg`, `PointArray`, `Position`.
- **yahboomcar_bringup** (Python) — Mcnamu driver node and calibration tools.
- **yahboomcar_nav** (Python) — Launch files for SLAM, Nav2, Gazebo simulation; nav params, EKF config.
- **ydlidar_ros2_driver** (C++) — External YDLidar driver submodule.

### Odometry Stack
```
Encoder ticks → base_node_X3 → /odom_raw
IMU → Madgwick filter → /imu/data
/odom_raw + /imu/data → robot_localization EKF → /odom + TF
```

### Multi-Domain ROS2
Default domain is 0. Scripts use domain ID 42 for Jetson/laptop isolation. Server accepts `--domain-id N` flag.

## Hardware
- **Robot**: Yahboom X3 (4-wheel mecanum drive)
- **Controller**: Rosmaster board on `/dev/ttyCH341USB0`
- **Lidar**: YDLidar X3 on `/dev/ttyUSB0` (TOF, 512000 baud, 8 Hz scan)
- **Camera**: Orbbec Astra Pro (RGB + depth)
- **Compute**: Jetson Orin Nano (JetPack 6.2)

## Key Configuration Files
- **[src/yahboomcar_nav/params/nav2_params_x3.yaml](src/yahboomcar_nav/params/nav2_params_x3.yaml)** — Nav2 DWA planner/controller tuning
- **[src/yahboomcar_nav/params/ekf_x3.yaml](src/yahboomcar_nav/params/ekf_x3.yaml)** — EKF fusion config
- **[src/yahboomcar_nav/params/ydlidar_x3.yaml](src/yahboomcar_nav/params/ydlidar_x3.yaml)** — YDLidar driver params
- **[src/yahboomcar_nav/params/slam_toolbox_params.yaml](src/yahboomcar_nav/params/slam_toolbox_params.yaml)** — SLAM Toolbox config

## Web GUI
- **[src/web/GUI.html](src/web/GUI.html)** — Single-page app: joystick control, camera feed, lidar visualization, SLAM map canvas, Nav2 goal clicking, telemetry display. Served via HTTP :8080 (required so Web Workers can load `lidar-worker.js`).
- **[src/web/main.js](src/web/main.js)** — WebSocket client; camera frames received as **binary** WebSocket messages (not JSON).
- **[src/web/lidar-worker.js](src/web/lidar-worker.js)** — Web Worker that decodes and transforms lidar point arrays off the main thread.

## Maps
Saved maps are stored as `.pgm`/`.yaml` pairs in `src/yahboomcar_nav/maps/`. New maps are created via the "Start SLAM" button in the GUI and saved through the server's map-save handler.

## Performance Notes (from PERFORMANCE_PLAN.md)
- Camera frame encoding (`cv2.imencode`) runs in a thread executor to avoid blocking the async event loop.
- JPEG frames are sent as **binary WebSocket messages** (not base64 JSON) to save 2–5 ms/frame.
- Lidar point conversion uses vectorized numpy operations.
- Battery voltage is read at 1 Hz (cached), not per-loop.
