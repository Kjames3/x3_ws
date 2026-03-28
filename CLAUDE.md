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

## Running the System

### Option A — Direct hardware (no ROS2)
```bash
python3 src/server_x3.py
```

### Option B — Physical robot with full ROS2 stack
```bash
bash scripts/jetson_bringup.sh [DOMAIN_ID]          # default domain 42
python3 src/server_x3.py --ros2 --domain-id 42
```

### Option C — Simulation on laptop (cross-machine)
```bash
bash scripts/laptop_sim.sh [DOMAIN_ID]              # on laptop
python3 src/server_x3.py --ros2 --domain-id 42      # on Jetson
```

### Option D — Full simulation on one machine
```bash
python3 src/server_x3.py --sim
```

### Auto-start via systemd
```bash
# Service file: src/x3_server.service
systemctl start x3_server
```

## Architecture

### Control Flow
```
Browser (WebSocket :8081)
    │
    └─ src/server_x3.py          # Main async WebSocket server
         ├─ Direct serial ──────→ Rosmaster controller (/dev/ttyCH341USB0)
         │                        └─ Motors, battery, OLED
         └─ ROS2 DDS ──────────→ /cmd_vel, /scan, /odom, /imu/data topics
```

### Key Source Files
- **[src/server_x3.py](src/server_x3.py)** — Main WebSocket server (1154 lines). Handles browser connections, telemetry broadcast loop, ROS2 bridge initialization, YOLO inference dispatch.
- **[src/drivers_x3.py](src/drivers_x3.py)** — Hardware abstraction: Rosmaster serial (motors, battery), YDLidar X3, Astra Pro camera (RGB/depth), OLED, mecanum kinematics.
- **[src/nav2_client.py](src/nav2_client.py)** — Nav2 action client: `navigate_to()`, `set_initial_pose()`, goal tracking with path sampling.
- **[src/navigation_fsm.py](src/navigation_fsm.py)** — YOLO-driven target tracking FSM (IDLE → SEARCHING → APPROACHING → ARRIVED).
- **[src/robot_state.py](src/robot_state.py)** — EKF odometry fusing wheel encoders + IMU heading.
- **[src/trt_detector.py](src/trt_detector.py)** — TensorRT YOLO wrapper loading `.engine` files, mimics Ultralytics API.
- **[src/Rosmaster_Lib.py](src/Rosmaster_Lib.py)** — Yahboom official low-level serial protocol library (motors, IMU, battery).

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
- **[src/web/GUI.html](src/web/GUI.html)** — Single-page app: joystick control, camera feed, lidar visualization, SLAM map canvas, Nav2 goal clicking, telemetry display.
- **[src/web/main.js](src/web/main.js)** — WebSocket client; camera frames received as binary WebSocket messages (not JSON) for performance.

## Maps
Saved maps are stored as `.pgm`/`.yaml` pairs in `src/yahboomcar_nav/maps/`. New maps are created via the "Start SLAM" button in the GUI and saved through the server's map-save handler.

## Performance Notes (from PERFORMANCE_PLAN.md)
- Camera frame encoding (`cv2.imencode`) runs in a thread executor to avoid blocking the async event loop.
- JPEG frames are sent as **binary WebSocket messages** (not base64 JSON) to save 2–5 ms/frame.
- Lidar point conversion uses vectorized numpy operations.
- Battery voltage is read at 1 Hz (cached), not per-loop.
