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
- **Controller**: Rosmaster board on `/dev/rosmaster` (`src/63-rosmaster.rules`)
- **Lidar**: YDLidar X3 on `/dev/ttyUSB0` (TOF, 512000 baud; the driver comments
  say 8 Hz / ~1000 beams, the hardware actually does **7.2 Hz / 2795 beams**),
  on a Dynamixel-tilted mount driven via `/dev/openrb150`
  (`src/64-openrb150.rules`)
- **Camera**: Orbbec Astra Pro (RGB + depth)
- **Compute**: Jetson Orin Nano (JetPack 6.2)

### Tilting Lidar Mount

The tilt axis is a **Dynamixel XL430-W250-T (id 1, fw 50)** on an
**OpenRB-150** (`2f5d:2202` → `/dev/openrb150`). It has its own supply; XL430
stall current exceeds 800 mA, so it is never on the Orin 40-pin 5 V rail.

> **The LX-16A is RETIRED.** It, `/dev/lx16a`, `src/62-lx16a.rules`,
> `src/lidar_tilt.py`, `x3_lidar_home.service` and
> `config/lidar_tilt_calibration.json` are all historical. Do not reason about
> the tilt axis from them — the counts scale, the port, the units and the
> calibration file are all different now. `src/lx16a_servo.py` is kept only for
> the bench. Background in `notes/dynamixel_servo_history.md`.

```bash
python3 src/dynamixel_tilt.py --status      # read-only (does NOT drop torque)
python3 src/dynamixel_tilt.py --calibrate   # store current position as level
python3 src/dynamixel_tilt.py --home        # drive back to level
python3 src/dynamixel_setup.py --commission # bench tool: identify/set-id/zero/sweep
```

Calibration lives in `config/lidar_tilt_calibration_dynamixel.json`.
X-series position is **0–4095 over 360° (11.378 counts/deg)** — neither the
LX-16A's 4.167 nor the YB-SD15M scale `Rosmaster_Lib.__arm_convert_angle`
assumes. Zero is the user's **hand-levelled 2032, not 2048**; do not "fix" it.
`tolerance_counts` is 12 because the residual ~10-count error is **backlash,
not PID droop** — do not tune gains, approach from a consistent direction.
Never set the EEPROM Homing Offset: Goal Position stays clamped to 0..4095 *in
the offset frame*, silently halving the reachable arc.

`x3_dynamixel_tilt_home.service` homes the mount at boot, ordered before
`x3_server` (`x3_lidar_home.service` is disabled). `x3_server` also homes on
startup, because a restart while tilted used to gate `/scan` off forever.

Two traps for any new X-series code:
- **The Moving flag (addr 122) lags the Goal Position write**, so "poll until
  not moving" returns before motion starts and looks exactly like a stalled
  servo. Judge arrival on position error with a grace period.
- The GUI 3D sweep is **ping-pong, not sawtooth** (`lidar_scan_loop` reverses
  `sweep_dir` at the ends). `tilt_direction` is **+1** (increasing counts =
  nose DOWN). A -1 was briefly recorded on 2026-09-04 from a forward-sector
  floor-range test that mistook raw laser +X for robot forward (`laser_joint`
  has yaw=pi); it is **wrong**. The settling evidence is a single-run A/B in
  `artifacts/deskew-2026-09-04/sign_comparison.npz`, which stores the same
  returns transformed both ways: -1 puts 13.7% of returns up to **2.4 m
  underground** and finds no ceiling, while +1 puts the floor at z~0 as the
  scene's tightest plane (std 0.041 m) and a ceiling at +2.97 m. Do not
  re-derive this sign by comparing a +30 station against a -30 one — a global
  flip only swaps which physical station gets the label, so both look
  sensible. The +1 recorded for the retired LX-16A is a coincidence, not
  evidence; it does not carry over.

**The Rosmaster and the LX-16A bridge were both CH340s (`1a86:7523`) with no
serial number**, so `/dev/ttyCH341USB0` and `USB1` swapped with enumeration
order; the `/dev/rosmaster` symlink keys on chip revision (`bcdDevice` 8134 vs
8133) to pin it. The OpenRB-150 has a real USB serial number and needs no such
trick — but keep the rule in mind before adding any further CH340 device.

## Key Configuration Files
- **[config/lidar_tilt_calibration_dynamixel.json](config/lidar_tilt_calibration_dynamixel.json)** — XL430 tilt zero (2032) + tolerance_counts (12). The older `lidar_tilt_calibration.json` is the retired LX-16A's and is dead.
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
