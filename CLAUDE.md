# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Yahboom X3 Mecanum Robot** workspace running on a **Jetson Orin Nano** (JetPack 6.2 / Ubuntu 22.04). It combines a Python WebSocket server, ROS2 Humble middleware, and a browser-based GUI for real-time control, SLAM, and autonomous navigation.

## Repo skills (`.claude/skills/`)

The skill list often shows these with no description, so the triggers are here.
- `deploy`: "commit and deploy", "push this to the robot", restart the robot or server.
- `robot-sync`: git is dirty or inconsistent, what differs between robot and laptop.
- `capture`: recording runs (C1/C3 RGB-D, rosbags, OAK logs), "am I good to continue".
- `urdf-nudge`: a mount is installed and its STL is in `stls/`, "move it N mm".
- `x3-robot`: any task that touches the robot, its services, ROS 2 graph, serial devices or web GUI.

## Build Commands

### Build ROS2 packages (run from workspace root)
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select yahboomcar_msgs yahboomcar_description yahboomcar_base_node yahboomcar_bringup yahboomcar_nav ydlidar_ros2_driver x3_msgs nav2_mppi_dbas_controller yahboomcar_rviz
```

### Full build with dependency installation (includes YDLidar SDK)
```bash
bash scripts/build_ros2.sh
```
This script passes `--symlink-install`. Not on the robot: the rule is step 5 of `.claude/skills/deploy/SKILL.md`.

### Run tests
The test command is step 1 of `.claude/skills/deploy/SKILL.md` (pytest over `tests/`), the same suite CI runs.

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
# Service file: deploy/systemd/x3_server.service (runs with --domain-id 42 by default)
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
         ├─ Direct serial ──────→ Rosmaster controller (/dev/rosmaster, see Hardware)
         │                        └─ Motors, battery, OLED
         └─ ROS2Bridge (default) → /cmd_vel_safe pub (the driver's only input; the
                                    CBF gate filters everything arriving on /cmd_vel, incl. Nav2);
                                    /scan /odom /odom_raw
                                    /map /voltage /camera/image_raw subs
```

### Repository layout
- `src/*.py` (top level only): the **runtime**, meaning `server_x3.py`, everything it
  imports, and `dynamixel_tilt.py`. systemd units point at these paths.
- `src/tools/<group>/`: everything run by hand. The groups are `calibration`,
  `hw_tests`, `perception` (C1/C3, velocity scoring), `lidar`, `nav` and
  `viewers`. Tools import runtime modules by bare name through a `sys.path`
  header. `tests/conftest.py` puts `src/` and every tools group on the path.
  `server_x3.py` launches `tools/hw_tests/point_to_point_test.py` and
  `tools/perception/ab_comparison_test.py` from the GUI.
- `deploy/`: `systemd/` units, `udev/` rules, `install_service.sh` (copies them into `/etc`).
- `notes/` (incl. `notes/plans/`, `notes/ee244/`), `ideas/` (monthly logs), `scripts/`, `config/`, `tests/`.

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
- **Controller**: Rosmaster board on `/dev/rosmaster` (`deploy/udev/63-rosmaster.rules`)
- **Lidar**: Livox Mid-360S since 2026-10-29 (memory note `project-livox-mid360s.md`).
  The YDLidar X3 and its tilt mount are off the robot; their rules are kept word
  for word in `notes/tilting_lidar_mount_rules.md`.
- **Camera**: OAK-D Pro W for depth and detection (memory note `project_oakd_pro_wide.md`).
  The Orbbec Astra Pro still gives the GUI colour feed (memory note `project_gui_black_camera.md`).
- **Compute**: Jetson Orin Nano (JetPack 6.2)

**The Rosmaster and the LX-16A bridge were both CH340s (`1a86:7523`) with no
serial number**, so `/dev/ttyCH341USB0` and `USB1` swapped with enumeration
order; the `/dev/rosmaster` symlink keys on chip revision (`bcdDevice` 8134 vs
8133) to pin it. The OpenRB-150 has a real USB serial number and needs no such
trick — but keep the rule in mind before adding any further CH340 device.

### Dual ToF arrays (VL53L5CX, front bumper)

Upper array pitched **15°** down, lower **40°**, one I2C bus each on a
**Teensy 4.1** (`scripts/teensy_tof_test/teensy_tof_test.ino`, 8x8 @ 15 Hz)
that streams NDJSON over USB to **`/dev/teensy_tof`** (`deploy/udev/65-teensy-tof.rules`;
`ttyACMn` swaps with the OpenRB-150). Sensors run off a separate 3.3 V buck with
a common ground. `server_x3.py` owns the port (`TeensyToFArrays`), feeds the GUI,
publishes `/tof/{upper,lower}/points` (raw) and `/tof/{upper,lower}/obstacles`
(what the CBF and the local costmap use), and adds the obstacles to the CBF.
`tof_node.py` / `src/tools/hw_tests/tof_array.py` are the old single-sensor i2c-1 bench path.

- **Heights are judged along each zone's LOWEST ray**
  (`tof_geometry.zone_low_edge_points`). A zone grazing the floor reports the
  near end of its floor strip; on the centre ray the upper array's row 3 fakes
  a 6 cm edge at ~0.64 m. Plus 3-frame per-zone persistence for crosstalk ghosts.
- **The base driver adds ~0.14 m/s** (`min_pwm` 28 at 200 PWM per m/s) to any
  nonzero command. The CBF output is re-checked at the executed speed
  (`_limit_for_deadband`); shrinking a small command otherwise does nothing.
- Coverage is a **height** gap, not a distance band: objects >= 6 cm are caught
  from the bumper out to 0.5 m (the upper array sees their tops). The lower
  array sees only the bottom ~3 cm of anything, and its 5.6° zones blend a
  small object with the floor (a 3 cm block reads 1–1.7 cm tall), so it also
  uses a **floor-deficit test**: a zone is an obstacle when it reads > 12 mm
  shorter than its bare-floor depth in `config/tof_floor_baseline.json`
  (bare floor stays within 6 mm; the 3 cm block shortened zones 16–41 mm).
  Frames where the median zone shifts > 10 mm are skipped as chassis tilt.
  Drive-tested: a 3 cm block, sandals, shoes and a folded towel all stopped
  the robot short. **Re-record the baseline** with `src/tools/calibration/tof_floor_baseline.py`
  (robot still, clear floor) after touching the bracket or on a very
  different floor; `--tof-baseline PATH` selects a per-floor copy
  (`tof_floor_baseline_{apartment_wood,lab_carpet}.json`) and a missing file
  degrades to the height test. Wood vs lab carpet differ by only 1–4 mm
  (under the 12 mm threshold), but carpet returns all 64 zones where wood
  loses the far row, so it detects slightly earlier.

### Settled hardware facts (do not re-derive)
- `base_joint` z = **0.0815 m**, caliper-measured 2026-09-13 (plate 27.5 mm off
  the floor). Sensor joints were adjusted to keep their measured floor heights.
- Rosmaster S1–S4 are PWM-only with **no read-back**; a bus servo there is
  unreachable. `Rosmaster_Lib.__arm_convert_angle` is calibrated for the YB-SD15M
  only. The Orin 40-pin GPIO is **not 5 V tolerant**.
- The 2 ms write gap in `Rosmaster_Lib` is load-bearing; don't delete it. The
  `/cmd_vel` watchdog stops the motors after 500 ms, so scripts must resend at 20 Hz.
- IMU heading comes from the **ICM-42688-P** on i2c-7 @ 0x68 (shares pins 3/5 with
  the INA226 @ 0x40), `use_external_imu_yaw: True`. Keep `aaf_bandwidth_hz` at 42,
  or wheel vibration aliases in. Its SPI path is defective on that breakout; don't retry it.
- The battery is a 4S **LiFePO4** 8.07 Ah pack. SoC is coulomb-counted because OCV
  is nearly flat.
- `ROS2Bridge.stop()` publishes a zero Twist directly and bypasses the CBF. Never
  route a stop through `move(0,0,0)`: the CBF's proactive repulsion can produce motion.
- The OLED is unplugged on purpose, so `OLED init failed 0x3C` at boot is expected.

## Key Configuration Files
- **[config/lidar_tilt_calibration_dynamixel.json](config/lidar_tilt_calibration_dynamixel.json)** — XL430 tilt zero (2032) + tolerance_counts (12). The older `lidar_tilt_calibration.json` is the retired LX-16A's and is dead.
- **[src/yahboomcar_nav/params/nav2_params_x3.yaml](src/yahboomcar_nav/params/nav2_params_x3.yaml)** — Nav2 planner/controller tuning (MPPI controller, named in the file header)
- **[src/yahboomcar_nav/params/ekf_x3.yaml](src/yahboomcar_nav/params/ekf_x3.yaml)** — EKF fusion config
- **[src/yahboomcar_nav/params/ydlidar_x3.yaml](src/yahboomcar_nav/params/ydlidar_x3.yaml)** — YDLidar driver params
- **[src/yahboomcar_nav/params/slam_toolbox_params.yaml](src/yahboomcar_nav/params/slam_toolbox_params.yaml)** — SLAM Toolbox config

## Web GUI
- **[src/web/GUI.html](src/web/GUI.html)** — Single-page app: joystick control, camera feed, lidar visualization, SLAM map canvas, Nav2 goal clicking, telemetry display. Served via HTTP :8080 (required so Web Workers can load `lidar-worker.js`).
- **[src/web/main.js](src/web/main.js)** — WebSocket client. On the robot the camera feed is WebRTC, not this path: see memory note `project_gui_black_camera.md` ("The robot runs `--webrtc-camera`"). The binary-WebSocket JPEG path is the fallback.
- **[src/web/lidar-worker.js](src/web/lidar-worker.js)** — Web Worker that decodes and transforms lidar point arrays off the main thread.

## Maps
Saved maps are stored as `.pgm`/`.yaml` pairs in `src/yahboomcar_nav/maps/`. New maps are created via the "Start SLAM" button in the GUI and saved through the server's map-save handler.

## Performance Notes (from notes/plans/PERFORMANCE_PLAN.md)
- Camera frame encoding (`cv2.imencode`) runs in a thread executor to avoid blocking the async event loop.
- JPEG fallback path only (the robot feed is WebRTC, see Web GUI above): JPEG frames are sent as **binary WebSocket messages** (not base64 JSON) to save 2–5 ms/frame.
- Lidar point conversion uses vectorized numpy operations.
- Battery voltage is read at 1 Hz (cached), not per-loop.
