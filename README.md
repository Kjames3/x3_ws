# Yahboom X3 — ROS2 Workspace

WebSocket-based control server and ROS2 navigation stack for the **Yahboom X3 mecanum-wheel robot** running on a **Jetson Orin Nano (JetPack 6.2 / Ubuntu 22.04)**.

---

## Architecture

```
Laptop browser
  │
  ├── WebSocket (port 8081) ────► server_x3.py  (Jetson)
  │                                     │
  │                        ┌────────────┴────────────┐
  │                 direct serial            ROS2 DDS
  │                        │                         │
  │              Rosmaster + YDLidar        /cmd_vel  /odom  /map
  │                                         /camera   /voltage
  │
  └── WebSocket (port 8765) ────► foxglove_bridge  (Jetson)
                                        │
                                  /map  /scan  (raw ROS2 messages,
                                               JSON-encoded)
```

### What each connection carries

| Connection | Port | Handles |
|---|---|---|
| `server_x3.py` | 8081 | Camera JPEG, YOLO detections, battery/power, motor commands, SLAM lifecycle, map save/load |
| `foxglove_bridge` | 8765 | `/map` OccupancyGrid (live SLAM map), `/scan` LaserScan (full lidar scan) |

The browser opens both connections automatically on connect. The Foxglove connection is non-fatal — camera, motors, and YOLO continue working if the bridge is not running.

---

## Prerequisites

### System (Ubuntu 22.04 / JetPack 6.2)

```bash
sudo apt update
sudo apt install \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-imu-filter-madgwick \
  ros-humble-tf2-ros \
  ros-humble-foxglove-bridge
```

### Python (server_x3.py dependencies)

```bash
pip install websockets opencv-python ultralytics numpy
# On Jetson — also install the Yahboom Rosmaster library:
cd ~/Downloads/X3-ROS2-source_code/For\ jetson\ orin\ super/yahboomcar_ros2_ws/software/py_install_V3.3.1
sudo python3 setup.py install
```

### YDLidar C++ SDK (required before colcon build)

Handled automatically by `scripts/install.sh`. To install manually:

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git ~/YDLidar-SDK
cd ~/YDLidar-SDK && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig
```

---

## Setup (first time)

Run the setup script from the workspace root. It installs all dependencies, builds the workspace, writes udev rules, and configures `.bashrc`:

```bash
cd ~/x3_ws
bash scripts/install.sh
# After completion:
sudo reboot
```

The script handles:
- APT system libraries and ROS2 Humble packages
- Python dependencies (ultralytics, websockets, etc.)
- YDLidar C++ SDK build and install
- Git submodule init (`ydlidar_ros2_driver`)
- udev rules (YDLidar, Rosmaster, Orbbec camera symlinks)
- User group membership (dialout, i2c, video, plugdev)
- Full `colcon build --symlink-install`
- `.bashrc` sourcing and aliases (`cb`, `cs`)

Pass `--sim` to also install `ros-humble-ros-gz` (Gazebo simulation):

```bash
bash scripts/install.sh --sim
```

---

## Build

```bash
cd ~/x3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  yahboomcar_msgs \
  yahboomcar_description \
  yahboomcar_base_node \
  yahboomcar_bringup \
  yahboomcar_nav \
  ydlidar_ros2_driver
source install/local_setup.bash
```

> **Note:** With `--symlink-install`, changes to Python files, launch files, and YAML params take effect immediately without rebuilding. Only C++ packages (`yahboomcar_base_node`, `ydlidar_ros2_driver`) require a rebuild when their source changes.

> **Tip:** If colcon picks up miniconda's Python and fails on `catkin_pkg` or `empy`, run:
> ```bash
> pip install catkin_pkg "empy==3.3.4" lark
> ```

---

## Running

### Option A — Direct hardware mode (no ROS2 stack)

Fastest mode: server talks directly to the Rosmaster serial board and YDLidar.

```bash
# Jetson
cd ~/x3_ws/src
python3 server_x3.py
```

Open `http://<jetson-ip>:8080/GUI.html` in your browser.

### Option B — Physical robot with ROS2 stack (SLAM / Nav2)

Full ROS2 stack for SLAM, EKF odometry, Nav2 autonomous navigation, and Foxglove bridge.

```bash
# Jetson — Terminal 1: hardware drivers + EKF + lidar
bash ~/x3_ws/scripts/jetson_bringup.sh

# Jetson — Terminal 2: WebSocket control server
cd ~/x3_ws/src
python3 server_x3.py --domain-id 42

# Jetson — Terminal 3: Foxglove bridge (map + lidar to browser)
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge.launch.xml \
  port:=8765 address:=0.0.0.0 send_buffer_limit:=10000000
```

Open `http://<jetson-ip>:8080/GUI.html`. Use the GUI **Start SLAM** button to begin mapping.

### Option C — Cross-machine simulation (Gazebo on laptop, server on Jetson)

```bash
# Laptop — Gazebo simulation
bash ~/x3_ws/scripts/laptop_sim.sh

# Jetson — server
cd ~/x3_ws/src
python3 server_x3.py --domain-id 42
```

### Option D — Laptop-only simulation (dev mode, no Jetson needed)

```bash
# Laptop
cd ~/x3_ws/src
python3 server_x3.py --sim
# Then click 🚀 Gazebo in the browser header to launch Gazebo
```

---

## GUI Feature Availability by Mode

| Feature | Direct hardware | ROS2 mode | `--sim` |
|---------|----------------|-----------|---------|
| Camera feed (RGB) | ✅ Astra Pro | ✅ Astra Pro | ❌ |
| Depth toggle | ✅ (if enabled) | ✅ (if enabled) | ❌ |
| Lidar view | ✅ YDLidar (serial) | ✅ via Foxglove `/scan` | ❌ |
| SLAM map | ❌ | ✅ via Foxglove `/map` | ✅ (Gazebo) |
| Motor control | ✅ direct serial | ✅ via `/cmd_vel` | ✅ (no-op) |
| YOLO detection | ✅ | ✅ | ❌ |
| Power / battery | ✅ | ✅ | ❌ |
| Nav2 navigation | ❌ | ✅ | ✅ (Gazebo) |

> **Depth toggle**: Requires `enable_depth=True` in `AstraCamera`. Disabled by default — enable in `initialize_hardware()` if needed.

> **Lidar / Map in ROS2 mode**: Requires `foxglove_bridge` to be running on port 8765. The GUI connects automatically and retries with exponential backoff if the bridge is temporarily unavailable.

---

## systemd Services (auto-start on boot)

Two services run on the Jetson: the control server and the Foxglove bridge.

### Install both services

```bash
sudo cp ~/x3_ws/src/x3_server.service /etc/systemd/system/
sudo cp ~/x3_ws/src/foxglove_bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable x3_server foxglove_bridge
sudo systemctl start x3_server foxglove_bridge
```

### View logs

```bash
journalctl -u x3_server -f
journalctl -u foxglove_bridge -f
```

### Service startup sequence

```
Boot
 └─ x3_server        starts after 10 s (hardware enumeration)
     └─ foxglove_bridge  starts after 15 s (waits for ROS2 stack)
```

### Switching server mode without editing the service file

Create a drop-in override:

```bash
sudo mkdir -p /etc/systemd/system/x3_server.service.d/
sudo nano /etc/systemd/system/x3_server.service.d/override.conf
```

Contents for simulation mode:
```ini
[Service]
Environment="SERVER_ARGS=--sim"
```

Apply:
```bash
sudo systemctl daemon-reload && sudo systemctl restart x3_server
```

### Desktop vs. headless boot

Disabling the desktop (GNOME) frees ~300–500 MB RAM and 5–15% CPU — recommended for robot operation:

```bash
# Disable desktop (takes effect on next boot)
sudo systemctl set-default multi-user.target

# Re-enable desktop temporarily (current session only)
sudo systemctl start gdm3

# Re-enable desktop permanently
sudo systemctl set-default graphical.target
```

### Jetson performance mode

```bash
sudo nvpmodel -m 0      # max power mode (persists across reboots)
sudo jetson_clocks      # lock clocks to max (current session)
```

---

## Key Hardware Parameters (YDLidar X3)

| Parameter | Value |
|-----------|-------|
| Port | `/dev/ttyUSB0` (symlink: `/dev/ydlidar`) |
| Baud rate | 512000 |
| Lidar type | `TYPE_TOF` (0) |
| Single channel | `true` |
| Sample rate | 20 kHz |
| Scan frequency | 8 Hz |
| Frame ID | `laser_link` |

---

## Project Structure

```
x3_ws/
├── src/
│   ├── server_x3.py              # Main WebSocket server (port 8081)
│   ├── drivers_x3.py             # Camera, lidar, motor drivers
│   ├── nav2_client.py            # Nav2 action client
│   ├── navigation_fsm.py         # YOLO-driven target tracking FSM
│   ├── frontier_explorer.py      # Autonomous frontier exploration
│   ├── trt_detector.py           # TensorRT YOLO wrapper
│   ├── oakd_driver.py            # OAK-D Lite DepthAI driver (stereo/depth/IMU)
│   ├── oakd_ros_publisher.py     # Republishes OAK-D streams as /oak/* ROS2 topics
│   ├── Rosmaster_Lib.py          # Yahboom serial protocol library
│   ├── x3_server.service         # systemd: control server
│   ├── foxglove_bridge.service   # systemd: Foxglove WebSocket bridge
│   ├── web/
│   │   ├── GUI.html              # Single-page browser GUI
│   │   ├── main.js               # WebSocket client + Foxglove client
│   │   └── lidar-worker.js       # Web Worker: lidar canvas rendering
│   ├── yahboomcar_base_node/     # ROS2 C++: dead-reckoning odometry
│   ├── yahboomcar_bringup/       # ROS2: hardware bringup launch
│   ├── yahboomcar_description/   # URDF / robot model
│   ├── yahboomcar_msgs/          # Custom ROS2 message types
│   ├── yahboomcar_nav/           # SLAM + Nav2 launch & params
│   ├── yahboomcar_rviz/          # RViz configs
│   └── ydlidar_ros2_driver/      # YDLidar ROS2 driver (git submodule)
├── scripts/
│   ├── install.sh                # Full first-time setup script
│   └── build_ros2.sh             # Workspace build + YDLidar SDK
├── record_bag.sh                 # Domain-adaptation rosbag recording
└── README.md
```

---

## Useful Commands

```bash
# Check which nodes are running
ros2 node list

# Monitor lidar scan
ros2 topic echo /scan --once

# Monitor odometry
ros2 topic echo /odom

# Send a test velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# View the map in RViz
ros2 run rviz2 rviz2

# Save the current SLAM map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'my_map'}}"

# Check Foxglove bridge topics
ros2 topic list | grep -E "scan|map|odom"
```

---

## SLAM Toolbox Mapping Workflow

The X3 uses [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) for real-time lidar-based mapping. The map is streamed live to the browser via the Foxglove bridge.

### Via the GUI (recommended)

1. Start the server and Foxglove bridge (or use systemd auto-start).
2. In the Navigation card, click **Start SLAM**. The live map will appear in the canvas within ~1 second.
3. Drive the robot around (joystick or WASD) to build the map.
4. Type a name in the map name field and click **Save Map**.
5. To use the map for autonomous navigation: select it from the dropdown, click **Launch Nav2**, set an initial pose, then click a goal on the map canvas.
6. Click **Stop SLAM** when done (frees CPU for navigation).

### Via the CLI

```bash
# SLAM Toolbox (physical robot)
ros2 launch yahboomcar_nav x3_slam.launch.py

# SLAM Toolbox (simulation)
ros2 launch yahboomcar_nav x3_slam_sim.launch.py

# Save the finished map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  '{name: {data: "/home/jetson/x3_ws/src/yahboomcar_nav/maps/mymap"}}'
```

Maps are saved as `<name>.pgm` + `<name>.yaml` pairs in `src/yahboomcar_nav/maps/` and are automatically discovered by the server.

---

## Rosbag Data Collection (OAK-D Lite)

`record_bag.sh` records a rosbag for domain-adaptation training — the robot is teleoperated while people walk around it.

### Why the OAK-D needs a server flag

The OAK-D Lite is driven **in-process** by `src/oakd_driver.py` inside `server_x3.py`. DepthAI opens the device **exclusively**, so no separate ROS2 node can ever open the camera — which means the OAK streams are invisible to `ros2 bag record` by default.

`src/oakd_ros_publisher.py` bridges this: it polls the driver's cached frames from a background thread and republishes them as standard `sensor_msgs`. It is **off unless the server is started with `--oak-ros-publish`**, because publishing costs real CPU and bandwidth.

| Topic | Type | Notes |
|---|---|---|
| `/oak/depth/image_raw` | `sensor_msgs/Image` | 16UC1, millimetres — same convention as the Astra |
| `/oak/depth/camera_info` | `sensor_msgs/CameraInfo` | CAM_A intrinsics, required to reproject depth |
| `/oak/left/image_raw` | `sensor_msgs/Image` | mono8 |
| `/oak/right/image_raw` | `sensor_msgs/Image` | mono8 |
| `/oak/imu` | `sensor_msgs/Imu` | accel + gyro; no fused orientation (`orientation_covariance[0] = -1`) |
| `/oak/detections` | `vision_msgs/Detection3DArray` | on-device YOLO spatial detections |

Frame ids follow the URDF: `oak_rgb_camera_optical_frame` (depth is aligned to CAM_A), `oak_left/right_camera_optical_frame`, `oak_imu_frame`.

Server flags:

| Flag | Default | Effect |
|---|---|---|
| `--oak-ros-publish` | off | Enable `/oak/*` publishing |
| `--oak-ros-rate N` | 10 | Publish rate in Hz |
| `--oak-ros-no-stereo` | off | Skip mono L/R (keeps depth + IMU + detections) |

### Enabling recording mode

The `x3_server` unit takes a `$SERVER_ARGS` environment variable. Add a drop-in — `30-` sorts after the existing `20-webrtc.conf`, so it wins:

```bash
sudo tee /etc/systemd/system/x3_server.service.d/30-oak-record.conf >/dev/null <<'EOF'
[Service]
Environment="SERVER_ARGS=--domain-id 42 --webrtc-camera --oak-ros-publish"
EOF
sudo systemctl daemon-reload && sudo systemctl restart x3_server
```

> **Verify it actually took.** A successful `systemctl restart` does **not** prove the drop-in was installed. Check the live process:
> ```bash
> tr '\0' ' ' < /proc/$(systemctl show x3_server -p MainPID --value)/cmdline
> ```
> `--oak-ros-publish` must appear. Allow ~40–90 s for the stack to come up, then confirm:
> ```bash
> journalctl -u x3_server | grep OakRosPublisher
> # OakRosPublisher: publishing /oak/* at 10 Hz (stereo on, detections on)
> ```

**Remove the drop-in when the recording session is over** — otherwise the server keeps paying for publishing forever:

```bash
sudo rm /etc/systemd/system/x3_server.service.d/30-oak-record.conf
sudo systemctl daemon-reload && sudo systemctl restart x3_server
```

### Recording

```bash
cd ~/x3_ws
RECORD_OAK_STEREO=true ./record_bag.sh              # stereo + depth + IMU
./record_bag.sh                                     # depth + IMU only
./record_bag.sh /path/to/output_dir                 # custom output dir
```

> ⚠️ **Options are environment variables, not arguments.** The variable must come *before* the script name, with no space around `=`. Running `./record_bag.sh RECORD_OAK_STEREO=true` silently creates an output **directory named** `RECORD_OAK_STEREO=true` and records with stereo **off**.

| Variable | Default | Effect |
|---|---|---|
| `RECORD_OAK_STEREO` | `false` | Also record `/oak/left` + `/oak/right` |
| `RECORD_ASTRA_DEPTH` | `false` | Also record the Astra's `/camera/depth/image_raw` |
| `RECORD_DURATION` | (unset) | Auto-stop after N seconds — use for trial runs |
| `BYPASS_TOPIC_CHECK` | `false` | Skip the pre-flight publisher check |

Always recorded: the three `/oak` core topics plus `/scan`, `/odom`, `/tf`, `/tf_static`. Default output is `~/bags/domain_adapt/`. Press **Ctrl+C** to stop (or let `RECORD_DURATION` end it).

The script auto-detects the DDS configuration from the **live `server_x3.py` process environment** — adopting a discovery server only if the robot actually uses one, and plain multicast otherwise. Do not hardcode `ROS_DISCOVERY_SERVER`: pointing the recorder at a discovery server nothing else uses produces a bag with zero messages.

A pre-flight check aborts if a required topic has no publisher, so you find out before recording rather than after. Optional topics only warn.

**Sizing** (measured, 10 Hz, ~8.9 Hz achieved): roughly **211 MB/min with stereo** (~6.3 GB per 30 min), about half that depth-only. Check free space before a long session.

### Reading a recorded bag

Bags are zstd file-compressed, so `rosbag2_py` needs the decompressing reader:

```python
from rosbag2_py import SequentialCompressionReader, StorageOptions, ConverterOptions
reader = SequentialCompressionReader()      # NOT SequentialReader
reader.open(StorageOptions(uri=bag_path, storage_id="sqlite3"),
            ConverterOptions("cdr", "cdr"))
```

> Reading decompresses a `.db3` alongside the `.zstd`. The `.zstd` is what `metadata.yaml` references — delete the loose `.db3` afterwards to reclaim the space.

> **On `/oak/detections`:** the on-device YOLO works (verified detecting `person` at 2.8–4.2 m), but in testing it fired on only ~12% of frames at confidence 0.37–0.43 against a 0.35 threshold. Treat it as a bonus label track, not reliable training ground truth. Depth, stereo and IMU are the dependable streams.

---

## Future Roadmap

### Trajectory Smoother: SavitzkyGolay → ConstrainedSmoother

The current smoother (`SavitzkyGolayFilterSmoother`) post-processes the global path with a fixed-window polynomial filter. It produces smooth trajectories but has no awareness of the costmap, so it can occasionally clip corners near obstacles.

The planned upgrade is **`ConstrainedSmootherServer`**, which optimises path curvature as a constrained minimisation problem against the costmap. Benefits:

- Paths respect robot footprint and inflation layer throughout
- Better clearance in narrow corridors
- Tunable cost weights (path length vs curvature vs clearance)

Configuration: `src/yahboomcar_nav/params/nav2_params_x3.yaml` — `smoother_server` block.

### Mapping: SLAM Toolbox → RTAB-Map

The current pipeline uses SLAM Toolbox (lidar-only 2D occupancy mapping). The planned upgrade is **RTAB-Map**, which fuses the Orbbec Astra Pro depth camera with the YDLidar. Benefits:

- **Visual loop closure** — camera-based place recognition is far more distinctive than lidar scan-shape matching, drastically reducing drift over large areas or after long loops.
- **Optional 3D reconstruction** — point cloud / OctoMap alongside the 2D occupancy grid.
- **Improved re-localisation** — RTAB-Map's appearance-based memory is more robust when returning to previously-visited areas after a long absence.

Infrastructure already present: `navigation_rtabmap_launch.py`, `rtabmap_nav_params.yaml`.

### TF / Odometry via Foxglove

The Foxglove bridge also exposes `/tf`. A future enhancement would subscribe to `/tf` client-side and use the `map → base_footprint` transform for robot pose overlay on the map canvas, replacing the current `/odom`-based pose estimate.
