# Yahboom X3 — ROS2 Workspace

WebSocket-based control server and ROS2 navigation stack for the **Yahboom X3 mecanum-wheel robot** running on a **Jetson Orin Nano (JetPack 6.2 / Ubuntu 22.04)**.

---

## Architecture

```
Laptop browser ──── WebSocket (port 8081) ────► server_x3.py  (Jetson)
                                                      │
                                         ┌────────────┴────────────┐
                                  direct serial            ROS2 DDS (--ros2)
                                         │                         │
                               Rosmaster + YDLidar        /cmd_vel  /scan  /odom
                                                                    │
                                                 ┌──────────────────┴──────────────────┐
                                           Jetson (physical)               Laptop (sim)
                                           x3_bringup.launch.py            x3_gazebo.launch.py
                                           (Mcnamu_driver + EKF + lidar)   (Ignition Fortress)
```

| Mode | Flag | Runs on | ROS2 topics come from |
|------|------|---------|----------------------|
| **Direct hardware** | *(none)* | Jetson | — (direct serial) |
| **Physical robot** | `--ros2` | Jetson | `jetson_bringup.sh` (local) |
| **Cross-machine sim** | `--ros2 --domain-id N` | Jetson | `laptop_sim.sh` on laptop |
| **Laptop-only sim** | `--sim` | Laptop | Gazebo (via 🚀 button) |

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
  ros-humble-tf2-ros
```

### Python (server_x3.py dependencies)

```bash
pip install websockets opencv-python ultralytics numpy
# On Jetson — also install the Yahboom Rosmaster library:
cd ~/Downloads/X3-ROS2-source_code/For\ jetson\ orin\ super/yahboomcar_ros2_ws/software/py_install_V3.3.1
sudo python3 setup.py install
```

### YDLidar C++ SDK (required before colcon build)

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git ~/YDLidar-SDK
cd ~/YDLidar-SDK && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig
```

---

## Build

```bash
cd ~/x3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  yahboomcar_msgs \
  yahboomcar_description \
  yahboomcar_base_node \
  yahboomcar_bringup \
  yahboomcar_nav \
  ydlidar_ros2_driver
source install/setup.bash
```

> **Tip:** If colcon picks up miniconda's Python and fails on `catkin_pkg` or `empy`, run:
> ```bash
> pip install catkin_pkg "empy==3.3.4" lark
> ```
> Then retry the build. See [scripts/build_ros2.sh](scripts/build_ros2.sh) for a one-shot setup script.

---

## Running

The server **always runs on the Jetson** (robot). The browser connects from your laptop.
Gazebo simulation also runs on the laptop — both machines communicate via ROS2 DDS over the LAN.

### Option A — Direct hardware mode (no ROS2 stack)

Fastest mode: server talks directly to the Rosmaster serial board and YDLidar.

```bash
# Jetson
cd ~/x3_ws/src
python3 server_x3.py
```

Open `http://<jetson-ip>:8081` in your browser.

### Option B — Physical robot with ROS2 stack (SLAM / Nav2)

Full ROS2 stack for SLAM, EKF odometry, and Nav2 autonomous navigation.

```bash
# Jetson — Terminal 1: hardware drivers + EKF + lidar
bash ~/x3_ws/scripts/jetson_bringup.sh

# Jetson — Terminal 2: server
cd ~/x3_ws/src
python3 server_x3.py --ros2 --domain-id 42
```

Open `http://<jetson-ip>:8081`. Use the GUI **Start SLAM** button to begin mapping.

### Option C — Cross-machine simulation (Gazebo on laptop, server on Jetson)

Gazebo runs on your laptop and publishes the same ROS2 topics that the physical robot
hardware would. The server on the Jetson receives them transparently via DDS — the GUI
experience is identical to Option B.

**Requirements:**
- Both machines on the same LAN (UDP multicast must not be blocked)
- Same `ROS_DOMAIN_ID` on both machines (the scripts default to `42`)
- ROS2 Humble and this workspace built on the laptop (`bash scripts/build_ros2.sh`)
- Ignition Fortress on the laptop: `sudo apt install ros-humble-ros-gz`

```bash
# Laptop — Gazebo simulation
bash ~/x3_ws/scripts/laptop_sim.sh

# Jetson — server (same domain ID)
cd ~/x3_ws/src
python3 server_x3.py --ros2 --domain-id 42
```

Open `http://<jetson-ip>:8081`. The GUI connects to the Jetson server; the Jetson's
ROS2Bridge receives topics from Gazebo on the laptop via DDS discovery.

> **Tip:** Both scripts accept an optional domain ID argument:
> `bash scripts/laptop_sim.sh 7`  /  `python3 server_x3.py --ros2 --domain-id 7`
> They must match. Any integer 0–101 works; avoid 0 on a busy network.

### Option D — Laptop-only simulation (dev mode, no Jetson needed)

Run everything on the laptop — useful for pure GUI / logic development without any robot.

```bash
# Laptop
cd ~/x3_ws/src
python3 server_x3.py --sim
# Then click 🚀 Gazebo in the browser header to launch Gazebo
```

---

## GUI Feature Availability by Mode

| Feature | Direct hardware | `--ros2` | `--sim` |
|---------|----------------|----------|---------|
| Camera feed (RGB) | ✅ Astra Pro | ✅ Astra Pro | ❌ |
| Depth toggle | ✅ (if depth enabled) | ✅ (if depth enabled) | ❌ |
| Lidar view | ✅ YDLidar | ✅ via `/scan` topic | ❌ |
| Motor control | ✅ direct serial | ✅ via `/cmd_vel` | ✅ (no-op) |
| YOLO detection | ✅ | ✅ | ❌ |
| Power / battery | ✅ | ✅ | ❌ |
| SLAM map (RViz) | ❌ | ✅ | ✅ (Gazebo) |

> **Depth toggle**: Requires the `enable_depth=True` flag in `AstraCamera`. Currently disabled by default — enable it in `initialize_hardware()` if your Astra Pro depth stream is needed.

---

## systemd Service (auto-start on boot)

```bash
sudo cp ~/x3_ws/src/x3_server.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable x3_server
sudo systemctl start x3_server

# View logs
journalctl -u x3_server -f
```

### Switching modes without editing the service file

Create a drop-in override:

```bash
sudo mkdir -p /etc/systemd/system/x3_server.service.d/
sudo nano /etc/systemd/system/x3_server.service.d/override.conf
```

Contents for ROS2 bridge mode:
```ini
[Service]
Environment=SERVER_ARGS=--ros2
```

Apply:
```bash
sudo systemctl daemon-reload && sudo systemctl restart x3_server
```

---

## Key Hardware Parameters (YDLidar X3)

| Parameter | Value |
|-----------|-------|
| Port | `/dev/ttyUSB0` |
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
│   ├── server_x3.py          # Main WebSocket server
│   ├── drivers_x3.py         # Camera, lidar, motor drivers
│   ├── x3_server.service     # systemd unit file
│   ├── web/                  # Browser GUI (HTML/CSS/JS)
│   ├── yahboomcar_base_node/ # ROS2: dead-reckoning odometry
│   ├── yahboomcar_bringup/   # ROS2: hardware bringup launch
│   ├── yahboomcar_description/ # URDF / robot model
│   ├── yahboomcar_msgs/      # Custom ROS2 message types
│   ├── yahboomcar_nav/       # SLAM + Nav2 launch & params
│   ├── yahboomcar_rviz/      # RViz configs
│   └── ydlidar_ros2_driver/  # YDLidar ROS2 node
├── scripts/
│   └── build_ros2.sh         # One-shot dependency + build script
├── PERFORMANCE_PLAN.md       # Optimization notes
└── README.md                 # This file
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
```

---

## SLAM Toolbox Mapping Workflow

The X3 uses [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) for real-time lidar-based
mapping. The resulting map can be saved and immediately used for autonomous navigation with Nav2.

### Via the GUI (recommended)

1. Launch the server in the appropriate mode:
   ```bash
   # Simulation
   python3 src/server_x3.py --sim

   # Physical robot
   python3 src/server_x3.py --ros2
   ```
2. In the Navigation card, click **Start SLAM**. The status text will confirm mapping is active.
3. Drive the robot around (use the joystick or WASD keys) to build the map.
4. When coverage is complete, type a name in the map name field and click **Save Map**.
5. The new map will appear in the **Map** dropdown in the Navigation card.
6. To use the map for autonomous navigation: select it, click **Launch Nav2**, then set an
   initial pose and click a goal on the map canvas.
7. Click **Stop SLAM** when done mapping (frees CPU for navigation).

### Via the CLI

```bash
# Terminal 1 — Gazebo (sim only)
ros2 launch yahboomcar_nav x3_gazebo.launch.py

# Terminal 2 — SLAM Toolbox (sim)
ros2 launch yahboomcar_nav x3_slam_sim.launch.py

# Terminal 2 — SLAM Toolbox (physical robot)
ros2 launch yahboomcar_nav x3_slam.launch.py

# Save the finished map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  '{name: {data: "/home/kamren/x3_ws/src/yahboomcar_nav/maps/mymap"}}'
```

Maps are saved as `<name>.pgm` + `<name>.yaml` pairs in
`src/yahboomcar_nav/maps/` and are automatically discovered by the server.

---

## Future Roadmap

### Trajectory Smoother: SavitzkyGolay → ConstrainedSmoother

The current smoother (`SavitzkyGolayFilterSmoother`) post-processes the global path with a
fixed-window polynomial filter. It produces smooth trajectories but has no awareness of the
costmap, so it can occasionally clip corners near obstacles.

The planned upgrade is to **`ConstrainedSmootherServer`**, which optimises path curvature as a
constrained minimisation problem against the costmap. Benefits:

- Paths respect robot footprint and inflation layer throughout
- Better clearance in narrow corridors
- Tunable cost weights (path length vs curvature vs clearance)

Configuration location: `src/yahboomcar_nav/params/nav2_params_x3.yaml` — `smoother_server` block.

### Mapping: SLAM Toolbox → RTAB-Map

The current pipeline uses SLAM Toolbox (lidar-only 2D occupancy mapping). The planned upgrade is
**RTAB-Map**, which fuses the Orbbec Astra Pro depth camera with the YDLidar. Benefits:

- **Visual loop closure** — camera-based place recognition is far more distinctive than
  lidar scan-shape matching, drastically reducing drift over large areas or after long loops.
- **Optional 3D reconstruction** — point cloud / OctoMap alongside the 2D occupancy grid for
  richer environment understanding.
- **Improved re-localisation** — RTAB-Map's appearance-based memory is more robust when
  returning to previously-visited areas after a long absence.

Infrastructure already present in the repository:
- `navigation_rtabmap_launch.py`
- `rtabmap_nav_params.yaml`
