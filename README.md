# Yahboom X3 — ROS2 Workspace

WebSocket-based control server and ROS2 navigation stack for the **Yahboom X3 mecanum-wheel robot** running on a **Jetson Orin Nano (JetPack 6.2 / Ubuntu 22.04)**.

---

## Architecture

```
Browser GUI (web/)
      │  WebSocket (port 8765)
      ▼
server_x3.py  ──── direct serial ──► Rosmaster (motors) + YDLidar
              └─── --ros2 flag  ──► /cmd_vel topic  (ROS2 bridge)
                                    /scan   topic   (ROS2 bridge)
                                         ▲
                              ros2_bringup launch
                           (Mcnamu_driver + base_node + EKF + lidar)
```

Two run modes:

| Mode | Flag | When to use |
|------|------|-------------|
| **Direct hardware** | *(none)* | Standalone operation, no ROS2 stack |
| **ROS2 bridge** | `--ros2` | Nav2 / SLAM running; server talks over topics |
| **Simulation** | `--sim` | No hardware present (dev/testing) |

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

### Option A — Direct hardware mode (no ROS2 stack)

```bash
cd ~/x3_ws/src
python3 server_x3.py
```

Open `http://<robot-ip>:8765` in your browser (or serve `src/web/` via any HTTP server).

### Option B — ROS2 bridge mode (SLAM / Nav2)

**Terminal 1** — start the ROS2 hardware + SLAM stack:
```bash
source /opt/ros/humble/setup.bash
source ~/x3_ws/install/setup.bash
ros2 launch yahboomcar_nav x3_slam.launch.py
```

**Terminal 2** — start the server in ROS2 bridge mode:
```bash
source /opt/ros/humble/setup.bash
source ~/x3_ws/install/setup.bash
cd ~/x3_ws/src
python3 server_x3.py --ros2
```

### Option C — Simulation / Gazebo

The `--ros2` flag is inherently Gazebo-compatible. Gazebo publishes to the same `/scan` and `/cmd_vel` topics, so:

```bash
# Terminal 1 — Gazebo with robot spawned
ros2 launch yahboomcar_nav x3_slam.launch.py use_sim_time:=true

# Terminal 2 — server in ROS2 bridge mode
python3 server_x3.py --ros2
```

Motor commands and lidar data will be reflected in the GUI in real time.

### Simulation without any hardware (dev mode)

```bash
python3 server_x3.py --sim
```

No serial port or camera needed. Returns dummy data so the GUI layout and logic can be tested.

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
