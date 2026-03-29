"""
Viam Rover Control Server - YAHBOOM X3 (Jetson Orin)

This WebSocket server specifically controls the Yahboom X3 robot.
Features:
- 4-Wheel Mecanum Drive (Holonomic)
- Serial Communication with ROS Controller Board
- YDLidar 4ROS Support
- Orbbec Astra Pro (RGB + Depth) Support
- YOLOv11 Detection
"""

import asyncio
import time
import json
import logging
import argparse
import base64
import numpy as np
import cv2
import websockets
import sys
import os
import signal
import threading
import socket
import subprocess
import yaml
try:
    import torch
    _TORCH_AVAILABLE = True
except ImportError:
    _TORCH_AVAILABLE = False

# Ultralytics SAM3 does a bare `import torchvision` at module load time.
# Mock it if the real package isn't installed so the server still starts.
try:
    import torchvision  # noqa: F401
except ModuleNotFoundError:
    import sys
    from unittest.mock import MagicMock
    _tv = MagicMock()
    _tv.__version__ = "0.15.0"
    sys.modules["torchvision"]            = _tv
    sys.modules["torchvision.ops"]        = MagicMock()
    sys.modules["torchvision.transforms"] = MagicMock()
    sys.modules["torchvision.models"]     = MagicMock()

from ultralytics import YOLO


# =============================================================================
# LOGGING CONFIGURATION
# =============================================================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Import X3 specific drivers
from drivers_x3 import (
    Rosmaster, MecanumDrive, YDLidarDriver, AstraCamera, OLEDDisplay, SERIAL_PORT
)
from nav2_client import Nav2Client
from frontier_explorer import FrontierExplorer

# =============================================================================
# CONFIGURATION
# =============================================================================
parser = argparse.ArgumentParser(description='Yahboom X3 Control Server')
parser.add_argument('--sim', action='store_true', help='Run in simulation mode (laptop, Gazebo on demand)')
parser.add_argument('--ros2', action='store_true',
                    help='ROS2 bridge mode: subscribe /scan and /cmd_vel via rclpy. '
                         'Works with local hardware bringup OR remote Gazebo on the laptop.')
parser.add_argument('--domain-id', type=int, default=None, dest='domain_id',
                    help='ROS_DOMAIN_ID for multi-machine ROS2 (must match laptop). '
                         'Overrides the ROS_DOMAIN_ID environment variable.')
args = parser.parse_args()
SIM_MODE   = args.sim
ROS2_MODE  = args.ros2

# Apply ROS_DOMAIN_ID early — must be set before rclpy.init() inside ROS2Bridge
if args.domain_id is not None:
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)

# Hardware Ports
# SERIAL_PORT auto-detected in drivers_x3 (/dev/ttyCH341USB0 or /dev/ttyUSB0)
LIDAR_PORT = "/dev/ttyUSB0"   # YDLidar (ROSMASTER is on ttyCH341USB0, so USB0 is free)

# Detection Config
MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yolo_models')

def find_model_path(model_name):
    # Search recursively for {model_name}.pt
    for root, _, files in os.walk(MODELS_DIR):
        if f"{model_name}.pt" in files:
            return os.path.join(root, f"{model_name}.pt")
    # Fallback default
    return os.path.join(MODELS_DIR, f"{model_name}.pt")

YOLO_MODEL = find_model_path("yolo11n_cans")
CONFIDENCE_THRESHOLD = 0.25
INFERENCE_SIZE = 640

# WebSocket port (must match GUI DEFAULT_PORT)
WS_PORT = 8081

# =============================================================================
# GLOBAL STATE
# =============================================================================
ros_board = None
ros_bridge = None   # ROS2Bridge instance when --ros2 or --sim; used for map streaming
drive = None
lidar = None
camera = None
model = None
oled = None
nav2_client = None    # Nav2Client instance (ROS2/sim modes only)
frontier_explorer = None  # FrontierExplorer instance (ROS2 mode only)
_gazebo_proc    = None   # subprocess handle when --sim auto-launches Gazebo
_ros2_stack_proc = None  # subprocess handle when --ros2 auto-launches x3_bringup
_slam_proc      = None   # subprocess handle when SLAM Toolbox is running

detection_enabled = False
depth_enabled = False
lidar_enabled = False
is_auto_driving = False
last_detections = []
active_model_name = "yolo11n_cans"

# Current motor powers (tank-drive representation for GUI readout)
current_left_power = 0.0
current_right_power = 0.0

# FPS tracking
_cam_frame_count = 0
_yolo_frame_count = 0
_fps_last_time = time.time()
fps_camera = 0.0
fps_detection = 0.0

# Battery voltage cache (refreshed at 1 Hz, not every frame)
_batt_cache_v    = 12.0
_batt_cache_time = 0.0

connected_clients = set()

# =============================================================================
# MOTION PIPELINE
# =============================================================================
# Watchdog: stop motors if no command received within this many seconds.
# Normal joystick input arrives at 20-50 Hz so 500 ms is unambiguously a drop.
MOTION_WATCHDOG_TIMEOUT = 0.5

# asyncio.Queue for decoupled motion commands — created in main() so it runs
# inside the event loop.  Handlers enqueue (vx, vy, omega) tuples; motion_loop()
# drains the queue at 100 Hz and calls drive.move().
motion_queue = None

# =============================================================================
# INITIALIZATION
# =============================================================================

# =============================================================================
# ROS2 BRIDGE (--ros2 mode)
# Replaces serial lidar + serial motor control with rclpy topic I/O so that
# the YDLidar and Rosmaster serial ports are owned exclusively by the ROS2
# stack (ydlidar_ros2_driver + Mcnamu_driver_X3).
# =============================================================================
class ROS2Bridge:
    """
    Drop-in adapter that mimics YDLidarDriver.get_points_xy() and
    MecanumDrive.move() over ROS2 topics.

      /scan  (sensor_msgs/LaserScan) → get_points_xy()
      /cmd_vel (geometry_msgs/Twist) ← move(vx, vy, omega)

    Velocity scaling matches Mcnamu_driver_X3.py defaults:
      linear  max = 0.5 m/s  (Rosmaster ±1.0 → ±0.5 m/s)
      angular max = 2.0 rad/s (Rosmaster ±5.0  → GUI scale)
    """
    LINEAR_SCALE  = 1.0   # pass-through: matches MecanumDrive.move() direct-mode behaviour
    ANGULAR_SCALE = 1.0   # Mcnamu_driver_X3 applies no additional scaling

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan, Image
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry, OccupancyGrid
        from std_msgs.msg import Float32

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        rclpy.init(args=None)
        self._node = Node('x3_ws_bridge')
        self._lock = threading.Lock()
        self._points: list[float] = []
        self._latest_frame = None          # cv2 BGR ndarray from Gazebo RGB camera
        self._latest_depth = None          # cv2 BGR ndarray from Gazebo depth camera
        self._pose_m = {"x": 0.0, "y": 0.0, "theta": 0.0}  # metres + radians
        self._twist = {"vx": 0.0, "vy": 0.0, "wz": 0.0}   # body velocity m/s, rad/s
        self._voltage = 12.0               # volts, updated by /voltage subscriber
        self._occupancy_grid: dict | None = None  # latest OccupancyGrid info dict for frontier explorer
        self._map_dirty = False

        # YDLidar publishes /scan as BEST_EFFORT; default (RELIABLE) causes a QoS mismatch
        _scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # SLAM Toolbox publishes /map as TRANSIENT_LOCAL; match so late-joining still gets the map
        _map_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._node.create_subscription(LaserScan,      '/scan',             self._scan_cb,  _scan_qos)
        self._node.create_subscription(Image,          '/camera/image_raw', self._image_cb,  1)
        self._node.create_subscription(Image,          '/camera/depth_image', self._depth_cb, 1)
        self._node.create_subscription(Odometry,       '/odom',             self._odom_cb,  10)
        self._node.create_subscription(Float32,        '/voltage',          self._voltage_cb, 10)
        self._node.create_subscription(OccupancyGrid,  '/map',              self._map_cb,    _map_qos)
        self._cmd_vel_pub = self._node.create_publisher(Twist, '/cmd_vel', 10)

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()
        logger.info("ROS2Bridge: spinning — subscribed /scan /camera/image_raw /odom /voltage /map, publishing /cmd_vel")

    def _image_cb(self, msg):
        """Convert sensor_msgs/Image (RGB_INT8 from Fortress) to cv2 BGR ndarray."""
        import numpy as np, cv2
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        with self._lock:
            self._latest_frame = bgr

    def get_frame(self):
        """Return latest camera frame as BGR ndarray, or None. Matches AstraCamera API."""
        with self._lock:
            f = self._latest_frame
        return f.copy() if f is not None else None

    def _depth_cb(self, msg):
        """Convert 32FC1 depth image (metres) from Fortress to colourised BGR uint8.
        White = near (0 m), dark blue = far (clipped at 4 m) — matches AstraCamera output."""
        import numpy as np, cv2
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        # Clip to sensor range and normalise to 0-255 (near=255 white, far=0 dark)
        clipped = np.clip(arr, 0.0, 4.0)
        norm = (255.0 * (1.0 - clipped / 4.0)).astype(np.uint8)
        coloured = cv2.applyColorMap(norm, cv2.COLORMAP_BONE)
        with self._lock:
            self._latest_depth = coloured

    def get_depth_frame(self):
        """Return latest colourised depth frame as BGR ndarray, or None. Matches AstraCamera API."""
        with self._lock:
            f = self._latest_depth
        return f.copy() if f is not None else None

    def _odom_cb(self, msg):
        """Extract position (metres), yaw (radians), and body twist from nav_msgs/Odometry."""
        import math
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose_m = {"x": p.x, "y": p.y, "theta": yaw}
            self._twist = {
                "vx": msg.twist.twist.linear.x,
                "vy": msg.twist.twist.linear.y,
                "wz": msg.twist.twist.angular.z,
            }

    def get_wheel_velocities(self) -> tuple:
        """Decompose body twist into per-wheel linear velocities (m/s) via mecanum kinematics.

        Wheel order: FL, FR, RL, RR.
        L = lx + ly ≈ 0.20 m for the Yahboom X3 (half-track + half-wheelbase).
        """
        L = 0.20
        with self._lock:
            t = dict(self._twist)
        vx, vy, wz = t["vx"], t["vy"], t["wz"]
        fl = vx - vy - L * wz
        fr = vx + vy + L * wz
        rl = vx + vy - L * wz
        rr = vx - vy + L * wz
        return fl, fr, rl, rr

    def get_pose_cm(self) -> dict:
        """Return pose with x/y in cm and theta in radians."""
        with self._lock:
            p = dict(self._pose_m)
        return {"x": p["x"] * 100.0, "y": p["y"] * 100.0, "theta": p["theta"]}

    def _voltage_cb(self, msg):
        v = float(msg.data)
        if v > 0.1:   # ignore empty/zero packets from Rosmaster
            with self._lock:
                self._voltage = v

    def get_battery_voltage(self) -> float:
        with self._lock:
            return self._voltage

    def _map_cb(self, msg):
        """Store the latest OccupancyGrid from SLAM Toolbox (/map topic)."""
        with self._lock:
            self._occupancy_grid = {
                "data":       list(msg.data),
                "width":      msg.info.width,
                "height":     msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x":   msg.info.origin.position.x,
                "origin_y":   msg.info.origin.position.y,
            }
            self._map_dirty = True

    def pop_map_update(self) -> dict | None:
        """If a new map has arrived since last call, convert it to a PNG and return
        a map_data message dict ready to JSON-encode and send to the GUI; else None."""
        import numpy as np, cv2, base64
        with self._lock:
            if not self._map_dirty or self._occupancy_grid is None:
                return None
            self._map_dirty = False
            g = dict(self._occupancy_grid)
        # -1 (unknown) → 128 grey,  0 (free) → 255 white,  100 (occupied) → 0 black
        arr = np.array(g["data"], dtype=np.int16)
        img = np.where(arr < 0, 128, np.where(arr == 0, 255, 0)).astype(np.uint8)
        img = img.reshape(g["height"], g["width"])
        img = np.flipud(img)   # ROS origin is bottom-left; canvas expects top-left
        _, buf = cv2.imencode('.png', img)
        png_b64 = base64.b64encode(bytes(buf)).decode('utf-8')
        return {
            "type": "map_data",
            "png_b64": png_b64,
            "meta": {
                "resolution": g["resolution"],
                "origin": [g["origin_x"], g["origin_y"]],
                "width":  g["width"],
                "height": g["height"],
            },
        }

    def get_occupancy_grid(self) -> dict | None:
        """Return the latest occupancy grid info dict, or None if not yet received."""
        with self._lock:
            return dict(self._occupancy_grid) if self._occupancy_grid else None

    def get_pose_m(self) -> dict:
        """Return the robot pose in metres: {x, y, theta}."""
        with self._lock:
            return dict(self._pose_m)

    def _scan_cb(self, msg):
        """Convert LaserScan → flat [x0,y0,x1,y1,…] (same format as YDLidarDriver)."""
        import math
        flat: list[float] = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                flat.append(r * math.cos(angle))
                flat.append(r * math.sin(angle))
        with self._lock:
            self._points = flat

    def get_points_xy(self, max_points: int = 512) -> list[float]:
        with self._lock:
            pts = self._points
        n = len(pts) // 2
        if n == 0:
            return []
        if n <= max_points:
            return list(pts)
        step = max(1, n // max_points)
        return np.array(pts, dtype=np.float32).reshape(-1, 2)[::step].ravel().tolist()

    def move(self, vx: float, vy: float, omega: float):
        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x  = float(vx)    * self.LINEAR_SCALE
        msg.linear.y  = float(vy)    * self.LINEAR_SCALE
        msg.angular.z = float(omega) * self.ANGULAR_SCALE
        self._cmd_vel_pub.publish(msg)

    def stop(self):
        self.move(0.0, 0.0, 0.0)

    def cleanup(self):
        import rclpy
        try:
            self._node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


def _launch_gazebo():
    """Start Gazebo + X3 robot as a background subprocess.

    Sources ROS2 and the workspace install, then calls x3_gazebo.launch.py.
    Returns the Popen handle so cleanup() can terminate it.
    """
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    install_setup = os.path.join(ws_root, 'install', 'setup.bash')

    # Kill any orphaned Ignition Fortress processes first
    subprocess.call(['bash', '-c', 'pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 -f "gz sim" 2>/dev/null || true'])

    # Build the subprocess environment: inherit everything from the current
    # process (which includes DISPLAY, XAUTHORITY, XDG_RUNTIME_DIR, LD_LIBRARY_PATH etc.)
    # then set the variables that must be set for Ignition Fortress + ROS2 to work.
    child_env = os.environ.copy()
    child_env.setdefault('DISPLAY', ':0')
    # GZ_SIM_RESOURCE_PATH tells Ignition Fortress where to find package:// URIs
    # and mesh files from the installed workspace.
    share_dir = os.path.join(ws_root, 'install', 'yahboomcar_description', 'share')
    gz_resource_path = child_env.get('GZ_SIM_RESOURCE_PATH', '')
    child_env['GZ_SIM_RESOURCE_PATH'] = f'{share_dir}:{gz_resource_path}' if gz_resource_path else share_dir
    # IGN_GAZEBO_RESOURCE_PATH is the alias used by some Fortress tools — set both
    child_env['IGN_GAZEBO_RESOURCE_PATH'] = child_env['GZ_SIM_RESOURCE_PATH']
    # OGRE_RTT_MODE was a workaround for Gazebo Classic/Ogre1 on NVIDIA — not needed
    # for Ignition Fortress which uses Ogre2.
    # ROS2 tools must run under system Python 3.10.
    # Miniconda prepends its Python 3.13 to PATH which breaks rclpy C extensions.
    # Strip all conda/miniconda dirs from PATH so /usr/bin/python3 wins.
    clean_path = ':'.join(
        p for p in child_env.get('PATH', '').split(':')
        if 'conda' not in p.lower()
    )
    child_env['PATH'] = clean_path
    # Also unset any conda-injected PYTHONPATH entries that would pull in 3.13
    if 'PYTHONPATH' in child_env:
        child_env['PYTHONPATH'] = ':'.join(
            p for p in child_env['PYTHONPATH'].split(':')
            if 'conda' not in p.lower()
        )
    # OpenCV (cv2) ships its own Qt plugins that are incompatible with Ignition's Qt.
    # When server_x3.py inherits QT_PLUGIN_PATH containing cv2/qt/plugins, the
    # Ignition GUI crashes with "Could not load Qt platform plugin xcb".
    # Strip any cv2 Qt paths so Ignition uses the system Qt plugins only.
    for qt_var in ('QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH'):
        if qt_var in child_env:
            cleaned = ':'.join(
                p for p in child_env[qt_var].split(':')
                if 'cv2' not in p and 'cv2' not in p.lower()
            )
            if cleaned:
                child_env[qt_var] = cleaned
            else:
                del child_env[qt_var]

    cmd = (
        f'source /opt/ros/humble/setup.bash && '
        f'source {install_setup} && '
        f'ros2 launch yahboomcar_nav x3_gazebo.launch.py'
    )
    log_path = '/tmp/gazebo_launch.log'
    log_file = open(log_path, 'w')
    proc = subprocess.Popen(
        ['bash', '-c', cmd],
        stdout=log_file,
        stderr=log_file,
        preexec_fn=os.setsid,   # process group — lets us kill all children
        env=child_env,
    )
    logger.info(f"Gazebo launched (pid {proc.pid}) — log: {log_path}")
    return proc


def _launch_ros2_stack():
    """Auto-launch the X3 hardware bringup (drivers + EKF, no SLAM) as a subprocess.

    Mirrors _launch_gazebo() — strips conda from PATH, sources ROS2 + workspace,
    then starts x3_bringup.launch.py.  SLAM is started separately via the GUI.
    Returns the Popen handle so cleanup() can SIGTERM the process group.
    """
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    install_setup = os.path.join(ws_root, 'install', 'setup.bash')

    child_env = os.environ.copy()
    # Strip conda dirs — they inject Python 3.13 which breaks rclpy C extensions
    child_env['PATH'] = ':'.join(
        p for p in child_env.get('PATH', '').split(':')
        if 'conda' not in p.lower()
    )
    if 'PYTHONPATH' in child_env:
        child_env['PYTHONPATH'] = ':'.join(
            p for p in child_env['PYTHONPATH'].split(':')
            if 'conda' not in p.lower()
        )

    cmd = (
        f'source /opt/ros/humble/setup.bash && '
        f'source {install_setup} && '
        f'ros2 launch yahboomcar_nav x3_bringup.launch.py'
    )
    log_path = '/tmp/ros2_bringup.log'
    log_file = open(log_path, 'w')
    proc = subprocess.Popen(
        ['bash', '-c', cmd],
        stdout=log_file,
        stderr=log_file,
        preexec_fn=os.setsid,
        env=child_env,
    )
    logger.info(f"ROS2 bringup launched (pid {proc.pid}) — log: {log_path}")
    return proc


def _launch_slam(use_sim_time: bool = False):
    """Start SLAM Toolbox as a background subprocess.

    For simulation (use_sim_time=True): launches x3_slam_sim.launch.py
      — only slam_toolbox_node, no hardware drivers (Gazebo provides topics).
    For physical (use_sim_time=False): launches x3_slam.launch.py
      — full stack (Mcnamu_driver, base_node, IMU filter, EKF, YDLidar, slam_toolbox).

    Uses the same clean-environment pattern as _launch_gazebo() to avoid
    conda/miniconda Python version conflicts.
    Returns the Popen handle.
    """
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    install_setup = os.path.join(ws_root, 'install', 'setup.bash')

    # Both sim and physical use x3_slam_sim.launch.py (slam_toolbox only).
    # In --ros2 mode the hardware stack is already running via _launch_ros2_stack().
    # In --sim mode Gazebo already provides all sensor topics.
    st_arg = 'use_sim_time:=true' if use_sim_time else 'use_sim_time:=false'
    launch_file = f'x3_slam_sim.launch.py {st_arg}'

    child_env = os.environ.copy()
    child_env.setdefault('DISPLAY', ':0')
    # Strip conda dirs so system Python 3.10 is used for ROS2 nodes
    clean_path = ':'.join(
        p for p in child_env.get('PATH', '').split(':')
        if 'conda' not in p.lower()
    )
    child_env['PATH'] = clean_path
    if 'PYTHONPATH' in child_env:
        child_env['PYTHONPATH'] = ':'.join(
            p for p in child_env['PYTHONPATH'].split(':')
            if 'conda' not in p.lower()
        )

    cmd = (
        f'source /opt/ros/humble/setup.bash && '
        f'source {install_setup} && '
        f'ros2 launch yahboomcar_nav {launch_file}'
    )
    log_path = '/tmp/slam_launch.log'
    log_file = open(log_path, 'w')
    proc = subprocess.Popen(
        ['bash', '-c', cmd],
        stdout=log_file,
        stderr=log_file,
        preexec_fn=os.setsid,
        env=child_env,
    )
    logger.info(f"SLAM Toolbox launched (pid {proc.pid}, sim={use_sim_time}) — log: {log_path}")
    return proc


def _save_map(name: str) -> tuple[bool, str]:
    """Call the slam_toolbox/save_map service synchronously.

    Saves the map to the yahboomcar_nav maps directory so it immediately
    appears in the GUI map selector.
    Returns (success: bool, message: str).
    """
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    maps_dir = os.path.join(ws_root, 'src', 'yahboomcar_nav', 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    map_path = os.path.join(maps_dir, name)

    install_setup = os.path.join(ws_root, 'install', 'setup.bash')
    child_env = os.environ.copy()
    clean_path = ':'.join(
        p for p in child_env.get('PATH', '').split(':')
        if 'conda' not in p.lower()
    )
    child_env['PATH'] = clean_path

    cmd = (
        f'source /opt/ros/humble/setup.bash && '
        f'source {install_setup} && '
        f'ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap '
        f'\'{{"name": {{"data": "{map_path}"}}}}\''
    )
    try:
        result = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True, text=True, timeout=10, env=child_env,
        )
        if result.returncode == 0:
            logger.info(f"Map saved: {map_path}")
            return True, f"Map '{name}' saved"
        else:
            logger.warning(f"Map save failed: {result.stderr.strip()}")
            return False, result.stderr.strip() or "Service call failed"
    except subprocess.TimeoutExpired:
        return False, "Timed out waiting for save_map service"
    except Exception as exc:
        return False, str(exc)


def initialize_hardware():
    global ros_board, ros_bridge, drive, lidar, camera, model, oled, _gazebo_proc
    global nav2_client, _ros2_stack_proc, frontier_explorer

    logger.info("="*50)
    logger.info("Initializing Yahboom X3 Hardware")
    logger.info("="*50)

    if SIM_MODE:
        # Simulation mode: create the ROS2Bridge now so topics are ready to receive
        # data as soon as Gazebo starts.  Gazebo itself is launched on-demand when
        # the user clicks "🚀 Gazebo" in the GUI (launch_gazebo WS message).
        bridge = ROS2Bridge()
        ros_bridge = bridge
        drive = bridge
        lidar = bridge
        camera = bridge   # bridge provides get_frame() from /camera/image_raw
        nav2_client = Nav2Client(bridge._node)
        frontier_explorer = FrontierExplorer(nav2_client, bridge)
        logger.info("Nav2Client + FrontierExplorer initialized (sim mode) — click 🚀 Gazebo in GUI to start simulation")
    elif ROS2_MODE:
        # ROS2 bridge mode: subscribe to /scan, /odom, publish /cmd_vel.
        # Auto-launches x3_bringup.launch.py (hardware drivers: Mcnamu_driver_X3,
        # base_node_X3, IMU filter, EKF, YDLidar) so /cmd_vel drives the real motors
        # and real /scan + /odom are published.
        # Camera stays as direct USB (AstraCamera) — falls through to init below.
        logger.info("ROS2 mode: launching hardware bringup stack...")
        _ros2_stack_proc = _launch_ros2_stack()
        # Give the hardware drivers a moment to come up before the bridge subscribes
        import time as _time; _time.sleep(3.0)
        logger.info("ROS2 mode: creating bridge — subscribing to hardware topics")
        bridge = ROS2Bridge()
        ros_bridge = bridge
        drive = bridge
        lidar = bridge
        # camera intentionally left unset — falls through to AstraCamera init below
        nav2_client = Nav2Client(bridge._node)
        frontier_explorer = FrontierExplorer(nav2_client, bridge)
        logger.info("Nav2Client + FrontierExplorer initialized (ros2 mode)")
    else:
        # 1. Motor Controller (Serial) - uses auto-detected SERIAL_PORT
        logger.info(f"Connecting to Rosmaster on {SERIAL_PORT}...")
        ros_board = Rosmaster(sim_mode=False)

        # 2. Mecanum Drive Wrapper
        drive = MecanumDrive(ros_board)
        logger.info("Mecanum Drive initialized")

        # 5. YDLidar
        logger.info(f"Initializing Lidar on {LIDAR_PORT}...")
        lidar = YDLidarDriver(port=LIDAR_PORT, sim_mode=False)

    # 3. Camera — direct USB in both direct-hardware and --ros2 modes.
    #    Only --sim uses ROS2Bridge.get_frame() (Gazebo publishes /camera/image_raw).
    if not SIM_MODE:
        logger.info("Initializing Camera...")
        camera = AstraCamera(width=640, height=480, sim_mode=False, enable_depth=False)

    # 4. YOLO Model — prefer TRT engine (.engine), fall back to .pt on CPU
    try:
        from trt_detector import TRTDetector
        _engine_path = YOLO_MODEL.replace(".pt", ".engine")
        if os.path.exists(_engine_path):
            model = TRTDetector(_engine_path, pt_path=YOLO_MODEL,
                                conf_thres=CONFIDENCE_THRESHOLD)
            logger.info(f"YOLO running on: TensorRT GPU ({_engine_path})")
        else:
            logger.info(f"Loading YOLO (CPU): {YOLO_MODEL}")
            model = YOLO(YOLO_MODEL)
    except Exception as e:
        logger.error(f"YOLO Load Failed: {e}")


    # 7. OLED Display (SSD1306 on I2C bus 1 — Jetson Orin pins 3/5)
    logger.info("Initializing OLED display...")
    oled = OLEDDisplay(i2c_port=7, i2c_address=0x3C, sim_mode=SIM_MODE)
    oled.show(["X3 Robot", "Starting...", ""])

    logger.info("="*50)
    logger.info("Initialization Complete")
    logger.info("="*50)

def cleanup():
    logger.info("Cleaning up...")
    if nav2_client is not None:
        nav2_client.stop_nav2()
    if (SIM_MODE or ROS2_MODE) and drive is not None:
        drive.cleanup()  # ROS2Bridge.cleanup() shuts down rclpy
    if _gazebo_proc is not None:
        logger.info("Shutting down Gazebo...")
        try:
            os.killpg(os.getpgid(_gazebo_proc.pid), signal.SIGTERM)
        except Exception:
            pass
    if _ros2_stack_proc is not None:
        logger.info("Shutting down ROS2 hardware stack...")
        try:
            os.killpg(os.getpgid(_ros2_stack_proc.pid), signal.SIGTERM)
        except Exception:
            pass
    if _slam_proc is not None:
        logger.info("Shutting down SLAM Toolbox...")
        try:
            os.killpg(os.getpgid(_slam_proc.pid), signal.SIGTERM)
        except Exception:
            pass
    if ros_board: ros_board.cleanup()
    if camera: camera.cleanup()
    if not (SIM_MODE or ROS2_MODE) and lidar: lidar.cleanup()
    if oled: oled.cleanup()


# =============================================================================
# NETWORK INFO HELPERS
# =============================================================================

def _get_ip() -> str:
    """Return the first non-loopback IP address, or 'No IP'."""
    try:
        result = subprocess.run(
            ['hostname', '-I'], capture_output=True, text=True, timeout=2
        )
        ips = result.stdout.strip().split()
        return ips[0] if ips else 'No IP'
    except Exception:
        return 'No IP'


def _get_ssid() -> str:
    """Return the connected WiFi SSID, or 'No WiFi'."""
    try:
        result = subprocess.run(
            ['iwgetid', '--raw'], capture_output=True, text=True, timeout=2
        )
        ssid = result.stdout.strip()
        return ssid if ssid else 'No WiFi'
    except Exception:
        return 'No WiFi'

# =============================================================================
# MOTION: Convert tank-drive (left/right power) to mecanum (vx, vy, omega)
# =============================================================================

def _enqueue_motion(vx: float, vy: float, omega: float):
    """Put a (vx, vy, omega) command onto motion_queue.

    Drops the oldest entry when the queue is full so stale commands never
    accumulate — the newest command always wins.
    """
    if motion_queue is None:
        return
    if motion_queue.full():
        try:
            motion_queue.get_nowait()
        except asyncio.QueueEmpty:
            pass
    try:
        motion_queue.put_nowait((vx, vy, omega))
    except asyncio.QueueFull:
        pass  # race: another coroutine filled it between our drain and put


def _enqueue_tank(left: float, right: float):
    """Convert tank-drive (left/right) to holonomic (vx, omega) and enqueue."""
    vx    = (left + right) / 2.0
    omega = (right - left) / 2.0
    _enqueue_motion(vx, 0.0, omega)

# =============================================================================
# WEBSOCKET HANDLER
# =============================================================================

def _maps_dir() -> str:
    """Absolute path to the yahboomcar_nav maps directory."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'yahboomcar_nav', 'maps')


def _list_maps() -> list:
    """Return list of map YAML filenames available in the maps directory."""
    d = _maps_dir()
    if not os.path.isdir(d):
        return []
    return [f for f in os.listdir(d) if f.endswith('.yaml')]


def _load_map_data(yaml_name: str) -> dict | None:
    """
    Read a ROS map YAML + PGM and return a dict suitable for the GUI:
      {png_b64, meta: {resolution, origin:[x,y], width, height}}
    Returns None on any error.
    """
    import yaml as _yaml
    d = _maps_dir()
    yaml_path = os.path.join(d, yaml_name)
    if not os.path.isfile(yaml_path):
        return None
    try:
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        pgm_name = meta.get('image', '')
        if not os.path.isabs(pgm_name):
            pgm_name = os.path.join(d, pgm_name)
        img = cv2.imread(pgm_name, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return None
        # Encode as PNG (lossless) for accurate display
        _, buf = cv2.imencode('.png', img)
        png_b64 = base64.b64encode(buf).decode('utf-8')
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        return {
            "png_b64": png_b64,
            "meta": {
                "resolution": float(meta.get('resolution', 0.05)),
                "origin": [float(origin[0]), float(origin[1])],
                "width":  int(img.shape[1]),
                "height": int(img.shape[0]),
            },
        }
    except Exception as exc:
        logger.warning(f"[map] Failed to load {yaml_name}: {exc}")
        return None


async def handle_client(websocket):
    global detection_enabled, depth_enabled, lidar_enabled, is_auto_driving
    global current_left_power, current_right_power
    global model, active_model_name
    global _gazebo_proc, nav2_client

    logger.info("Client connected")
    connected_clients.add(websocket)
    if camera:
        camera._has_clients = True  # P7: allow capture loop to store frames

    # Tell the client what mode the server is running in
    _mode = "sim" if SIM_MODE else "ros2" if ROS2_MODE else "direct"
    await websocket.send(json.dumps({"type": "hello", "mode": _mode}))

    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                msg_type = data.get("type")

                if msg_type == "set_power":
                    # Tank-drive sliders / gamepad axes
                    motor = data.get("motor")
                    power = float(data.get("power", 0.0))
                    if motor == "left":
                        current_left_power = power
                    elif motor == "right":
                        current_right_power = power
                    _enqueue_tank(current_left_power, current_right_power)

                elif msg_type == "set_move":
                    # Holonomic move: vx, vy, omega (direct)
                    vx    = float(data.get("vx", 0.0))
                    vy    = float(data.get("vy", 0.0))
                    omega = float(data.get("omega", 0.0))
                    # Keep display globals in sync so motor status card updates
                    current_left_power  = max(-1.0, min(1.0, vx - omega))
                    current_right_power = max(-1.0, min(1.0, vx + omega))
                    _enqueue_motion(vx, vy, omega)

                elif msg_type == "move":
                    # D-pad direction buttons
                    direction = data.get("direction")
                    _dir_map = {
                        "forward":      ( 0.5,  0.0,  0.0),
                        "backward":     (-0.5,  0.0,  0.0),
                        "left":         ( 0.0,  0.0,  0.5),   # Rotate CCW
                        "right":        ( 0.0,  0.0, -0.5),   # Rotate CW
                        "strafe_left":  ( 0.0, -0.5,  0.0),
                        "strafe_right": ( 0.0,  0.5,  0.0),
                        "stop":         ( 0.0,  0.0,  0.0),
                    }
                    if direction in _dir_map:
                        _enqueue_motion(*_dir_map[direction])

                elif msg_type == "stop":
                    current_left_power = 0.0
                    current_right_power = 0.0
                    _enqueue_motion(0.0, 0.0, 0.0)

                elif msg_type == "toggle_detection":
                    detection_enabled = data.get("enabled", False)
                    logger.info(f"Detection: {detection_enabled}")

                elif msg_type == "launch_gazebo":
                    if SIM_MODE:
                        if _gazebo_proc is None or _gazebo_proc.poll() is not None:
                            _gazebo_proc = _launch_gazebo()
                            await websocket.send(json.dumps(
                                {"type": "launch_gazebo_result", "success": True,
                                 "msg": "Gazebo launching..."}))
                        else:
                            await websocket.send(json.dumps(
                                {"type": "launch_gazebo_result", "success": False,
                                 "msg": "Gazebo is already running"}))
                    else:
                        await websocket.send(json.dumps(
                            {"type": "launch_gazebo_result", "success": False,
                             "msg": "launch_gazebo only available in --sim mode"}))

                elif msg_type == "toggle_lidar":
                    lidar_enabled = data.get("enabled", False)
                    if lidar and not (SIM_MODE or ROS2_MODE):
                        if lidar_enabled:
                            lidar.start()
                        else:
                            lidar.stop()
                    logger.info(f"Lidar: {'enabled' if lidar_enabled else 'disabled'}")

                elif msg_type == "toggle_depth":
                    depth_enabled = data.get("enabled", False)
                    if camera and hasattr(camera, '_depth_stream'):
                        if depth_enabled and camera._depth_stream is None:
                            camera._open_depth()
                        elif not depth_enabled and camera._depth_stream is not None:
                            camera._close_depth()
                    logger.info(f"Depth streaming: {depth_enabled}")

                elif msg_type == "start_auto_drive":
                    is_auto_driving = True
                    logger.info("Auto-drive started")

                elif msg_type == "stop_auto_drive":
                    is_auto_driving = False
                    current_left_power = 0.0
                    current_right_power = 0.0
                    _enqueue_motion(0.0, 0.0, 0.0)
                    if nav2_client:
                        nav2_client.cancel()

                # ── Nav2 messages ──────────────────────────────────────────
                elif msg_type == "launch_nav2":
                    if nav2_client:
                        use_st = data.get("use_sim_time", SIM_MODE)
                        map_f  = data.get("map")
                        slam   = data.get("slam", False)
                        ok = nav2_client.launch_nav2(
                            use_sim_time=use_st, map_path=map_f, slam=slam)
                        await websocket.send(json.dumps({
                            "type": "nav2_launch_result",
                            "success": ok,
                            "msg": "Nav2 launching..." if ok else "Nav2 already running or failed",
                        }))
                    else:
                        await websocket.send(json.dumps({
                            "type": "nav2_launch_result", "success": False,
                            "msg": "Nav2 requires --ros2 or --sim mode",
                        }))

                elif msg_type == "stop_nav2":
                    if nav2_client:
                        nav2_client.stop_nav2()

                elif msg_type == "set_nav_goal":
                    if nav2_client:
                        nav2_client.navigate_to(
                            float(data.get("x", 0.0)),
                            float(data.get("y", 0.0)),
                            float(data.get("theta", 0.0)),
                        )
                    else:
                        logger.warning("set_nav_goal ignored: no Nav2Client")

                elif msg_type == "cancel_nav":
                    if nav2_client:
                        nav2_client.cancel()

                elif msg_type == "set_initial_pose":
                    if nav2_client:
                        nav2_client.set_initial_pose(
                            float(data.get("x", 0.0)),
                            float(data.get("y", 0.0)),
                            float(data.get("theta", 0.0)),
                        )

                elif msg_type == "get_maps":
                    maps = _list_maps()
                    await websocket.send(json.dumps(
                        {"type": "map_list", "maps": maps}))

                elif msg_type == "request_map":
                    map_data = _load_map_data(data.get("map", ""))
                    if map_data:
                        await websocket.send(json.dumps(
                            {"type": "map_data", **map_data}))
                    else:
                        await websocket.send(json.dumps(
                            {"type": "map_data", "error": "Map not found"}))

                # ── Frontier exploration messages ───────────────────────────
                elif msg_type == "start_frontier_explore":
                    if frontier_explorer and ROS2_MODE:
                        ok = frontier_explorer.start()
                        await websocket.send(json.dumps({
                            "type": "frontier_explore_result",
                            "success": ok,
                            "msg": "Frontier exploration started" if ok else "Already exploring",
                        }))
                    else:
                        await websocket.send(json.dumps({
                            "type": "frontier_explore_result",
                            "success": False,
                            "msg": "Frontier exploration requires ROS2 mode with Nav2 running",
                        }))

                elif msg_type == "stop_frontier_explore":
                    if frontier_explorer:
                        frontier_explorer.stop()
                        await websocket.send(json.dumps({
                            "type": "frontier_explore_result",
                            "success": True,
                            "msg": "Frontier exploration stopped",
                        }))

                # ── SLAM messages ───────────────────────────────────────────
                elif msg_type == "start_slam":
                    global _slam_proc
                    if _slam_proc is None or _slam_proc.poll() is not None:
                        _slam_proc = _launch_slam(use_sim_time=SIM_MODE)
                        logger.info("SLAM Toolbox launched")
                    else:
                        logger.info("SLAM already running")

                elif msg_type == "stop_slam":
                    if _slam_proc is not None and _slam_proc.poll() is None:
                        try:
                            os.killpg(os.getpgid(_slam_proc.pid), signal.SIGTERM)
                            logger.info("SLAM Toolbox stopped")
                        except Exception as exc:
                            logger.warning(f"Failed to stop SLAM: {exc}")
                        _slam_proc = None

                elif msg_type == "save_map":
                    map_name = data.get("name", "slam_map").strip() or "slam_map"
                    ok, msg_text = _save_map(map_name)
                    await websocket.send(json.dumps({
                        "type": "save_map_result",
                        "success": ok,
                        "message": msg_text,
                        "name": map_name,
                    }))
                    if ok:
                        await websocket.send(json.dumps(
                            {"type": "map_list", "maps": _list_maps()}))

                elif msg_type == "set_model":
                    model_name = data.get("model")
                    if model_name:
                        new_model_path = find_model_path(model_name)
                        try:
                            if not os.path.exists(new_model_path):
                                logger.warning(f"Model file not found: {new_model_path}")
                            _new_engine = new_model_path.replace(".pt", ".engine")
                            if os.path.exists(_new_engine):
                                from trt_detector import TRTDetector
                                new_model = TRTDetector(_new_engine, pt_path=new_model_path,
                                                        conf_thres=CONFIDENCE_THRESHOLD)
                                logger.info(f"Switched to TRT model: {_new_engine}")
                            else:
                                new_model = YOLO(new_model_path)
                                logger.info(f"Switched to YOLO model: {new_model_path}")
                            model = new_model
                            active_model_name = model_name
                            
                            response = {
                                "type": "model_changed",
                                "model": model_name,
                                "success": True,
                                "path": new_model_path
                            }
                            await websocket.send(json.dumps(response))
                        except Exception as e:
                            logger.error(f"Failed to load new YOLO model {model_name}: {e}")
                            await websocket.send(json.dumps({
                                "type": "model_changed",
                                "model": model_name,
                                "success": False,
                                "error": str(e),
                                "path": new_model_path
                            }))

                # Silently ignore GUI-only messages (capture, demo, etc.)
                elif msg_type in ("set_classes", "set_labels",
                                  "capture_image", "download_images",
                                  "collect_blur_dataset", "start_golden_collection",
                                  "stop_golden_collection", "start_demo", "stop_demo",
                                  "disconnect"):
                    pass

                else:
                    logger.debug(f"Unhandled message type: {msg_type}")

            except Exception as e:
                logger.error(f"Msg Error: {e}")

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.discard(websocket)
        if camera and not connected_clients:
            camera._has_clients = False  # P7: no clients left — skip lock+copy in capture loop
        # Eager stop — don't wait for the watchdog timeout on disconnect
        _enqueue_motion(0.0, 0.0, 0.0)
        logger.info("Client disconnected")

# =============================================================================
# MAIN BROADCAST LOOP
# =============================================================================

async def broadcast_loop():
    global _cam_frame_count, _yolo_frame_count, _fps_last_time
    global fps_camera, fps_detection, last_detections, depth_enabled, lidar_enabled
    global _batt_cache_v, _batt_cache_time  # P9

    loop = asyncio.get_event_loop()
    _depth_cycle = 0  # throttle depth to ~10 fps (every other 20fps cycle)

    while True:
        if connected_clients:
            now = time.time()

            # 1. Camera frame — blocking capture in thread pool
            frame = await loop.run_in_executor(None, camera.get_frame) if camera else None
            if frame is not None:
                _cam_frame_count += 1

            # 2. YOLO + JPEG encode — all in executor (P1+P2: off event loop, draw on copy)
            #    P3: return raw bytes — sent as binary WS frame, eliminating base64 entirely
            img_bytes = b""
            if detection_enabled and frame is not None and model:
                def _run_yolo():
                    _names = model.names or {}
                    annotated = frame.copy()   # P2: never mutate the shared frame reference
                    results = model(frame, verbose=False, conf=CONFIDENCE_THRESHOLD, imgsz=INFERENCE_SIZE)
                    dets = []
                    for r in results:
                        for box in r.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            label = _names.get(int(box.cls[0]), str(int(box.cls[0])))
                            conf  = float(box.conf[0])
                            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            dets.append({"label": label, "bbox": [x1, y1, x2, y2], "conf": conf})
                    _, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    return dets, bytes(buf)
                last_detections, img_bytes = await loop.run_in_executor(None, _run_yolo)
                _yolo_frame_count += 1
            elif frame is not None:
                def _encode_frame():
                    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    return bytes(buf)
                img_bytes = await loop.run_in_executor(None, _encode_frame)

            # 3. FPS update every second
            elapsed = now - _fps_last_time
            if elapsed >= 1.0:
                fps_camera    = round(_cam_frame_count / elapsed, 1)
                fps_detection = round(_yolo_frame_count / elapsed, 1)
                _cam_frame_count  = 0
                _yolo_frame_count = 0
                _fps_last_time    = now

            # 4. Depth frame — throttled to 10fps; encode also in executor (P1)
            depth_str = ""
            _depth_cycle += 1
            if depth_enabled and camera and (_depth_cycle % 2 == 0):
                def _get_depth():
                    df = camera.get_depth_frame()
                    if df is None:
                        return ""
                    _, dbuf = cv2.imencode('.jpg', df, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    return base64.b64encode(dbuf).decode('utf-8')
                depth_str = await loop.run_in_executor(None, _get_depth)

            # 5. Lidar points (only when toggle is on)
            scan_points = lidar.get_points_xy() if (lidar and lidar_enabled) else []

            # 6. Robot pose — from /odom (ROS2/sim) or zeros if unavailable
            if hasattr(lidar, 'get_pose_cm'):
                pose = lidar.get_pose_cm()
            else:
                pose = {"x": 0.0, "y": 0.0, "theta": 0.0}

            # 7. Encoders + battery
            if drive is not None and (ROS2_MODE or SIM_MODE) and hasattr(drive, 'get_battery_voltage'):
                # Voltage comes from the /voltage topic published by Mcnamu_driver_X3
                # Per-wheel velocities (m/s) derived from /odom twist via mecanum kinematics
                m1_enc, m2_enc, m3_enc, m4_enc = drive.get_wheel_velocities()
                batt_v = drive.get_battery_voltage()
            elif ros_board:
                m1_enc, m2_enc, m3_enc, m4_enc = ros_board.get_motor_encoder()
                if now - _batt_cache_time >= 1.0:
                    _batt_cache_v    = ros_board.get_battery_voltage()
                    _batt_cache_time = now
                batt_v = _batt_cache_v
            else:
                m1_enc = m2_enc = m3_enc = m4_enc = 0
                batt_v = 12.0

            batt_pct    = max(0.0, min(100.0, (batt_v - 8.1) / (12.6 - 8.1) * 100.0))
            avg_pwr     = (abs(current_left_power) + abs(current_right_power)) / 2.0
            est_current = 0.5 + (avg_pwr * 6.0)
            est_watts   = batt_v * est_current

            # P3: send camera frame as a binary WebSocket message (raw JPEG, no base64)
            if img_bytes:
                websockets.broadcast(connected_clients, img_bytes)

            # 8. Build readout (P10: removed always-None/False fields; P3: no "image" key)
            nav_status = nav2_client.get_status() if nav2_client else {"state": "UNAVAILABLE"}
            msg = {
                "type": "readout",
                "depth_image": depth_str,
                "lidar_points": scan_points,
                "robot_pose": pose,
                "m1_pos": m1_enc,
                "m2_pos": m2_enc,
                "m3_pos": m3_enc,
                "m4_pos": m4_enc,
                "m1_power": current_left_power,
                "m2_power": current_right_power,
                "m3_power": current_left_power,
                "m4_power": current_right_power,
                "left_power": current_left_power,
                "right_power": current_right_power,
                "detection_enabled": detection_enabled,
                "is_auto_driving": is_auto_driving,
                "nav_phase": nav_status["state"],
                "nav": nav_status,
                "slam_active": _slam_proc is not None and _slam_proc.poll() is None,
                "frontier": frontier_explorer.status() if frontier_explorer else None,
                "active_model_name": active_model_name,
                "fps_camera": fps_camera,
                "fps_detection": fps_detection,
                "detections": last_detections,
                "battery": {"voltage": batt_v, "amps": est_current, "watts": est_watts},
                "power": {
                    "voltage":     batt_v,
                    "current":     est_current,
                    "power":       est_watts,
                    "battery_pct": batt_pct,
                },
            }

            # Push SLAM map update when a new OccupancyGrid has arrived from /map
            if ros_bridge and (map_upd := ros_bridge.pop_map_update()):
                websockets.broadcast(connected_clients, json.dumps(map_upd))

            websockets.broadcast(connected_clients, json.dumps(msg))

        await asyncio.sleep(0.05)  # 20 FPS cap

async def motion_loop():
    """Dedicated 100 Hz motion command consumer.

    Drains motion_queue and calls drive.move() at consistent low latency,
    independent of broadcast_loop timing and lidar GIL contention.

    Safety watchdog: if no command arrives within MOTION_WATCHDOG_TIMEOUT
    seconds (e.g. dropped WebSocket), motors are stopped automatically.
    Watchdog is suppressed while Nav2 auto-drive is active.
    """
    _last_cmd_time = time.monotonic()
    _watchdog_fired = False

    while True:
        if motion_queue is None:
            await asyncio.sleep(0.01)
            continue

        cmd = None
        try:
            cmd = motion_queue.get_nowait()
        except asyncio.QueueEmpty:
            pass

        if cmd is not None:
            vx, vy, omega = cmd
            if drive:
                drive.move(vx, vy, omega)
            _last_cmd_time = time.monotonic()
            _watchdog_fired = False
        elif not is_auto_driving and not _watchdog_fired:
            if time.monotonic() - _last_cmd_time > MOTION_WATCHDOG_TIMEOUT:
                if drive:
                    drive.move(0.0, 0.0, 0.0)
                _watchdog_fired = True
                logger.debug("motion_loop: watchdog fired — motors stopped")

        await asyncio.sleep(0.01)  # 100 Hz


async def oled_loop():
    """Refresh OLED with WiFi SSID and IP every 5 seconds."""
    while True:
        try:
            if oled:
                ssid    = _get_ssid()
                ip      = _get_ip()
                clients = len(connected_clients)
                oled.show([
                    f"WiFi:{ssid[:16]}",
                    f"IP:{ip}",
                    f"WS:{WS_PORT} C:{clients}",
                ])
        except Exception as e:
            logger.warning(f"OLED update error: {e}")
        await asyncio.sleep(5)


async def main():
    global motion_queue
    initialize_hardware()
    motion_queue = asyncio.Queue(maxsize=2)
    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT):
        logger.info(f"Server started on ws://0.0.0.0:{WS_PORT}")
        await asyncio.gather(broadcast_loop(), motion_loop(), oled_loop())

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
