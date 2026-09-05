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
try:
    import orjson
    def orjson_dumps(msg):
        return orjson.dumps(msg).decode('utf-8')
except ImportError:
    def orjson_dumps(msg):
        return json.dumps(msg)
import logging
import argparse
import base64
import functools
import http.server
import struct
import numpy as np
import cv2
import websockets
import sys
import os
import signal
import threading
import socket
import subprocess
from multiprocessing import shared_memory
import math
try:
    from dynamixel_servo import (
        XL430, DynamixelError, COUNTS_PER_DEG,
        OPERATING_MODE_POSITION, deg_s_to_profile_velocity,
    )
except ImportError:
    pass

# Shared Memory Buffers for zero-copy IPC (Idea 145)
_shared_bgr_shm = None
_shared_depth_shm = None
_shared_bgr_array = None
_shared_depth_array = None
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
    Rosmaster, MecanumDrive, YDLidarDriver, AstraCamera, OLEDDisplay, SERIAL_PORT,
    INA226BatteryMonitor
)
from nav2_client import Nav2Client
from frontier_explorer import FrontierExplorer
from velocity_estimator import VelocityEstimator
from pathlib import Path

# =============================================================================
# CONFIGURATION
# =============================================================================
parser = argparse.ArgumentParser(description='Yahboom X3 Control Server (ROS2 hardware mode by default)')
parser.add_argument('--sim', action='store_true', help='Run in simulation mode (laptop, Gazebo on demand)')
parser.add_argument('--domain-id', type=int, default=42, dest='domain_id',
                    help='ROS_DOMAIN_ID for multi-machine ROS2 (must match laptop). '
                         'Overrides the ROS_DOMAIN_ID environment variable.')
parser.add_argument('--no-oak', action='store_true', dest='no_oak',
                    help='Disable the OAK-D Lite driver (stereo + depth + IMU). '
                         'When enabled (default), the OAK-D is the depth source.')
parser.add_argument('--no-oak-spatial', action='store_true', dest='no_oak_spatial',
                    help='Disable on-device YOLO spatial detection on the OAK-D '
                         '(still streams stereo/depth/IMU).')
parser.add_argument('--oak-ros-publish', action='store_true', dest='oak_ros_publish',
                    help='Republish the OAK-D streams on ROS2 topics (/oak/depth/image_raw, '
                         '/oak/left|right/image_raw, /oak/imu, /oak/detections) so they can be '
                         'recorded with "ros2 bag record" or viewed in RViz. Off by default — '
                         'the images cost real CPU/bandwidth. See record_bag.sh.')

def _valid_oak_ros_rate(val_str):
    import math
    val = float(val_str)
    if math.isnan(val) or math.isinf(val) or val <= 0.0:
        raise argparse.ArgumentTypeError(f"oak_ros_rate must be a positive finite float, got '{val_str}'")
    return val

parser.add_argument('--oak-ros-rate', type=_valid_oak_ros_rate, default=10.0, dest='oak_ros_rate',
                    help='Publish rate (Hz) for --oak-ros-publish. Default 10.')
parser.add_argument('--oak-ros-no-stereo', action='store_true', dest='oak_ros_no_stereo',
                    help='With --oak-ros-publish, skip the mono left/right images '
                         '(roughly halves the bag size; keeps depth + IMU + detections).')
parser.add_argument('--webrtc-camera', action='store_true', dest='webrtc_camera',
                    help='Serve the color camera over WebRTC (via mediamtx) instead of '
                         'base64/WebSocket. Releases the Astra so ffmpeg can capture it at '
                         'full framerate; launches mediamtx. GUI shows the WebRTC <video>.')
parser.add_argument('--auto-nav2-map', type=str, default=None, dest='auto_nav2_map',
                    metavar='MAP.yaml',
                    help='Launch Nav2 + AMCL on this map at startup. OFF by '
                         'default: AMCL publishes map->odom, and so does '
                         'slam_toolbox, so leaving this on breaks mapping and '
                         'makes RViz drop odom-framed messages.')
parser.add_argument('--oak-cloud', action='store_true', dest='oak_cloud',
                    help='Enable OAK-D point cloud publishing.')
args = parser.parse_args()
SIM_MODE  = args.sim
ROS2_MODE = not args.sim  # ROS2 hardware bridge is the default; only --sim disables it
OAK_ENABLED = not args.no_oak  # OAK-D Lite supplies stereo/depth/imu unless --no-oak
OAK_SPATIAL = not args.no_oak_spatial  # on-device YOLO spatial detection (if blob present)
OAK_ROS_PUBLISH = args.oak_ros_publish      # republish OAK streams as ROS2 topics (bagging/RViz)
OAK_ROS_RATE = args.oak_ros_rate
OAK_ROS_STEREO = not args.oak_ros_no_stereo
WEBRTC_CAMERA = args.webrtc_camera  # color via WebRTC/mediamtx instead of base64 (releases Astra)
AUTO_NAV2_MAP = args.auto_nav2_map  # None = do not auto-launch Nav2/AMCL (see --auto-nav2-map)

# Apply ROS_DOMAIN_ID early — must be set before rclpy.init() inside ROS2Bridge
if args.domain_id is not None:
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)

# Force FastDDS to use the TCP profile for large messages when in Simple Discovery mode
if os.environ.get('X3_SIMPLE_DISCOVERY') == '1':
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/jetson/x3_ws/config/fastdds_tcp_server.xml'
    os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = '/home/jetson/x3_ws/config/fastdds_tcp_server.xml'

def _resolve_discovery_server() -> str:
    """Resolve the active IP address to use for the FastDDS Discovery Server.
    Avoids 127.0.0.1 loopback locator advertising to ensure multi-machine connection works.
    """
    env_val = os.environ.get('ROS_DISCOVERY_SERVER')
    if env_val:
        return env_val
    try:
        result = subprocess.run(
            ['hostname', '-I'], capture_output=True, text=True, timeout=2
        )
        ips = result.stdout.strip().split()
        if ips:
            # Filter out docker interface IP if possible, prefer 10.x or 192.x
            for ip in ips:
                if not ip.startswith('172.'):
                    return f"{ip}:11811"
            return f"{ips[0]}:11811"
    except Exception:
        pass
    return "127.0.0.1:11811"

# Opt-out of the FastDDS discovery server in favour of default ROS2 Simple
# discovery.  Set X3_SIMPLE_DISCOVERY=1 to enable.  This is required for
# cross-machine RViz over unicast initial peers: the discovery server's EDP
# forwarding between clients is broken (subscribers stay invisible to
# publishers, so data never flows), whereas Simple discovery + unicast peers
# delivers /tf, /scan, /map reliably.  When unset, behaviour is unchanged.
SIMPLE_DISCOVERY = os.environ.get('X3_SIMPLE_DISCOVERY', '').lower() in ('1', 'true', 'yes', 'on')

if ROS2_MODE:
    if SIMPLE_DISCOVERY:
        os.environ.pop('ROS_DISCOVERY_SERVER', None)
    else:
        os.environ['ROS_DISCOVERY_SERVER'] = _resolve_discovery_server()

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

YOLO_MODEL = find_model_path("yolo26n")
CONFIDENCE_THRESHOLD = 0.25
INFERENCE_SIZE = 640

# WebSocket port (must match GUI DEFAULT_PORT)
WS_PORT = 8081
# HTTP port for serving static GUI files (required for Web Workers to load)
HTTP_PORT = 8080

# =============================================================================
# GLOBAL STATE
# =============================================================================
ros_board = None
ros_bridge = None   # ROS2Bridge instance (always active unless --sim); used for map streaming
drive = None
lidar = None
camera = None
oak = None          # OakDCamera instance (stereo + depth + IMU); depth source when present
ina226 = None       # INA226 Battery Monitor
battery_estimator = None   # battery.BatteryEstimator — coulomb-counted SoC
_batt_cache_a = None       # last measured pack current (A), None = no sensor
_batt_cache_w = None       # last measured pack power (W)
_batt_cache_a_time = 0.0   # own clock: the voltage lane resets _batt_cache_time
_batt_state_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "config", "battery_state.json")
_batt_state_last_save = 0.0
oak_ros_pub = None  # OakRosPublisher — republishes /oak/* topics (--oak-ros-publish only)
model = None
oled = None
nav2_client = None    # Nav2Client instance (ROS2/sim modes only)
frontier_explorer = None  # FrontierExplorer instance (ROS2 mode only)
_gazebo_proc    = None   # subprocess handle when --sim auto-launches Gazebo
_ros2_stack_proc = None  # subprocess handle for x3_bringup (launched by default in ROS2 mode)
_mediamtx_proc = None    # subprocess handle for mediamtx (WebRTC camera, --webrtc-camera)
_slam_proc      = None   # subprocess handle when SLAM Toolbox is running
_octomap_proc   = None   # subprocess handle for 3D SLAM (octomap_server)
_shutting_down  = False  # set on SIGINT/SIGTERM/cleanup so motion_loop stops publishing
                         # to /cmd_vel before rclpy is torn down (avoids a publish-on-dead-
                         # context RCLError → C++ abort that orphaned the bringup stack)
velocity_estimator = None  # VelocityEstimator instance (EE244 project)
active_velocity_model_name = "velocity_mlp_v3"  # currently loaded torchscript velocity model
# v3 deployed 2026-09-04. Measured against v1 on one recorded capture replayed
# offline (src/score_velocity_models.py): closer to truth at range (+15% vs
# +33% at a 4.0 m gate) and far fewer phantoms on a static scene (raw p95
# 0.179 vs 0.728, 7 vs 69 frames >0.30 m/s). Its scaler is paired by name --
# do NOT point this at a model whose scaler_params_<suffix>.json is missing.
velocity_estimation_enabled = True  # A/B toggle: False = reactive, True = predictive
# Range limit for obstacles fed to the CBF, in metres. Deliberately SEPARATE
# from the estimator's own range: this one is a safety parameter and 1.8 m
# preserves the behaviour the CBF was tuned against. Raising it is a driving
# change and needs a driving test -- see F7, where a stale obstacle set drove
# the robot into a wall.
CBF_SPEED_RANGE_M = 1.8
_p2p_proc = None           # point-to-point test subprocess handle
_ab_test_proc = None       # A/B comparison test subprocess handle
_ab_test_mode = None       # A/B comparison test mode ("reactive" or "predictive")

detection_enabled = False
depth_enabled = False
stereo_enabled = False   # OAK-D left/right mono streaming to the GUI (off by default; bandwidth)
lidar_3d_scan_enabled = False
# Tilt sweep geometry.  +/-45 deg about level covers a 0.8 m tabletop from
# 0.61 m out and a 2.4 m ceiling from 2.2 m out, given the lidar at z=0.191.
# The step sets vertical resolution: 1.0 deg is ~5 cm at 3 m, 0.5 deg ~2.6 cm.
LIDAR_SWEEP_DEG = 45.0
LIDAR_SWEEP_STEP_DEG = 1.0
LIDAR_SWEEP_SETTLE_S = 0.15   # moving-flag startup grace for each 1 deg step
LIDAR_SWEEP_DWELL_S = 0.25    # leaves capture margin after a 156 ms scan window
LIDAR_SWEEP_SPEED_DEG_S = 40.0
LIDAR_SWEEP_ACCELERATION = 40  # deterministic, moderate XL430 profile ramp

# ── Sweep style, switchable from the GUI for A/B testing ────────────────────
# "step"       = step-and-stare (the production path; every ray in a dwell
#                shares one exact transform, so the cloud needs no deskewing)
# "continuous" = drive endpoint-to-endpoint without stopping, deliberately
#                smearing each 156 ms scan across speed*0.156 deg of pitch.
# The point of the toggle is to SEE that smear: at 40 deg/s a scan spans
# 6.25 deg, which at 3 m is ~33 cm of height error per revolution.
lidar_sweep_mode = "step"
lidar_sweep_speed_deg_s = LIDAR_SWEEP_SPEED_DEG_S
LIDAR_SWEEP_SPEED_MIN_DEG_S = 2.0
LIDAR_SWEEP_SPEED_MAX_DEG_S = 90.0
# lidar_3d_processor_node drops every cloud captured while the mount is
# travelling (require_settled). Runtime mode changes affect 3D only;
# failures stop the sweep instead of falsifying measurement state.
_sweep_settled_bypass = False   # Retained in telemetry for compatibility.

# How old /scan-derived obstacles may be before the CBF stops trusting them.
# /scan_raw measures ~6.4 Hz (156 ms), so 0.5 s allows ~3 missed scans.
OBSTACLE_STALE_S = 0.5

# Which websocket asked for the running sweep, so it can be cancelled when
# that client goes away, and a wall-clock backstop for every other way a
# sweep can be orphaned (crashed client, lost disable, forgotten tab).
_sweep_owner = None
_sweep_started_at = 0.0
LIDAR_SWEEP_MAX_S = 600.0

# ── Tilt sampler: the servo's read path, owned by ONE thread ───────────────
# The tilt angle used to be read inside lidar_scan_loop's 0.1 s asyncio cycle,
# which measured 6.4 Hz with a 429 ms worst gap (std 66 ms) -- at 45 deg/s that
# is 20 deg of pitch across one gap, and interpolating linearly through it puts
# ~54 cm of error at 3 m into any deskewed cloud, WORSE than the smear being
# corrected.  A dedicated thread at a fixed period fixes both the rate and the
# regularity, and stamps the sample at the instant it was measured rather than
# whenever the event loop got round to publishing it.
#
# dynamixel_sdk BUSY-WAITS on the read (measured 97% CPU for the whole round
# trip), so the poll rate costs real CPU: at 1 Mbps one read_pos is 1.49 ms, so
# 50 Hz is ~7.5% of a core.  At the old 57600 it was 6.14 ms and ~31%.  Do not
# raise TILT_SAMPLE_HZ without re-checking that arithmetic.
# TILT_SAMPLE_HZ is a TARGET, not what you get.  Measured in-process
# 2026-08-29: 38.4 Hz, period p50 20.0 ms (exactly on target) but p95 86 ms,
# p99 105 ms.  The median proves the thread's scheduling is fine; the tail is
# the read itself stretching from **1.49 ms standalone to 8.0 ms in-process**,
# because dynamixel_sdk's read is a Python-level busy-wait poll competing for
# the GIL with ~16 other working threads (oakd alone is 52% of a core).
#
# `sys.setswitchinterval(0.001)` was TRIED and does NOT help: 38.2 Hz, read
# 8.31 ms, tail actually worse (max 350 ms). Do not retry it.  The only real
# fix is moving this sampler into its own process, which is why the plan has
# that as a separate step -- see notes/ and the memory entry.
TILT_SAMPLE_HZ = 50.0
_servo_lock = threading.Lock()       # every touch of the Dynamixel bus
_tilt_lock = threading.Lock()        # guards _tilt_sample
_tilt_sample = {
    "counts": None,      # raw servo counts, None until the first good read
    "deg": 0.0,          # signed degrees about the calibrated level
    "moving": False,     # the XL430's Moving flag (only polled when needed)
    "t": 0.0,            # time.monotonic() midpoint of the read
    "reads": 0,
    "errors": 0,
}
# read_moving costs a second round trip (1.49 -> 2.87 ms) and only the
# step-and-stare state machine needs it, so lidar_scan_loop turns it on for
# exactly as long as it has an outstanding goal.  In continuous mode the mount
# never stops and the flag is meaningless, so it stays off.
_tilt_need_moving = False
_tilt_gate_moving = False   # what to publish as the /scan gate's "discard" flag


# Wall-clock stamp of the last non-zero MANUAL motion command (set in
# _enqueue_motion).  -inf so a fresh process is never considered "just driven".
_last_teleop_motion_t = float('-inf')
# How long after a teleop command the mount stays locked out.  Joystick input
# arrives at 20-50 Hz, so anything above ~0.2 s makes a continuous drive read as
# continuously moving; 1.5 s also covers the gap between D-pad taps and lets the
# chassis actually come to rest before a station is captured.
TELEOP_TILT_LOCKOUT_S = 1.5


def _sweep_config_payload(note=None):
    """Current sweep style + the derived numbers the GUI shows as a hint."""
    scan_s = 0.156   # one YDLidar revolution at ~6.4 Hz
    smear_deg = lidar_sweep_speed_deg_s * scan_s
    return {
        "type": "sweep_config",
        "mode": lidar_sweep_mode,
        "speed_deg_s": round(lidar_sweep_speed_deg_s, 1),
        "speed_min": LIDAR_SWEEP_SPEED_MIN_DEG_S,
        "speed_max": LIDAR_SWEEP_SPEED_MAX_DEG_S,
        "step_deg": LIDAR_SWEEP_STEP_DEG,
        "dwell_s": LIDAR_SWEEP_DWELL_S,
        # What the mode actually costs, so the GUI can state it rather than
        # making the operator remember it.
        "smear_deg": round(smear_deg, 2) if lidar_sweep_mode == "continuous" else 0.0,
        "smear_cm_at_3m": round(300.0 * math.tan(math.radians(smear_deg)), 1)
                          if lidar_sweep_mode == "continuous" else 0.0,
        "settled_bypass": _sweep_settled_bypass,
        "scanning": lidar_3d_scan_enabled,
        "note": note,
    }


async def _broadcast_sweep_config(note=None):
    payload = json.dumps(_sweep_config_payload(note))
    for ws in list(connected_clients):
        try:
            await ws.send(payload)
        except Exception:
            pass


async def _set_processor_require_settled(value):
    """Turn lidar_3d_processor_node's moving-scan gate on or off at runtime.

    Returns (ok, detail). A failure stops the sweep; measurement state must
    never be falsified to bypass a failed parameter update.

    Done over the CLI rather than an rclpy parameter client on purpose: the
    processor runs in the bringup stack as a separate process (sometimes
    composed), and `ros2 param set` works the same either way.
    """
    cmd = ["ros2", "param", "set", "/lidar_3d_processor_node",
           "require_settled", "true" if value else "false"]
    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT)
        out, _ = await asyncio.wait_for(proc.communicate(), timeout=8.0)
        text = (out or b"").decode(errors="replace").strip()
        # `ros2 param set` exits 0 even when it cannot find the node, so the
        # exit code alone is not enough -- it prints "Set parameter successful"
        # only on the real thing.
        if proc.returncode == 0 and "successful" in text.lower():
            return True, text
        return False, text or ("exit %s" % proc.returncode)
    except asyncio.TimeoutError:
        proc.kill()
        await proc.communicate()
        return False, "timed out (is lidar_3d_processor_node running?)"
    except Exception as e:
        return False, str(e)


async def _apply_sweep_gate():
    """Make the processor's moving-scan gate match the current sweep mode.

    Called both when the mode changes and at the start of every sweep -- the
    node can be restarted (or the server can be) independently of this
    process, and an out-of-date gate is invisible until a map comes out empty
    (continuous) or suspiciously clean-looking (step with the gate stuck off).
    Returns (ok, detail).
    """
    global _sweep_settled_bypass, lidar_3d_scan_enabled, lidar_sweep_mode
    ok, detail = await _set_processor_require_settled(lidar_sweep_mode == "step")
    _sweep_settled_bypass = False
    if not ok:
        lidar_3d_scan_enabled = False
        lidar_sweep_mode = "step"
        logger.error("Sweep stopped: could not configure processor gate: %s", detail)
        await _set_processor_require_settled(True)
    return ok, detail


def _tilt_nav_conflict():
    """Why a tilt sweep and robot motion must not overlap, or None if they may.

    A tilted scan plane makes lidar_3d_processor_node gate /scan off, so
    slam_toolbox and AMCL stop receiving data while Nav2's costmaps go stale
    and the robot is still free to drive.  Measured on the robot 2026-08-22:
    during a 50 s sweep only 52 of 391 scans reached /scan, all of them from
    the level part of the sweep.

    This is the ONE interlock that is safety-critical, so it is checked in
    both directions -- a sweep refuses to start while the robot is moving, and
    a sweep in progress is aborted the moment motion begins.

    FOUR arms, not three.  The first three cover the autonomous drivers (Nav2,
    auto-drive, frontier exploration); the fourth covers a human on the
    joystick, which for a long time bypassed this check entirely.  Manual
    driving during a sweep is the worst case of the lot: it gates /scan off
    while the robot moves AND it breaks the stationary-station assumption that
    the whole step-and-stare design rests on, so every point inserted during
    the drive is smeared by chassis motion on top of the tilt.
    """
    if nav2_client is not None and nav2_client.is_busy():
        return "Nav2 has an active goal"
    if is_auto_driving:
        return "auto-drive is running"
    if frontier_explorer is not None and frontier_explorer.is_exploring():
        return "frontier exploration is running"
    if time.monotonic() - _last_teleop_motion_t < TELEOP_TILT_LOCKOUT_S:
        return "the robot is being driven manually"
    return None
lidar_enabled = False
is_auto_driving = False
last_detections = []
active_model_name = "yolo26n"

# Current motor powers (tank-drive representation for GUI readout)
current_left_power = 0.0
current_right_power = 0.0

motors_enabled = True

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
MOTION_WATCHDOG_TIMEOUT = 0.50

# asyncio.Queue for decoupled motion commands — created in main() so it runs
# inside the event loop.  Handlers enqueue (vx, vy, omega) tuples; motion_loop()
# drains the queue at 100 Hz and calls drive.move().
motion_queue = None
servo_stats = {"voltage": "--", "state": "Free"}

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
    Adapter that drives the robot over ROS2 topics and relays sensor data.

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
        from sensor_msgs.msg import Image
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry, OccupancyGrid
        from std_msgs.msg import Float32
        from sensor_msgs.msg import LaserScan

        import sys
        import os
        sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from src.cbf_filter import HolonomicCBFFilter

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import JointState

        rclpy.init(args=None)
        self._node = Node('x3_ws_bridge')
        self._lock = threading.Lock()
        self._latest_frame = None          # cv2 BGR ndarray from Gazebo RGB camera
        self._latest_depth = None          # cv2 BGR ndarray from Gazebo depth camera
        self._latest_raw_depth = None      # np.ndarray containing raw depth values in meters
        self._pose_m = {"x": 0.0, "y": 0.0, "theta": 0.0}  # metres + radians
        self._twist = {"vx": 0.0, "vy": 0.0, "wz": 0.0}   # body velocity m/s, rad/s
        self._voltage = 12.0               # volts, updated by /voltage subscriber
        self._occupancy_grid: dict | None = None  # latest OccupancyGrid info dict for frontier explorer
        # TWO publishers, on purpose.
        #
        # `joint_pub` -> /lidar_tilt/joint_states is the tilt mount's own topic.
        # lidar_3d_processor_node gates /scan on it and must NOT read the merged
        # /joint_states, where anything publishing `lidar_tilt_joint: 0.0` reads
        # as a fresh, level measurement forever.
        #
        # `full_joint_pub` -> /joint_states feeds robot_state_publisher DIRECTLY,
        # replacing the joint_state_publisher `source_list` relay that used to
        # sit in between.  Measured on the robot 2026-08-29, that relay:
        #   * pinned TF at 10.0 Hz however fast it was fed (10/25/50/100 Hz in
        #     -> 8.9/10.0/10.0/10.0 Hz out), and reached only 15.7-18 Hz even at
        #     `rate:=100`, while burning 28% of a core to do it;
        #   * RE-STAMPED ~34% of what it emitted with its own timer's time
        #     instead of the measurement time (proved by feeding it joint states
        #     stamped 10 s in the past: 66% came out old, 34% came out "now").
        # Both are fatal for per-ray deskew, which interpolates TF between the
        # stamps on either side of a scan.  Publishing here instead makes this
        # process the single authority at its own rate with honest stamps.
        #
        # It carries all four wheel joints at 0.0 as well, exactly as
        # joint_state_publisher did, so removing that node changes nothing for
        # the wheel TF.  (`driver_node` also publishes /joint_states, but under
        # the R2's Ackermann joint names, of which only back_left/back_right
        # exist in the X3 URDF -- robot_state_publisher merges by name and
        # ignores the rest, so there is no fight over lidar_tilt_joint.)
        self.joint_pub = self._node.create_publisher(
            JointState, '/lidar_tilt/joint_states', 10)
        self.full_joint_pub = self._node.create_publisher(
            JointState, '/joint_states', 10)
        # safe_distance MUST stay ABOVE the _scan_cb min-range gate (0.12 m). The CBF
        # only brakes hard as distance -> safe_distance; if safe_distance were below the
        # gate, a real obstacle would be filtered out (mistaken for the chassis) before
        # the brake engaged and the robot would drive straight through it. 0.30 m stops
        # the robot with the wall still visible. Lower toward ~0.15 m for tighter
        # doorways (at the cost of a thinner stop margin). This whole range-gate
        # workaround goes away once the Lidar is raised above the chassis.
        self.cbf = HolonomicCBFFilter(safe_distance=0.30, gamma=1.0)
        self._latest_obstacles = []
        # When that list was last REFRESHED.  Without this the CBF cannot
        # tell a live obstacle from one frozen by a /scan dropout.
        self._obstacles_stamp = 0.0

        # SLAM Toolbox publishes /map as TRANSIENT_LOCAL; match so late-joining still gets the map
        _map_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Nav2 costmap topics use VOLATILE durability
        _costmap_qos = QoSProfile(depth=1,
                                  reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)

        self._node.create_subscription(Image,          '/camera/image_raw', self._image_cb,  1)
        self._node.create_subscription(Image,          '/camera/depth_image', self._depth_cb, 1)
        self._node.create_subscription(Image,          '/camera/depth/image_raw', self._depth_cb, 1)
        self._node.create_subscription(Odometry,       '/odom',             self._odom_cb,  10)
        self._node.create_subscription(Odometry,       '/odom_raw',         self._odom_cb,  10)
        self._node.create_subscription(Float32,        '/voltage',          self._voltage_cb, 10)
        self._node.create_subscription(OccupancyGrid,  '/map',              self._map_cb,    _map_qos)
        from rclpy.qos import qos_profile_sensor_data
        self._node.create_subscription(LaserScan,      '/scan',             self._scan_cb,   qos_profile_sensor_data)
        self._cmd_vel_pub = self._node.create_publisher(Twist, '/cmd_vel', 10)

        # Publishers for pedestrian tracking (EE244 Project)
        from geometry_msgs.msg import PoseArray
        from visualization_msgs.msg import MarkerArray
        from x3_msgs.msg import PedestrianArray
        self._pedestrian_pub = self._node.create_publisher(PoseArray, '/pedestrian_poses', 10)
        self._pedestrian_marker_pub = self._node.create_publisher(MarkerArray, '/pedestrian_markers', 10)
        self._pedestrian_state_pub = self._node.create_publisher(PedestrianArray, '/pedestrian_states', 10)

        # Staleness tracking for odom (Idea 225) and depth (Idea 230)
        self._odom_stamp = 0.0
        self._last_depth_write_time = 0.0

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()
        logger.info("ROS2Bridge: spinning — subscribed /camera/image_raw /odom /voltage /map /scan, publishing /cmd_vel")

    def _scan_cb(self, msg):
        import math as _math
        obstacles = []
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.12 < r < 1.0:
                # Transform from laser_link (yaw=180 deg) to base_link
                # x_base = -x_laser + x_offset, y_base = -y_laser
                x_laser = r * _math.cos(angle)
                y_laser = r * _math.sin(angle)
                x = -x_laser + 0.0435
                y = -y_laser
                obstacles.append((x, y))
            angle += msg.angle_increment
        with self._lock:
            self._latest_obstacles = obstacles
            self._obstacles_stamp = time.monotonic()

    def _image_cb(self, msg):
        """Convert sensor_msgs/Image (RGB_INT8 from Fortress) to cv2 BGR ndarray."""
        import numpy as np, cv2
        global _shared_bgr_array
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if _shared_bgr_array is not None and msg.height == 480 and msg.width == 640:
            np.copyto(_shared_bgr_array, bgr)
            with self._lock:
                self._latest_frame = _shared_bgr_array
        else:
            with self._lock:
                self._latest_frame = bgr

    def get_frame(self):
        """Return latest camera frame as BGR ndarray, or None. Matches AstraCamera API."""
        with self._lock:
            f = self._latest_frame
        return f if f is not None else None

    def _depth_cb(self, msg):
        """Convert depth image to colourised BGR uint8.
        Handles both 32FC1 (meters, Gazebo) and 16UC1/mono16 (millimeters, physical robot)."""
        self._last_depth_write_time = time.monotonic()  # Idea 230: record arrival time for staleness gate
        import numpy as np, cv2
        global _shared_depth_array
        try:
            # Check if image is 16-bit (millimeters) or 32-bit (meters)
            is_16bit = "16" in msg.encoding or "mono16" in msg.encoding
            if is_16bit:
                # 16UC1 / mono16 (millimeters)
                arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
                # Convert millimeters to meters
                arr_meters = arr.astype(np.float32) / 1000.0
            else:
                # 32FC1 (meters)
                arr_meters = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)

            # Replace invalid/zero/nan values with a far distance (5.0m)
            # so they do not corrupt the minimum depth (near object) search.
            arr_meters_clean = np.where((arr_meters <= 0.1) | np.isnan(arr_meters), 5.0, arr_meters)
            arr_meters_clean = np.clip(arr_meters_clean, 0.3, 5.0)

            # Dynamic min-max scaling: closest valid point maps to 255 (bright white),
            # furthest maps to 0 (dark).
            min_d = np.min(arr_meters_clean)
            max_d = np.max(arr_meters_clean)
            if max_d > min_d:
                norm = (255.0 * (1.0 - (arr_meters_clean - min_d) / (max_d - min_d))).astype(np.uint8)
            else:
                norm = np.zeros_like(arr_meters_clean, dtype=np.uint8)

            coloured = cv2.applyColorMap(norm, cv2.COLORMAP_BONE)
            if _shared_depth_array is not None and msg.height == 480 and msg.width == 640:
                np.copyto(_shared_depth_array, arr_meters)
                with self._lock:
                    self._latest_depth = coloured
                    self._latest_raw_depth = _shared_depth_array
            else:
                with self._lock:
                    self._latest_depth = coloured
                    self._latest_raw_depth = arr_meters
        except Exception as exc:
            logger.error(f"ROS2Bridge: failed to decode depth frame: {exc}")

    def get_depth_frame(self):
        """Return latest colourised depth frame as BGR ndarray, or None. Matches AstraCamera API."""
        with self._lock:
            f = self._latest_depth
        return f if f is not None else None

    def get_raw_depth_frame(self):
        """Return latest raw depth frame as float32 ndarray (in meters), or None."""
        with self._lock:
            f = self._latest_raw_depth
        return f if f is not None else None

    def get_depth_frame_age(self) -> float:
        """Return seconds since the last depth frame was received. Returns inf if never received.

        Used by VelocityEstimator to gate depth-based inference on frame freshness (Idea 230).
        """
        if self._last_depth_write_time == 0.0:
            return float('inf')
        return time.monotonic() - self._last_depth_write_time

    def _odom_cb(self, msg):
        """Extract position (metres), yaw (radians), and body twist from nav_msgs/Odometry."""
        self._odom_stamp = time.monotonic()  # Idea 225: record arrival time for staleness guard
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
        import math as _math
        q = msg.info.origin.orientation
        origin_yaw = _math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._occupancy_grid = {
                "data":       np.array(msg.data, dtype=np.int8),
                "width":      msg.info.width,
                "height":     msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x":   msg.info.origin.position.x,
                "origin_y":   msg.info.origin.position.y,
                "origin_yaw": origin_yaw,
            }
    def get_occupancy_grid(self) -> dict | None:
        """Return the latest occupancy grid info dict, or None if not yet received."""
        with self._lock:
            return dict(self._occupancy_grid) if self._occupancy_grid else None

    def get_pose_m(self) -> dict:
        """Return the robot pose in metres: {x, y, theta}."""
        with self._lock:
            return dict(self._pose_m)

    def get_pose_cm(self) -> dict:
        """Return robot pose in centimetres — matches YDLidarDriver interface
        so the broadcast_loop's hasattr(lidar, 'get_pose_cm') check succeeds."""
        with self._lock:
            return {
                "x":     self._pose_m["x"] * 100.0,
                "y":     self._pose_m["y"] * 100.0,
                "theta": self._pose_m["theta"],
            }

    def move(self, vx: float, vy: float, omega: float):
        from geometry_msgs.msg import Twist
        global velocity_estimation_enabled, velocity_estimator
        
        with self._lock:
            obstacles = list(self._latest_obstacles)
            obst_age = time.monotonic() - self._obstacles_stamp
            
        if velocity_estimation_enabled and velocity_estimator is not None:
            try:
                estimates = velocity_estimator.get_estimates()
                for est in estimates:
                    speed = est.get("speed", 0.0)
                    # SAFETY GATE. This used to live inside the estimator, which
                    # meant the range limit the CBF needs also silenced telemetry
                    # and made every accuracy measurement wrong (a walker at
                    # 1.5-3.5 m read -30% because 82% of its frames were forced
                    # to 0.0). The estimator now reports its full range and the
                    # limit is applied HERE, at the only safety-critical
                    # consumer, so driving behaviour is unchanged: a track beyond
                    # this range used to arrive with speed 0.0 and fail the 0.15
                    # test, and now it is rejected outright. Same obstacles.
                    if est.get("z", 0.0) > CBF_SPEED_RANGE_M:
                        continue
                    if speed > 0.15:
                        ox = est.get("z", est.get("y", 0.0))
                        oy = -est.get("x", 0.0)
                        rx = float(ox)
                        ry = float(oy)
                        rvx = est.get("vx", 0.0)
                        rvy = est.get("vy", 0.0)
                        for t in [0.5, 1.0, 1.5]:
                            obstacles.append((rx + rvx * t, ry + rvy * t))
            except Exception as e:
                logger.warning(f"Failed to add predictive obstacles: {e}")
        
        # CBF safety filter. Its proactive repulsion INVENTS velocity from a zero
        # command, so it is only ever safe on FRESH obstacles -- and _scan_cb only
        # refreshes them when a /scan message arrives.
        #
        # A 3D tilt sweep gates /scan off for its whole tilted portion BY DESIGN
        # (lidar_3d_processor_node, so slam_toolbox and AMCL never see a pitched
        # scan plane). That freezes _latest_obstacles, and repelling from the
        # frozen set drove the robot forward at 30 Hz until it hit a wall and kept
        # pushing -- a frozen list can never reveal the wall being approached.
        # Reported from the GUI 2026-08-23.
        if lidar_3d_scan_enabled:
            # The mount is sweeping: the robot is supposed to be a stationary
            # scanning station anyway, and every step-and-stare guarantee depends
            # on it not moving. Hold still, rotation included.
            safe_vx, safe_vy, omega = 0.0, 0.0, 0.0
        elif obst_age > OBSTACLE_STALE_S:
            # Blind. Pass the operator's own command through untouched, but never
            # let the CBF ADD motion nobody asked for.
            safe_vx, safe_vy = vx, vy
            if (vx or vy) and not getattr(self, '_warned_stale_obst', False):
                self._warned_stale_obst = True
                logger.warning(
                    "CBF disabled: /scan obstacles are %.1f s stale. Driving "
                    "UNPROTECTED until /scan resumes.", obst_age)
        else:
            self._warned_stale_obst = False
            # Robot is at (0,0) in its local frame, matching the obstacle coords.
            safe_vx, safe_vy = self.cbf.filter_velocity(vx, vy, (0.0, 0.0), obstacles)

        msg = Twist()
        msg.linear.x  = float(safe_vx)    * self.LINEAR_SCALE
        msg.linear.y  = float(safe_vy)    * self.LINEAR_SCALE
        msg.angular.z = float(omega) * self.ANGULAR_SCALE
        self._cmd_vel_pub.publish(msg)

    def stop(self):
        """A REAL stop: a zero Twist straight to /cmd_vel, bypassing the CBF.

        This used to be `self.move(0, 0, 0)`, which routed the stop through the
        CBF's proactive repulsion and could therefore produce MOTION -- which is
        why the robot has previously wandered off during battery swaps. A stop
        must never be conditional on obstacle data.
        """
        try:
            from geometry_msgs.msg import Twist
            self._cmd_vel_pub.publish(Twist())
        except Exception as e:
            logger.warning("stop(): direct zero-Twist publish failed: %s", e)

    def cleanup(self):
        import rclpy
        import time
        try:
            logger.info("ROS2Bridge: sending stop command before shutdown...")
            self.stop()
            time.sleep(0.2)  # Allow DDS to flush stop packet
            self._node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    def publish_pedestrians(self, estimates):
        from geometry_msgs.msg import PoseArray, Pose
        from visualization_msgs.msg import MarkerArray, Marker
        from x3_msgs.msg import PedestrianArray, PedestrianState
        import math

        pose_array = PoseArray()
        pose_array.header.stamp.sec = 0
        pose_array.header.stamp.nanosec = 0
        pose_array.header.frame_id = 'base_link'

        marker_array = MarkerArray()

        state_array = PedestrianArray()
        state_array.header = pose_array.header

        for est in estimates:
            x = est.get("x", 0.0)
            y = est.get("y", 0.0)
            z = est.get("z", 1.0)
            vx = est.get("vx", 0.0)
            vy = est.get("vy", 0.0)
            tid = est.get("id", 0)

            # Robot frame (REP-103): x=forward (camera z), y=left (camera -x)
            pose = Pose()
            pose.position.x = float(z)
            pose.position.y = -float(x)
            pose.position.z = 0.0
            
            yaw = math.atan2(vy, vx)
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            pose_array.poses.append(pose)

            state = PedestrianState()
            state.id = int(tid)
            state.position.x = float(z)
            state.position.y = -float(x)
            state.position.z = 0.0
            state.velocity.x = float(vy) # math.atan2(vy, vx) uses vy for y-component and vx for x-component
            state.velocity.y = float(vx) # Actually atan2(y, x). So if yaw = atan2(vy, vx), then vy is Y, vx is X? Wait.
            # Usually atan2 is atan2(y, x). So vy is y, vx is x.
            # In camera frame, x is right, y is down, z is forward.
            # The pose maps z to x (forward) and -x to y (left).
            # So velocity vx, vy might map similarly, but we will just pass vx, vy as given by the estimator.
            state.velocity.x = float(vx)
            state.velocity.y = float(vy)
            state.velocity.z = 0.0
            state.radius = 0.3
            state_array.pedestrians.append(state)

            # Marker Arrow
            marker = Marker()
            marker.header = pose_array.header
            marker.ns = "pedestrian_velocities"
            marker.id = int(tid)
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000
            marker.pose.position.x = float(z)
            marker.pose.position.y = -float(x)
            marker.pose.position.z = 0.2
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)

            speed = math.hypot(vx, vy)
            marker.scale.x = float(max(0.3, speed))
            marker.scale.y = 0.08
            marker.scale.z = 0.08

            marker.color.a = 0.8
            if speed > 0.3:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            marker_array.markers.append(marker)

            # Text Marker
            text_marker = Marker()
            text_marker.header = pose_array.header
            text_marker.ns = "pedestrian_labels"
            text_marker.id = int(tid) + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 500000000
            text_marker.pose.position.x = float(z)
            text_marker.pose.position.y = -float(x)
            text_marker.pose.position.z = 0.7
            text_marker.scale.z = 0.2
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f"ID:{tid} {speed:.2f}m/s"
            marker_array.markers.append(text_marker)

            # Trajectory Marker
            if speed > 0.1:
                from geometry_msgs.msg import Point
                traj_marker = Marker()
                traj_marker.header = pose_array.header
                traj_marker.ns = "pedestrian_trajectories"
                traj_marker.id = int(tid) + 2000
                traj_marker.type = Marker.LINE_STRIP
                traj_marker.action = Marker.ADD
                traj_marker.lifetime.sec = 0
                traj_marker.lifetime.nanosec = 500000000
                traj_marker.scale.x = 0.05
                traj_marker.color.a = 0.8
                traj_marker.color.r = 1.0
                traj_marker.color.g = 1.0
                traj_marker.color.b = 0.0
                
                for t in [0.0, 0.4, 0.8, 1.2, 1.6, 2.0]:
                    p = Point()
                    p.x = float(z) + float(vx) * t
                    p.y = -float(x) + float(vy) * t
                    p.z = 0.0
                    traj_marker.points.append(p)
                marker_array.markers.append(traj_marker)

            # Person Volume Marker (Bounding Box)
            vol_marker = Marker()
            vol_marker.header = pose_array.header
            vol_marker.ns = "pedestrian_volumes"
            vol_marker.id = int(tid) + 3000
            vol_marker.type = Marker.CUBE
            vol_marker.action = Marker.ADD
            vol_marker.lifetime.sec = 0
            vol_marker.lifetime.nanosec = 500000000
            vol_marker.pose.position.x = float(z)
            vol_marker.pose.position.y = -float(x)
            vol_marker.pose.position.z = 0.85
            vol_marker.pose.orientation.w = 1.0
            vol_marker.scale.x = 0.5
            vol_marker.scale.y = 0.5
            vol_marker.scale.z = 1.7
            vol_marker.color.a = 0.3
            vol_marker.color.r = 0.0
            vol_marker.color.g = 1.0
            vol_marker.color.b = 1.0
            marker_array.markers.append(vol_marker)

        self._pedestrian_pub.publish(pose_array)
        self._pedestrian_marker_pub.publish(marker_array)
        self._pedestrian_state_pub.publish(state_array)


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


def _launch_mediamtx():
    """Launch mediamtx (WebRTC server) for the --webrtc-camera color feed.

    mediamtx spawns the Astra->H.264 ffmpeg encoder on-demand (see mediamtx.yml
    runOnDemand), so nothing touches the camera until the GUI actually watches.
    Returns/stores the Popen handle so cleanup() can stop it.
    """
    global _mediamtx_proc
    poc_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'webrtc_poc')
    binary = os.path.join(poc_dir, 'mediamtx')
    config = os.path.join(poc_dir, 'mediamtx.yml')
    if not os.path.exists(binary):
        logger.error(f"WebRTC: mediamtx binary not found at {binary} — color feed unavailable")
        return
    try:
        _mediamtx_proc = subprocess.Popen(
            [binary, config], cwd=poc_dir,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        logger.info(f"WebRTC: mediamtx started (pid {_mediamtx_proc.pid}) — GUI camera on :8889/astra")
    except Exception as e:
        logger.error(f"WebRTC: failed to launch mediamtx: {e}")


def _launch_ros2_stack():
    """Auto-launch the X3 hardware bringup (drivers + EKF, no SLAM) as a subprocess.

    Mirrors _launch_gazebo() — strips conda from PATH, sources ROS2 + workspace,
    then starts x3_bringup.launch.py.  SLAM is started separately via the GUI.
    Returns the Popen handle so cleanup() can SIGTERM the process group.
    """
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    install_setup = os.path.join(ws_root, 'install', 'setup.bash')

    child_env = os.environ.copy()
    if SIMPLE_DISCOVERY:
        child_env.pop('ROS_DISCOVERY_SERVER', None)
    else:
        child_env['ROS_DISCOVERY_SERVER'] = os.environ.get('ROS_DISCOVERY_SERVER', '127.0.0.1:11811')
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
    if SIMPLE_DISCOVERY:
        child_env.pop('ROS_DISCOVERY_SERVER', None)
    else:
        child_env['ROS_DISCOVERY_SERVER'] = os.environ.get('ROS_DISCOVERY_SERVER', '127.0.0.1:11811')
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


def _launch_octomap(use_sim_time: bool = False, frame_id: str = 'odom'):
    """Start octomap_server (3D mapping) as a background subprocess.

    Consumes /pointcloud_raw from lidar_3d_processor_node and publishes
    /octomap_binary, /octomap_binary_throttled (1 Hz, for RViz over WiFi) and
    /projected_map.  Pair it with the GUI's "3D Scan" toggle: the scan tilts the
    mount and produces the cloud, this turns that cloud into a map.

    frame_id defaults to `odom`, NOT `map`.  `map` only exists while AMCL or
    slam_toolbox is actually localized; without map->odom octomap_server's
    message filter drops 100% of clouds while logging only "queue is full",
    never naming the missing frame.  For a stationary sweep odom is exactly
    equivalent and never jumps.

    Same clean-environment pattern as _launch_slam(), and the same
    preexec_fn=os.setsid so the stop handler's os.killpg() reaches the whole
    launch tree rather than just the `ros2 launch` parent.
    Returns the Popen handle.
    """
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    install_setup = os.path.join(ws_root, 'install', 'setup.bash')

    child_env = os.environ.copy()
    if SIMPLE_DISCOVERY:
        child_env.pop('ROS_DISCOVERY_SERVER', None)
    else:
        child_env['ROS_DISCOVERY_SERVER'] = os.environ.get(
            'ROS_DISCOVERY_SERVER', '127.0.0.1:11811')
    child_env.setdefault('DISPLAY', ':0')
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

    st_arg = 'use_sim_time:=true' if use_sim_time else 'use_sim_time:=false'
    cmd = (
        f'source /opt/ros/humble/setup.bash && '
        f'source {install_setup} && '
        f'ros2 launch yahboomcar_nav x3_octomap.launch.py '
        f'frame_id:={frame_id} {st_arg}'
    )
    log_path = '/tmp/octomap_launch.log'
    log_file = open(log_path, 'w')
    proc = subprocess.Popen(
        ['bash', '-c', cmd],
        stdout=log_file,
        stderr=log_file,
        preexec_fn=os.setsid,
        env=child_env,
    )
    logger.info(
        f"OctoMap 3D mapping launched (pid {proc.pid}, frame={frame_id}) "
        f"— log: {log_path}")
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
    if SIMPLE_DISCOVERY:
        child_env.pop('ROS_DISCOVERY_SERVER', None)
    else:
        child_env['ROS_DISCOVERY_SERVER'] = os.environ.get('ROS_DISCOVERY_SERVER', '127.0.0.1:11811')
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


def _save_octomap(name: str, full: bool = False) -> tuple[bool, str]:
    """Write the live octree to disk as .bt (binary) or .ot (full).

    Until this existed NOTHING in the pipeline ever wrote an octree: octomap
    ran, RViz drew it, and the map died with the process.  src/octomap_viewer.py
    browses .bt/.ot files that had no way of being produced.

    Two non-obvious things about the ROS 2 saver, neither of which matches the
    ROS 1 documentation everyone quotes:

      * it takes the path as a PARAMETER (`-p octomap_path:=...`), not as
        `-f path` or a positional argument.  Both of those are accepted
        silently and then fail with "Invalid file name or extension:" and an
        empty name.
      * it pulls the tree from the `/octomap_binary` SERVICE, not the topic of
        the same name.  If octomap_server is not running it simply blocks on
        "Waiting for service..." forever, which is why this call is bounded by
        a timeout rather than left to finish.
    """
    if not name.endswith(('.bt', '.ot')):
        name += '.ot' if full else '.bt'
    name = os.path.basename(name)          # no path traversal from the GUI

    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    maps_dir = os.path.join(ws_root, 'src', 'yahboomcar_nav', 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    out_path = os.path.join(maps_dir, name)

    if _octomap_proc is None or _octomap_proc.poll() is not None:
        return False, ("3D mapping is not running -- start it first, or the "
                       "saver blocks forever waiting on /octomap_binary")

    install_setup = os.path.join(ws_root, 'install', 'setup.bash')
    child_env = os.environ.copy()
    if SIMPLE_DISCOVERY:
        child_env.pop('ROS_DISCOVERY_SERVER', None)
    else:
        child_env['ROS_DISCOVERY_SERVER'] = os.environ.get(
            'ROS_DISCOVERY_SERVER', '127.0.0.1:11811')
    child_env['PATH'] = ':'.join(
        p for p in child_env.get('PATH', '').split(':')
        if 'conda' not in p.lower())

    cmd = (
        f'source /opt/ros/humble/setup.bash && '
        f'source {install_setup} && '
        f'ros2 run octomap_server octomap_saver_node --ros-args '
        f'-p octomap_path:={out_path}'
        + (' -p full:=true' if full else '')
    )
    try:
        result = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True, text=True, timeout=30, env=child_env,
        )
    except subprocess.TimeoutExpired:
        return False, ("Timed out waiting for the /octomap_binary service -- "
                       "is octomap_server actually up?")
    except Exception as exc:
        return False, str(exc)

    # The saver exits 0 even when it refuses the filename, so trust the file.
    if os.path.isfile(out_path) and os.path.getsize(out_path) > 0:
        size_kb = os.path.getsize(out_path) / 1024.0
        logger.info("3D map saved: %s (%.1f kB)", out_path, size_kb)
        return True, "Saved '%s' (%.1f kB)" % (name, size_kb)
    err = (result.stderr or result.stdout).strip().splitlines()
    return False, (err[-1] if err else "octomap_saver wrote no file")


def _list_octomaps() -> list:
    """Every saved octree, newest first, for the GUI's 3D map list."""
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    maps_dir = os.path.join(ws_root, 'src', 'yahboomcar_nav', 'maps')
    out = []
    try:
        for f in os.listdir(maps_dir):
            if f.endswith(('.bt', '.ot')):
                fp = os.path.join(maps_dir, f)
                out.append({"name": f,
                            "size_kb": round(os.path.getsize(fp) / 1024.0, 1),
                            "mtime": os.path.getmtime(fp)})
    except OSError:
        return []
    return sorted(out, key=lambda m: m["mtime"], reverse=True)


def _encode_mapu(grid: dict) -> bytes | None:
    """Encode an OccupancyGrid dict as a MAPU binary WebSocket frame.

    Frame layout (big-endian):
      4B  tag 'MAPU'
      4B  width  (uint32)
      4B  height (uint32)
      4B  resolution (float32)
      4B  origin_x   (float32)
      4B  origin_y   (float32)
      4B  origin_yaw (float32)
      N×1B pixel data (uint8): unknown=128, free=255, occupied=0
    """
    try:
        data   = grid["data"]          # np.int8 array, row-0 = south (ROS convention)
        w      = grid["width"]
        h      = grid["height"]
        res    = grid["resolution"]
        ox     = grid["origin_x"]
        oy     = grid["origin_y"]
        oyaw   = grid["origin_yaw"]

        # Map OccupancyGrid values → grayscale pixels
        pixels = np.empty(len(data), dtype=np.uint8)
        pixels[data < 0]   = 128   # unknown
        pixels[data == 0]  = 255   # free
        pixels[data > 0]   = 0     # occupied

        # Flip rows: ROS row-0 is at the bottom; canvas row-0 is at the top
        pixels = pixels.reshape(h, w)[::-1, :].reshape(-1)

        header = struct.pack('>4sIIffff', b'MAPU', w, h, res, ox, oy, oyaw)
        return header + pixels.tobytes()
    except Exception as exc:
        logger.warning(f"_encode_mapu failed: {exc}")
        return None


def initialize_hardware():
    global ros_board, ros_bridge, drive, lidar, camera, oak, ina226, oak_ros_pub, model, oled, _gazebo_proc
    global nav2_client, _ros2_stack_proc, frontier_explorer
    global velocity_estimator, active_velocity_model_name
    global _shared_bgr_shm, _shared_depth_shm, _shared_bgr_array, _shared_depth_array

    logger.info("="*50)
    logger.info("Initializing Yahboom X3 Hardware")
    logger.info("="*50)

    # Allocate Shared Memory blocks for camera and depth frames (Idea 145)
    try:
        # 640x480 BGR frame = 921600 bytes
        _shared_bgr_shm = shared_memory.SharedMemory(name="x3_bgr_frame", create=True, size=921600)
        _shared_bgr_array = np.ndarray((480, 640, 3), dtype=np.uint8, buffer=_shared_bgr_shm.buf)
        logger.info("Shared Memory: BGR frame buffer allocated")
    except FileExistsError:
        _shared_bgr_shm = shared_memory.SharedMemory(name="x3_bgr_frame", create=False)
        _shared_bgr_array = np.ndarray((480, 640, 3), dtype=np.uint8, buffer=_shared_bgr_shm.buf)
        logger.info("Shared Memory: BGR frame buffer reattached")
    except Exception as e:
        logger.warning(f"Shared Memory: BGR allocation failed: {e}")

    try:
        # 480x640 float32 depth frame = 1228800 bytes
        _shared_depth_shm = shared_memory.SharedMemory(name="x3_depth_frame", create=True, size=1228800)
        _shared_depth_array = np.ndarray((480, 640), dtype=np.float32, buffer=_shared_depth_shm.buf)
        logger.info("Shared Memory: Depth frame buffer allocated")
    except FileExistsError:
        _shared_depth_shm = shared_memory.SharedMemory(name="x3_depth_frame", create=False)
        _shared_depth_array = np.ndarray((480, 640), dtype=np.float32, buffer=_shared_depth_shm.buf)
        logger.info("Shared Memory: Depth frame buffer reattached")
    except Exception as e:
        logger.warning(f"Shared Memory: Depth allocation failed: {e}")

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
        # ROS2 bridge mode: subscribe to /odom, /map, /camera, /voltage; publish /cmd_vel.
        # /scan is served directly to the browser via foxglove_bridge (port 8765).
        # Auto-launches x3_bringup.launch.py (hardware drivers: Mcnamu_driver_X3,
        # base_node_X3, IMU filter, EKF, YDLidar) so /cmd_vel drives the real motors.
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

        # Opt-in only (--auto-nav2-map). This used to fire unconditionally on a
        # hardcoded RealLab2.yaml "for velocity estimator testing", which meant
        # every session started with someone else's map on /map and with AMCL
        # publishing map->odom. slam_toolbox publishes map->odom as well, so
        # starting SLAM produced two publishers fighting over one transform:
        # RViz could not transform odom-framed octomap data to a `map` fixed
        # frame, its filter queue filled, and the 3D map lagged by seconds.
        if AUTO_NAV2_MAP:
            if not os.path.isfile(AUTO_NAV2_MAP):
                logger.error("--auto-nav2-map %s does not exist; not launching "
                             "Nav2", AUTO_NAV2_MAP)
            else:
                logger.info("Auto-launching Nav2 with AMCL on %s (requested via "
                            "--auto-nav2-map). Do NOT start SLAM while this "
                            "runs -- both publish map->odom.", AUTO_NAV2_MAP)
                nav2_client.launch_nav2(use_sim_time=False,
                                        map_path=AUTO_NAV2_MAP, slam=False)
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

    if not SIM_MODE:
        logger.info("Initializing INA226 Battery Monitor on bus 7...")
        ina226 = INA226BatteryMonitor(i2c_bus=7)
        _init_battery_estimator()

    # 3. Camera — direct USB in both direct-hardware and --ros2 modes.
    #    Only --sim uses ROS2Bridge.get_frame() (Gazebo publishes /camera/image_raw).
    #    With --webrtc-camera we DON'T open the Astra here — mediamtx's ffmpeg owns it
    #    and serves it over WebRTC (full framerate); the base64 path stays empty.
    if not SIM_MODE and not WEBRTC_CAMERA:
        logger.info("Initializing Camera...")
        # In ROS2 mode the depth stream is owned by the orbbec_depth service,
        # so direct USB depth stream initialization is disabled to prevent conflicts.
        _enable_direct_depth = not ROS2_MODE
        camera = AstraCamera(width=640, height=480, sim_mode=False, enable_depth=_enable_direct_depth)
    elif WEBRTC_CAMERA:
        logger.info("Camera: --webrtc-camera set — Astra released for the WebRTC/mediamtx feed")
        _launch_mediamtx()

    # 3b. OAK-D Lite — stereo + on-device metric depth + 6-axis IMU (no color; the
    #     Astra keeps RGB). When present it becomes the depth source for the
    #     broadcast loop and VelocityEstimator, superseding the Astra/orbbec depth.
    #     The driver degrades gracefully (getters return None, worker retries) if
    #     the device is unplugged, so the server still runs without it.
    if not SIM_MODE and OAK_ENABLED:
        try:
            from oakd_driver import OakDCamera
            # Auto-enable on-device YOLO spatial detection if the plain blob exists
            # (extract it from the .superblob once via blobs/extract_superblob.py).
            _blob_dir = Path(__file__).parent / "blobs" / "yolo26n"
            _blob = _blob_dir / "yolo26n.blob"
            _cfg = _blob_dir / "config.json"
            _spatial_blob = str(_blob) if (OAK_SPATIAL and _blob.exists()) else None
            _spatial_cfg = str(_cfg) if (_spatial_blob and _cfg.exists()) else None
            # mono_fps targets high on-device stereo/depth rate for perception; the
            # actual rate is gated by the USB link (needs USB3/SUPER to approach it).
            oak = OakDCamera(mono_fps=80, spatial_blob=_spatial_blob, spatial_config=_spatial_cfg,
                             conf_threshold=0.35, accel_hz=500, gyro_hz=400)
            oak.start()
            logger.info("OAK-D Lite: driver started (stereo + depth + IMU"
                        + (" + spatial detection)" if _spatial_blob else ")"))
        except Exception as e:
            logger.error(f"OAK-D Lite: init failed: {e}")
            oak = None

        # DepthAI opens the device exclusively, so nothing outside this process can
        # see the OAK streams. --oak-ros-publish republishes them as sensor_msgs so
        # they can be bagged (record_bag.sh) or viewed in RViz/Foxglove.
        if oak is not None and OAK_ROS_PUBLISH and ros_bridge is not None:
            try:
                from oakd_ros_publisher import OakRosPublisher
                oak_ros_pub = OakRosPublisher(ros_bridge._node, oak,
                                              rate_hz=OAK_ROS_RATE,
                                              publish_stereo=OAK_ROS_STEREO)
                oak_ros_pub.start()
            except Exception as e:
                logger.error(f"OAK-D ROS publisher: init failed: {e}")
                oak_ros_pub = None
        elif OAK_ROS_PUBLISH and ros_bridge is None:
            logger.warning("--oak-ros-publish ignored: no ROS2 bridge in this mode")

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
        _class_list = [f"{k}:{v}" for k, v in sorted((model.names or {}).items())]
        logger.info(f"YOLO classes ({len(_class_list)}): {', '.join(_class_list) or '(none loaded)'}")
    except Exception as e:
        logger.error(f"YOLO Load Failed: {e}")


    # 7. OLED Display (SSD1306 on I2C bus 1 — Jetson Orin pins 3/5)
    logger.info("Initializing OLED display...")
    oled = OLEDDisplay(i2c_port=7, i2c_address=0x3C, sim_mode=SIM_MODE)
    oled.show(["X3 Robot", "Starting...", ""])

    # 8. Velocity Estimator (EE244 Project)
    try:
        model_name = active_velocity_model_name
        model_path = str(Path(__file__).parent.resolve() / f"{model_name}.torchscript")
        logger.info(f"Initializing VelocityEstimator with model: {model_path}...")
        # Depth source priority: OAK-D Lite (when available) > ROS2 bridge (/camera/depth,
        # ROS2/sim) > direct Astra USB.
        def get_current_depth_source():
            if oak is not None and getattr(oak, "available", False):
                return oak
            if ROS2_MODE or SIM_MODE:
                return ros_bridge
            return camera
        
        # Callback to retrieve EKF pose and twist (Idea 1 & 11)
        def get_robot_pose_and_twist():
            if ros_bridge is not None:
                # Idea 225: reject stale odom — prevents frozen pose from corrupting velocity estimation
                if time.monotonic() - ros_bridge._odom_stamp > 0.5:
                    return None
                pose = ros_bridge.get_pose_m()
                with ros_bridge._lock:
                    twist = dict(ros_bridge._twist)
                return {"pose": pose, "twist": twist}
            return None

        # Callback to retrieve latest visual YOLO detections (Idea 146)
        def get_latest_yolo_detections():
            global last_detections
            return list(last_detections)

        velocity_estimator = VelocityEstimator(get_current_depth_source, lidar, robot_pose_fn=get_robot_pose_and_twist, model_path=model_path, detections_fn=get_latest_yolo_detections)
        velocity_estimator.start()
        logger.info("VelocityEstimator started successfully")
    except Exception as e:
        logger.error(f"Failed to start VelocityEstimator: {e}")

    logger.info("="*50)
    logger.info("Initialization Complete")
    logger.info("="*50)

def cleanup():
    global _p2p_proc, _ab_test_proc, _shared_bgr_shm, _shared_depth_shm, _shutting_down
    _shutting_down = True   # stop motion_loop publishing before rclpy is torn down
    logger.info("Cleaning up...")
    if _shared_bgr_shm is not None:
        try:
            _shared_bgr_shm.close()
            _shared_bgr_shm.unlink()
            logger.info("Shared Memory: BGR buffer released")
        except Exception as e:
            logger.warning(f"Error releasing BGR shared memory: {e}")
        _shared_bgr_shm = None

    if _shared_depth_shm is not None:
        try:
            _shared_depth_shm.close()
            _shared_depth_shm.unlink()
            logger.info("Shared Memory: Depth buffer released")
        except Exception as e:
            logger.warning(f"Error releasing Depth shared memory: {e}")
        _shared_depth_shm = None

    if _p2p_proc is not None and _p2p_proc.poll() is None:
        logger.info("Stopping P2P Test subprocess...")
        try:
            _p2p_proc.terminate()
            _p2p_proc.wait(timeout=2.0)
        except Exception:
            try:
                _p2p_proc.kill()
            except Exception:
                pass
        _p2p_proc = None

    if _ab_test_proc is not None and _ab_test_proc.poll() is None:
        logger.info("Stopping A/B Test subprocess...")
        try:
            _ab_test_proc.terminate()
            _ab_test_proc.wait(timeout=2.0)
        except Exception:
            try:
                _ab_test_proc.kill()
            except Exception:
                pass
        _ab_test_proc = None
    if oak_ros_pub is not None:
        try:
            oak_ros_pub.stop()
        except Exception as e:
            logger.error(f"Failed to stop OAK-D ROS publisher: {e}")
    if oak is not None:
        try:
            logger.info("Stopping OAK-D Lite driver...")
            oak.cleanup()
        except Exception as e:
            logger.error(f"Failed to stop OAK-D driver: {e}")
    if velocity_estimator is not None:
        try:
            logger.info("Stopping VelocityEstimator...")
            velocity_estimator.stop()
        except Exception as e:
            logger.error(f"Failed to stop VelocityEstimator: {e}")
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
            _pgid = os.getpgid(_ros2_stack_proc.pid)
            os.killpg(_pgid, signal.SIGTERM)
            # Guarantee it's gone — a lingering Mcnamu_driver_X3 keeps the Rosmaster
            # serial open and a fresh bringup then fights it (robot won't move).
            try:
                _ros2_stack_proc.wait(timeout=5.0)
            except Exception:
                logger.warning("ROS2 stack did not exit on SIGTERM — sending SIGKILL")
                try:
                    os.killpg(_pgid, signal.SIGKILL)
                except Exception:
                    pass
        except Exception:
            pass
    if _mediamtx_proc is not None:
        logger.info("Shutting down mediamtx (WebRTC camera)...")
        try:
            os.killpg(os.getpgid(_mediamtx_proc.pid), signal.SIGTERM)
        except Exception:
            pass
    if _slam_proc is not None:
        logger.info("Shutting down SLAM Toolbox...")
        try:
            os.killpg(os.getpgid(_slam_proc.pid), signal.SIGTERM)
        except Exception:
            pass
    if _octomap_proc is not None:
        logger.info("Shutting down OctoMap 3D SLAM...")
        try:
            os.killpg(os.getpgid(_octomap_proc.pid), signal.SIGTERM)
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

def _enqueue_motion(vx: float, vy: float, omega: float, instant: bool = False):
    """Enqueue a motion command; drops the oldest entry when full so stale commands never accumulate."""
    global _last_teleop_motion_t
    # Every manual path funnels through here (joystick, D-pad, tank power) --
    # Nav2, auto-drive and the FSM do NOT.  Stamping non-zero commands here is
    # therefore exactly "a human is driving right now", which is the fourth arm
    # of the tilt interlock (see _tilt_nav_conflict).  Zeros are stops, so they
    # must not count or releasing the joystick would keep the sweep locked out.
    if vx or vy or omega:
        _last_teleop_motion_t = time.monotonic()
    if motion_queue is None:
        return
    if motion_queue.full():
        try:
            motion_queue.get_nowait()
        except asyncio.QueueEmpty:
            pass
    try:
        motion_queue.put_nowait((vx, vy, omega, instant))
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


def _resolve_map_path(yaml_name: str) -> str | None:
    """
    Turn a map name from the GUI into an absolute YAML path inside the maps
    directory. Returns None if empty or if the file does not exist.

    Only the basename is honoured, so a crafted name can't escape maps/.
    Nav2's map_server and x3_nav2.launch.py both need a full path — a bare
    filename would be resolved against the launcher's cwd and silently fail.
    """
    if not yaml_name:
        return None
    name = os.path.basename(yaml_name)
    if not name.endswith('.yaml'):
        name += '.yaml'
    path = os.path.join(_maps_dir(), name)
    return path if os.path.isfile(path) else None


async def handle_client(websocket):
    global detection_enabled, depth_enabled, stereo_enabled, lidar_enabled, is_auto_driving
    global current_left_power, current_right_power
    global model, active_model_name
    global _gazebo_proc, nav2_client
    global detection_enabled, depth_enabled, stereo_enabled, lidar_enabled, is_auto_driving
    global velocity_estimator, active_velocity_model_name, velocity_estimation_enabled
    global motors_enabled
    global _p2p_proc, _ab_test_proc, _ab_test_mode

    logger.info("Client connected")
    connected_clients.add(websocket)
    if camera:
        camera._has_clients = True  # P7: allow capture loop to store frames

    # Tell the client what mode the server is running in
    _mode = "sim" if SIM_MODE else "ros2"
    await websocket.send(json.dumps({
        "type": "hello",
        "mode": _mode,
        "active_velocity_model": active_velocity_model_name,
        "webrtc_camera": WEBRTC_CAMERA,   # GUI shows a WebRTC <video> instead of base64 frames
        "build": _build_info(),
    }))
    # A fresh tab must not show "step" while the mount is actually running a
    # continuous test sweep started from another tab.
    await websocket.send(json.dumps(_sweep_config_payload()))

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
                    _enqueue_motion(vx, vy, omega, instant=True)

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
                    if ros_board: ros_board.set_motor(0, 0, 0, 0)
                    _enqueue_motion(0.0, 0.0, 0.0)

                elif msg_type == "toggle_detection":
                    detection_enabled = data.get("enabled", False)
                    logger.info(f"Detection: {detection_enabled}")

                elif msg_type == "toggle_motors":
                    motors_enabled = data.get("enabled", True)
                    logger.info(f"Motors: {'enabled' if motors_enabled else 'disabled'}")

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

                elif msg_type == "toggle_stereo":
                    global stereo_enabled
                    stereo_enabled = data.get("enabled", False)
                    logger.info(f"OAK-D stereo streaming: {stereo_enabled}")

                elif msg_type == "toggle_3d_scan":
                    global lidar_3d_scan_enabled
                    want_scan = data.get("enabled", False)
                    conflict = _tilt_nav_conflict() if want_scan else None
                    if conflict:
                        logger.warning(
                            "3D scan REFUSED: %s. Tilting the lidar gates "
                            "/scan off, which would leave Nav2 driving on a "
                            "costmap that has stopped updating.", conflict)
                        await websocket.send(json.dumps({
                            "type": "3d_scan_status",
                            "enabled": False,
                            "refused": True,
                            "reason": conflict,
                        }))
                    else:
                        lidar_3d_scan_enabled = want_scan
                        global _sweep_owner, _sweep_started_at
                        _sweep_owner = websocket if want_scan else None
                        _sweep_started_at = time.monotonic() if want_scan else 0.0
                        logger.info(f"Lidar 3D Scan: {lidar_3d_scan_enabled}")
                        await websocket.send(json.dumps({
                            "type": "3d_scan_status",
                            "enabled": lidar_3d_scan_enabled,
                            "refused": False,
                        }))

                elif msg_type == "set_sweep_config":
                    # Testing knob: swap the 3D sweep between step-and-stare
                    # and a constant-speed nod, and set that speed, so the
                    # mapping degradation can be watched side by side.
                    global lidar_sweep_mode, lidar_sweep_speed_deg_s
                    note = None
                    if "speed_deg_s" in data:
                        try:
                            lidar_sweep_speed_deg_s = min(
                                LIDAR_SWEEP_SPEED_MAX_DEG_S,
                                max(LIDAR_SWEEP_SPEED_MIN_DEG_S,
                                    float(data["speed_deg_s"])))
                        except (TypeError, ValueError):
                            pass
                    if "mode" in data:
                        want_mode = ("continuous"
                                     if data.get("mode") == "continuous"
                                     else "step")
                        if want_mode != lidar_sweep_mode:
                            lidar_sweep_mode = want_mode
                            # Continuous means never settled, so the
                            # processor's moving-scan gate has to come off or
                            # the octree stays empty.  Put it back on the way
                            # out -- leaving it off would silently degrade
                            # every later step-and-stare map.
                            ok, detail = await _apply_sweep_gate()
                            if not ok:
                                logger.warning("set_sweep_config failed: %s; "
                                               "sweep stopped and step mode restored", detail)
                                note = ("processor param not set (%s)"
                                        % detail)
                    logger.info("3D sweep config: mode=%s speed=%.1f deg/s "
                                "(settled_bypass=%s)",
                                lidar_sweep_mode, lidar_sweep_speed_deg_s,
                                _sweep_settled_bypass)
                    await _broadcast_sweep_config(note)

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
                        # Resolve to an absolute path: the launch file's map:=
                        # argument is interpreted relative to the launcher cwd.
                        map_f  = _resolve_map_path(data.get("map") or "")
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
                    if lidar_3d_scan_enabled:
                        # The other half of the interlock.  Without this you can
                        # click a goal mid-sweep and Nav2 plans against frozen
                        # costmaps.
                        logger.warning(
                            "set_nav_goal REFUSED: a 3D tilt scan is running. "
                            "Turn 3D Scan off (the mount returns to level) "
                            "before navigating.")
                        await websocket.send(json.dumps({
                            "type": "nav_goal_refused",
                            "reason": "3D tilt scan in progress",
                        }))
                    elif nav2_client:
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

                # Hot-swap the map AMCL localises against, mid-drive.
                elif msg_type == "set_amcl_map":
                    map_name = data.get("map") or ""
                    map_path = _resolve_map_path(map_name)
                    if nav2_client is None:
                        ok, msg_text = False, "Requires --ros2 or --sim mode"
                    elif map_path is None:
                        ok, msg_text = False, f'Map "{map_name}" not found'
                    else:
                        # Service round-trip: keep it off the event loop.
                        ok, msg_text = await asyncio.get_running_loop().run_in_executor(
                            None, nav2_client.load_map, map_path)
                    await websocket.send(json.dumps({
                        "type": "amcl_map_result",
                        "success": ok,
                        "map": os.path.basename(map_path) if map_path else map_name,
                        "msg": msg_text,
                    }))
                    if ok:
                        # Redraw the nav canvas with the map now in use
                        map_data = _load_map_data(os.path.basename(map_path))
                        if map_data:
                            await websocket.send(json.dumps(
                                {"type": "map_data", **map_data}))

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
                elif msg_type == "toggle_frontier":
                    enabled = data.get("enabled", False)
                    if enabled:
                        nav_running = (nav2_client and
                                       nav2_client.get_status().get("nav2_running", False))
                        if not nav_running:
                            await websocket.send(json.dumps({
                                "type": "frontier_explore_result",
                                "success": False,
                                "msg": "Nav2 must be running before starting Auto-Explore. Launch Nav2 first.",
                            }))
                        elif frontier_explorer and ROS2_MODE:
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
                                "msg": "Frontier exploration requires ROS2 mode",
                            }))
                    else:
                        if frontier_explorer:
                            frontier_explorer.stop()
                            await websocket.send(json.dumps({
                                "type": "frontier_explore_result",
                                "success": True,
                                "msg": "Frontier exploration stopped",
                            }))

                elif msg_type == "start_slam":
                    global _slam_proc
                    if _slam_proc is None or _slam_proc.poll() is not None:
                        _slam_proc = _launch_slam(use_sim_time=SIM_MODE)
                        logger.info("SLAM Toolbox launched")
                        await websocket.send(json.dumps({
                            "type": "slam_started",
                            "success": True,
                            "msg": "SLAM Toolbox launching...",
                        }))
                    else:
                        logger.info("SLAM already running")
                        await websocket.send(json.dumps({
                            "type": "slam_started",
                            "success": True,
                            "msg": "SLAM already running",
                        }))

                elif msg_type == "stop_slam":
                    if _slam_proc is not None and _slam_proc.poll() is None:
                        try:
                            os.killpg(os.getpgid(_slam_proc.pid), signal.SIGTERM)
                            logger.info("SLAM Toolbox stopped")
                        except Exception as exc:
                            logger.warning(f"Failed to stop SLAM: {exc}")
                        _slam_proc = None

                elif msg_type == "start_3d_slam":
                    global _octomap_proc
                    if _octomap_proc is None or _octomap_proc.poll() is not None:
                        _octomap_proc = _launch_octomap(use_sim_time=SIM_MODE)
                        logger.info("OctoMap 3D Mapping launched")
                        await websocket.send(json.dumps({
                            "type": "slam_3d_started",
                            "success": True,
                            "msg": "3D SLAM launching...",
                        }))
                    else:
                        logger.info("3D SLAM already running")
                        await websocket.send(json.dumps({
                            "type": "slam_3d_started",
                            "success": True,
                            "msg": "3D SLAM already running",
                        }))

                elif msg_type == "stop_3d_slam":
                    if _octomap_proc is not None and _octomap_proc.poll() is None:
                        try:
                            os.killpg(os.getpgid(_octomap_proc.pid), signal.SIGTERM)
                            logger.info("OctoMap 3D Mapping stopped")
                        except Exception as exc:
                            logger.warning(f"Failed to stop 3D SLAM: {exc}")
                        _octomap_proc = None

                elif msg_type == "request_live_map":
                    if ros_bridge is not None:
                        grid = ros_bridge.get_occupancy_grid()
                        if grid:
                            frame_bytes = _encode_mapu(grid)
                            if frame_bytes:
                                await websocket.send(frame_bytes)

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

                elif msg_type == "save_3d_map":
                    map_name = (data.get("name") or "octomap").strip() or "octomap"
                    ok, msg_text = await asyncio.to_thread(
                        _save_octomap, map_name, bool(data.get("full", False)))
                    await websocket.send(json.dumps({
                        "type": "save_3d_map_result",
                        "success": ok,
                        "message": msg_text,
                        "name": map_name,
                    }))
                    if ok:
                        await websocket.send(json.dumps(
                            {"type": "octomap_list", "maps": _list_octomaps()}))

                elif msg_type == "list_3d_maps":
                    await websocket.send(json.dumps(
                        {"type": "octomap_list", "maps": _list_octomaps()}))

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

                elif msg_type == "set_velocity_estimation":
                    enabled = data.get("enabled", False)
                    velocity_estimation_enabled = enabled
                    if velocity_estimator is not None:
                        try:
                            velocity_estimator.estimation_enabled = enabled
                        except Exception as e:
                            logger.warning(f"set_velocity_estimation error: {e}")
                    logger.info(f"Velocity estimation: {'enabled' if enabled else 'disabled'}")
                    await websocket.send(json.dumps({
                        "type": "velocity_estimation_status",
                        "enabled": enabled
                    }))

                elif msg_type == "set_velocity_model":
                    model_name = data.get("model")
                    if model_name in ("velocity_mlp", "velocity_mlp_finetuned",
                                      "velocity_mlp_v2", "velocity_mlp_v3"):
                        new_path = str(Path(__file__).parent.resolve() / f"{model_name}.torchscript")
                        if os.path.exists(new_path):
                            logger.info(f"Switching velocity estimator model to {new_path}")
                            try:
                                if velocity_estimator:
                                    velocity_estimator.stop()
                                    velocity_estimator.model_path = new_path
                                    velocity_estimator._load_model()
                                    velocity_estimator.start()
                                active_velocity_model_name = model_name
                                await websocket.send(json.dumps({
                                    "type": "velocity_model_changed",
                                    "model": model_name,
                                    "success": True
                                }))
                            except Exception as e:
                                logger.error(f"Failed to switch velocity model: {e}")
                                await websocket.send(json.dumps({
                                    "type": "velocity_model_changed",
                                    "model": model_name,
                                    "success": False,
                                    "error": str(e)
                                }))
                        else:
                            logger.warning(f"Velocity model file not found: {new_path}")
                            await websocket.send(json.dumps({
                                "type": "velocity_model_changed",
                                "model": model_name,
                                "success": False,
                                "error": "File not found"
                            }))

                elif msg_type == "start_p2p_test":
                    if _p2p_proc is None or _p2p_proc.poll() is not None:
                        if nav2_client and not (nav2_client._nav2_proc and nav2_client._nav2_proc.poll() is None):
                            logger.info("Auto-launching Nav2 with AMCL on RealLab2.yaml for test...")
                            nav2_client.launch_nav2(use_sim_time=SIM_MODE, map_path='/home/jetson/x3_ws/src/yahboomcar_nav/maps/RealLab2.yaml', slam=False)
                        logger.info("Starting Point-to-Point Test...")
                        script_path = str(Path(__file__).parent.resolve() / "point_to_point_test.py")
                        
                        child_env = os.environ.copy()
                        child_env['PATH'] = ':'.join(
                            p for p in child_env.get('PATH', '').split(':')
                            if 'conda' not in p.lower()
                        )
                        if 'PYTHONPATH' in child_env:
                            child_env['PYTHONPATH'] = ':'.join(
                                p for p in child_env['PYTHONPATH'].split(':')
                                if 'conda' not in p.lower()
                            )

                        _p2p_proc = subprocess.Popen(
                            [sys.executable, script_path],
                            env=child_env
                        )
                    await websocket.send(json.dumps({
                        "type": "p2p_test_status",
                        "status": "running"
                    }))

                elif msg_type == "cancel_p2p_test":
                    if _p2p_proc is not None and _p2p_proc.poll() is None:
                        logger.info("Canceling Point-to-Point Test...")
                        try:
                            _p2p_proc.terminate()
                            _p2p_proc.wait(timeout=2.0)
                        except Exception as exc:
                            logger.warning(f"Failed to terminate P2P subprocess: {exc}")
                            try:
                                _p2p_proc.kill()
                            except Exception:
                                pass
                        _p2p_proc = None
                        _enqueue_motion(0.0, 0.0, 0.0)
                        
                    await websocket.send(json.dumps({
                        "type": "p2p_test_status",
                        "status": "idle"
                    }))

                elif msg_type == "start_ab_test":
                    mode = data.get("mode", "reactive")  # "reactive" | "predictive"
                    distance = data.get("distance", 4.0)
                    repeat = data.get("repeat", False)
                    if _ab_test_proc is None or _ab_test_proc.poll() is not None:
                        if nav2_client and not (nav2_client._nav2_proc and nav2_client._nav2_proc.poll() is None):
                            logger.info("Auto-launching Nav2 with AMCL on RealLab2.yaml for test...")
                            nav2_client.launch_nav2(use_sim_time=SIM_MODE, map_path='/home/jetson/x3_ws/src/yahboomcar_nav/maps/RealLab2.yaml', slam=False)
                        logger.info(f"Starting A/B comparison test (mode={mode}, distance={distance}, repeat={repeat})...")
                        script_path = str(Path(__file__).parent.resolve() / "ab_comparison_test.py")

                        child_env = os.environ.copy()
                        child_env['PATH'] = ':'.join(
                            p for p in child_env.get('PATH', '').split(':')
                            if 'conda' not in p.lower()
                        )
                        if 'PYTHONPATH' in child_env:
                            child_env['PYTHONPATH'] = ':'.join(
                                p for p in child_env['PYTHONPATH'].split(':')
                                if 'conda' not in p.lower()
                            )

                        cmd_args = [sys.executable, script_path, "--mode", mode, "--distance", str(distance)]
                        if repeat:
                            cmd_args.append("--repeat")

                        _ab_test_proc = subprocess.Popen(
                            cmd_args,
                            env=child_env
                        )
                        _ab_test_mode = mode
                    await websocket.send(json.dumps({
                        "type": "ab_test_status",
                        "status": "running",
                        "mode": mode,
                    }))

                elif msg_type == "cancel_ab_test":
                    if _ab_test_proc is not None and _ab_test_proc.poll() is None:
                        logger.info("Canceling A/B comparison test...")
                        try:
                            _ab_test_proc.terminate()
                            _ab_test_proc.wait(timeout=2.0)
                        except Exception as exc:
                            logger.warning(f"Failed to terminate AB test subprocess: {exc}")
                            try:
                                _ab_test_proc.kill()
                            except Exception:
                                pass
                        _ab_test_proc = None
                        _enqueue_motion(0.0, 0.0, 0.0)
                    _ab_test_mode = None
                    await websocket.send(json.dumps({
                        "type": "ab_test_status",
                        "status": "idle",
                        "mode": None,
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
        # A sweep must not outlive the client that asked for it: while the
        # mount is tilted /scan is gated off, so an orphaned sweep blinds
        # slam_toolbox and AMCL and freezes the CBF's obstacle set.
        # Both names are already declared global earlier in this handler
        # (the toggle_3d_scan branch); re-declaring here is a SyntaxError,
        # because a global statement must precede every use in the scope.
        if _sweep_owner is websocket and lidar_3d_scan_enabled:
            lidar_3d_scan_enabled = False
            logger.warning("3D sweep CANCELLED: the client that started it "
                           "disconnected. Returning the mount to level.")
        if _sweep_owner is websocket:
            _sweep_owner = None
        logger.info("Client disconnected")

# =============================================================================
# MAIN BROADCAST LOOP
# =============================================================================

# --- System metrics -------------------------------------------------------
# Sampled on a background thread, never in the event loop: psutil.cpu_percent
# with an interval blocks, and a blocking call here stalls every client.
_sys_stats = {"cpu_per_core": [], "cpu_total": 0.0, "proc_cpu": 0.0,
              "proc_rss_mb": 0.0, "temp_c": None, "mem_pct": 0.0, "loadavg": 0.0}
_sys_stats_lock = threading.Lock()


_BUILD_INFO = None


def _build_info():
    """Identify which checkout is actually serving, cached after first call.

    The robot and the laptop have drifted repeatedly, and the failure mode is
    silent: an old GUI against a new server looks like a broken feature rather
    than a stale file.  Showing the rev in the header makes that a glance.
    """
    global _BUILD_INFO
    if _BUILD_INFO is not None:
        return _BUILD_INFO
    here = os.path.dirname(os.path.abspath(__file__))
    info = {"rev": "unknown", "dirty": False, "server_mtime": None}
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=here,
                             capture_output=True, text=True, timeout=5)
        if out.returncode == 0:
            info["rev"] = out.stdout.strip()
        st = subprocess.run(["git", "status", "--porcelain"], cwd=here,
                            capture_output=True, text=True, timeout=5)
        info["dirty"] = bool(st.returncode == 0 and st.stdout.strip())
    except Exception:
        pass
    try:
        info["server_mtime"] = int(os.path.getmtime(os.path.abspath(__file__)))
    except Exception:
        pass
    info["host"] = socket.gethostname()
    _BUILD_INFO = info
    return info


_THERMAL_ZONES = None


def _read_thermal_c():
    """Hottest thermal zone in Celsius, or None.

    psutil.sensors_temperatures() returns nothing on this Jetson, so read the
    sysfs zones directly.  Several Orin zones (cv0-cv2) exist but have no
    reading and yield an empty file -- those are skipped rather than treated
    as 0 C, which would make the max look healthy while hiding a real one.
    """
    global _THERMAL_ZONES
    if _THERMAL_ZONES is None:
        try:
            import glob
            _THERMAL_ZONES = sorted(glob.glob(
                "/sys/devices/virtual/thermal/thermal_zone*/temp"))
        except Exception:
            _THERMAL_ZONES = []
    best = None
    for path in _THERMAL_ZONES:
        try:
            with open(path) as fh:
                raw = fh.read().strip()
            if not raw:
                continue
            c = int(raw) / 1000.0
            if 0.0 < c < 150.0 and (best is None or c > best):
                best = c
        except Exception:
            continue
    return round(best, 1) if best is not None else None


def _system_snapshot():
    with _sys_stats_lock:
        return dict(_sys_stats)


def _system_sampler(period_s=2.0):
    """1/2 Hz snapshot of CPU, memory and thermals for the GUI diagnostics tile.

    Per-core rather than aggregate on purpose: the interesting failure on this
    machine is one core pinned at 100% while five idle, which an average of
    ~20% hides completely.
    """
    try:
        import psutil
    except Exception as e:
        logger.warning(f"psutil unavailable, system metrics disabled: {e}")
        return

    proc = psutil.Process()
    proc.cpu_percent(None)          # prime; the first call always returns 0.0
    psutil.cpu_percent(percpu=True)

    while not _shutting_down:
        try:
            per_core = psutil.cpu_percent(interval=period_s, percpu=True)
            vm = psutil.virtual_memory()
            temp = _read_thermal_c()
            snap = {
                "cpu_per_core": [round(c, 1) for c in per_core],
                "cpu_total": round(sum(per_core) / max(1, len(per_core)), 1),
                # >100% is correct and meaningful here: it means the server is
                # finally using more than one core.
                "proc_cpu": round(proc.cpu_percent(None), 1),
                "proc_rss_mb": round(proc.memory_info().rss / 1048576.0, 1),
                "mem_pct": round(vm.percent, 1),
                "loadavg": round(os.getloadavg()[0], 2),
                "temp_c": temp,
            }
            with _sys_stats_lock:
                _sys_stats.update(snap)
        except Exception as e:
            logger.debug(f"system sampler: {e}")
            time.sleep(period_s)


def _init_battery_estimator():
    """Stand up the coulomb-counted SoC gauge, restoring the persisted charge.

    Coulomb counting has no absolute reference of its own, so a restart that
    re-seeds from the voltage plateau throws the estimate away -- on this pack
    the plateau spans 5 mV across 85-98% SoC, so the reseed can be off by most
    of a charge.  Persisting charge_ah is what makes the gauge survive the
    service restarts this robot does constantly.
    """
    global battery_estimator
    try:
        import battery
    except Exception as e:
        logger.warning(f"battery module unavailable, SoC stays voltage-only: {e}")
        return

    initial = None
    try:
        with open(_batt_state_path) as fh:
            saved = json.load(fh)
        # A stale charge is worse than none: the pack may have been swapped or
        # charged with the robot off, and neither shows up in the integral.
        age_h = (time.time() - float(saved.get("saved_epoch", 0))) / 3600.0
        if 0.0 <= age_h <= 12.0:
            initial = float(saved["charge_ah"])
            logger.info(f"Battery: restored {initial:.2f} Ah ({age_h:.1f} h old)")
        else:
            logger.info(f"Battery: persisted charge is {age_h:.1f} h old, re-seeding")
    except FileNotFoundError:
        pass
    except Exception as e:
        logger.warning(f"Battery: could not restore charge: {e}")

    # No current sensor means no coulomb counting; battery.py wants capacity_ah
    # None in that case so it degrades to an honest OCV lookup rather than
    # integrating a fabricated current.
    has_current = ina226 is not None and ina226.get_current() is not None
    battery_estimator = battery.BatteryEstimator(
        capacity_ah=battery.PACK_CAPACITY_AH if has_current else None,
        initial_charge_ah=initial,
    )
    logger.info(f"Battery estimator ready (coulomb counting: {has_current})")


def _save_battery_state():
    """Persist charge_ah so the next restart does not lose the integral."""
    global _batt_state_last_save
    if battery_estimator is None or battery_estimator.charge_ah is None:
        return
    now = time.time()
    if now - _batt_state_last_save < 60.0:
        return
    _batt_state_last_save = now
    try:
        os.makedirs(os.path.dirname(_batt_state_path), exist_ok=True)
        tmp = _batt_state_path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump({"charge_ah": battery_estimator.charge_ah,
                       "saved_epoch": now}, fh)
        os.replace(tmp, _batt_state_path)   # atomic: a torn file reads as absent
    except Exception as e:
        logger.debug(f"Battery: could not persist charge: {e}")


async def broadcast_loop():
    global _cam_frame_count, _yolo_frame_count, _fps_last_time
    global fps_camera, fps_detection, last_detections, depth_enabled, lidar_enabled
    global _batt_cache_v, _batt_cache_time, _batt_cache_a, _batt_cache_w  # P9
    global _batt_cache_a_time
    global _ab_test_mode, stereo_enabled

    loop = asyncio.get_event_loop()
    _depth_cycle = 0  # throttle depth to ~10 fps (every other 20fps cycle)
    _loop_i = 0       # drives the ~2 Hz "readout_slow" lane (every 10th cycle)

    while True:
        if _shutting_down:
            break
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
                    # Downscale visualization to 320x240 before compressing (Idea 106)
                    annotated_downscaled = cv2.resize(annotated, (320, 240), interpolation=cv2.INTER_NEAREST)
                    _, buf = cv2.imencode('.jpg', annotated_downscaled, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    return dets, bytes(buf)
                last_detections, img_bytes = await loop.run_in_executor(None, _run_yolo)
                _yolo_frame_count += 1
            elif frame is not None:
                def _encode_frame():
                    # Downscale visualization to 320x240 before compressing (Idea 106)
                    frame_downscaled = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_NEAREST)
                    _, buf = cv2.imencode('.jpg', frame_downscaled, [cv2.IMWRITE_JPEG_QUALITY, 70])
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

            # 4. Depth frame — throttled to 10fps; encode also in executor (P1).
            #    Depth source priority: OAK-D Lite > ROS2 bridge (/camera/depth) > Astra USB.
            depth_str = ""
            _depth_cycle += 1
            # Prefer the OAK-D only while its link is live; otherwise fall back so the
            # GUI depth view keeps working if the OAK is unplugged.
            _oak_up = oak is not None and getattr(oak, "available", False)
            depth_source = oak if _oak_up else (ros_bridge if (ROS2_MODE or SIM_MODE) else camera)
            if depth_enabled and (_depth_cycle % 2 == 0):
                if depth_source is not None:
                    def _get_depth():
                        df = depth_source.get_depth_frame()
                        if df is None:
                            return ""
                        # Downscale depth visualization to 320x240 before compressing (Idea 106)
                        df_downscaled = cv2.resize(df, (320, 240), interpolation=cv2.INTER_NEAREST)
                        _, dbuf = cv2.imencode('.jpg', df_downscaled, [cv2.IMWRITE_JPEG_QUALITY, 60])
                        return base64.b64encode(dbuf).decode('utf-8')
                    depth_str = await loop.run_in_executor(None, _get_depth)

            # 4b. OAK-D stereo (left/right mono) — toggled off by default (bandwidth),
            #     throttled to the same 10fps cadence as depth. IMU is cheap and always sent.
            oak_left_str = ""
            oak_right_str = ""
            if stereo_enabled and oak is not None and (_depth_cycle % 2 == 0):
                def _get_stereo():
                    left, right = oak.get_stereo_frames()
                    out = ["", ""]
                    for i, gray in enumerate((left, right)):
                        if gray is None:
                            continue
                        small = cv2.resize(gray, (320, 200), interpolation=cv2.INTER_NEAREST)
                        _, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                        out[i] = base64.b64encode(buf).decode('utf-8')
                    return out
                oak_left_str, oak_right_str = await loop.run_in_executor(None, _get_stereo)
            oak_imu = oak.get_imu() if oak is not None else None
            # On-device YOLO spatial detections (3D positions), when the OAK NN is running.
            oak_detections = oak.get_spatial_detections() if (oak is not None and getattr(oak, "spatial_active", False)) else []

            # 5. Lidar points — served via Foxglove bridge (/scan topic); not sent here

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
                if now - _batt_cache_time >= 1.0:  # Throttle to 1Hz (Idea 67)
                    if ina226 is not None:
                        _batt_cache_v = ina226.get_voltage() or drive.get_battery_voltage()
                    else:
                        _batt_cache_v = drive.get_battery_voltage()
                    _batt_cache_time = now
                batt_v = _batt_cache_v
            elif ros_board:
                m1_enc, m2_enc, m3_enc, m4_enc = ros_board.get_motor_encoder()
                if now - _batt_cache_time >= 1.0:
                    if ina226 is not None:
                        _batt_cache_v = ina226.get_voltage() or ros_board.get_battery_voltage()
                    else:
                        _batt_cache_v = ros_board.get_battery_voltage()
                    _batt_cache_time = now
                batt_v = _batt_cache_v
            else:
                m1_enc = m2_enc = m3_enc = m4_enc = 0
                batt_v = 12.0

            # Pack current: measured across the INA226 shunt when the chip is
            # there.  The old `0.5 + avg_power*6.0` was a guess from motor duty
            # that ignored the Orin, the OAK and the lidar -- i.e. most of the
            # draw -- and it fed a gauge the GUI labelled in amps.
            if ina226 is not None and now - _batt_cache_a_time >= 1.0:
                _batt_cache_a = ina226.get_current()
                _batt_cache_w = ina226.get_power()
                _batt_cache_a_time = now
            batt_a = _batt_cache_a
            batt_w = _batt_cache_w
            batt_measured = batt_a is not None

            if not batt_measured:
                # Fall back to the duty-cycle guess, but flag it so the GUI can
                # show it as an estimate rather than a measurement.
                avg_pwr = (abs(current_left_power) + abs(current_right_power)) / 2.0
                batt_a  = 0.5 + (avg_pwr * 6.0)
                batt_w  = batt_v * batt_a

            batt_minutes = None
            if battery_estimator is not None:
                batt_pct = battery_estimator.update(
                    batt_v, batt_a if batt_measured else 0.0)
                batt_minutes = battery_estimator.minutes_remaining
                _save_battery_state()
            elif ina226 is not None:
                batt_pct = ina226.get_percentage(batt_v)
            else:
                batt_pct = max(0.0, min(100.0, (batt_v - 8.1) / (12.6 - 8.1) * 100.0))
            est_current = batt_a
            est_watts   = batt_w

            # Get velocity estimator predictions (EE244 Project)
            velocity_estimates = []
            if velocity_estimator is not None:
                try:
                    velocity_estimates = velocity_estimator.get_estimates()
                    if ROS2_MODE and ros_bridge is not None and velocity_estimates:
                        ros_bridge.publish_pedestrians(velocity_estimates)
                except Exception as e:
                    logger.error(f"Failed to get velocity estimates: {e}")

            # P3: send camera frame as a binary WebSocket message (raw JPEG, no base64)
            if img_bytes:
                websockets.broadcast(connected_clients, img_bytes)

            # 7b. Tilt mount — sampled at 50 Hz by _tilt_sampler but, until now,
            #     never sent anywhere.  The GUI had no way to see where the
            #     mount actually was, which is why a mount stuck off-level
            #     could only be diagnosed over SSH.
            with _tilt_lock:
                _ts = dict(_tilt_sample)
            tilt_state = {
                "deg":     round(_ts["deg"], 2),
                "counts":  _ts["counts"],
                "moving":  _ts["moving"],
                "reads":   _ts["reads"],
                "errors":  _ts["errors"],
                # Age of the newest good sample.  A rising age with a static
                # angle is the read_pos desync that has spiked TF by 117deg.
                "age_s":   round(time.monotonic() - _ts["t"], 2) if _ts["t"] else None,
                "scanning": lidar_3d_scan_enabled,
                # True while the sweep is gating /scan away from the CBF.  A
                # frozen obstacle set here is what drove the robot into a wall.
                "scan_gated": _tilt_gate_moving,
            }

            # 8. Fast readout lane (20 Hz): pose, motors, power, detections, depth.
            msg = {
                "type": "readout",
                "depth_image": depth_str,
                "oak_left": oak_left_str,
                "oak_right": oak_right_str,
                "oak_imu": oak_imu,
                "oak_detections": oak_detections,
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
                "detections": last_detections,
                "velocity_estimates": velocity_estimates,
                "battery": {"voltage": batt_v, "amps": est_current, "watts": est_watts},
                "power": {
                    "voltage":     batt_v,
                    "current":     est_current,
                    "power":       est_watts,
                    "battery_pct": batt_pct,
                    # measured=False means current/power are the duty-cycle
                    # guess, not the shunt.  The GUI must not present the two
                    # the same way.
                    "measured":    batt_measured,
                    "minutes_left": batt_minutes,
                },
                "tilt": tilt_state,
            }
            # Fast orjson serialization with fallback (Idea 87)
            websockets.broadcast(connected_clients, orjson_dumps(msg))

            # 9. Slow readout lane (~2 Hz): nav / SLAM / frontier / model / fps / test
            #    status. These are subprocess polls or dict-builds that don't need 20 Hz,
            #    so we compute and broadcast them only every 10th cycle.
            _loop_i += 1
            if _loop_i % 10 == 0:
                nav_status = nav2_client.get_status() if nav2_client else {"state": "UNAVAILABLE"}
                _ab_running = _ab_test_proc is not None and _ab_test_proc.poll() is None
                slow = {
                    "type": "readout_slow",
                    "nav_phase": nav_status["state"],
                    "nav": nav_status,
                    "slam_active": _slam_proc is not None and _slam_proc.poll() is None,
                    "slam_3d_active": _octomap_proc is not None and _octomap_proc.poll() is None,
                    "frontier": frontier_explorer.status() if frontier_explorer else None,
                    "active_model_name": active_model_name,
                    "active_velocity_model_name": active_velocity_model_name,
                    "p2p_test_running": _p2p_proc is not None and _p2p_proc.poll() is None,
                    "ab_test_running": _ab_running,
                    "ab_test_mode": _ab_test_mode if _ab_running else None,
                    "servo_stats": servo_stats if 'servo_stats' in globals() else {"voltage": "--", "state": "Free"},
                    "fps_camera": fps_camera,
                    "fps_detection": fps_detection,
                    # OAK-D stereo/depth capture rate — cheap cached read from the
                    # driver's worker thread (no capture-path impact).
                    "system": _system_snapshot(),
                    "fps_oak_depth": (oak.get_depth_fps()
                                      if (oak is not None and getattr(oak, "available", False))
                                      else 0.0),
                }
                websockets.broadcast(connected_clients, orjson_dumps(slow))

        await asyncio.sleep(0.05)  # 20 FPS cap

def _step_toward(current: float, target: float, accel: float, decel: float) -> float:
    """Advance current toward target by at most accel or decel per step."""
    diff = target - current
    is_decelerating = (target == 0.0) or (abs(target) < abs(current)) or (target * current < 0)
    step = decel if is_decelerating else accel
    if abs(diff) <= step:
        return target
    return current + (step if diff > 0 else -step)


_standalone_test_cache = (False, 0.0)   # (last_result, last_scan_monotonic)
_STANDALONE_TEST_SCAN_INTERVAL = 1.0    # seconds between full /proc scans

def _is_standalone_test_running() -> bool:
    # Scanning every process' cmdline is expensive (~260 /proc reads). motion_loop
    # calls this at ~30 Hz, so cache the result and only re-scan at ~1 Hz — an
    # externally-launched test appearing/disappearing within 1 s is harmless.
    global _standalone_test_cache
    now = time.monotonic()
    last_result, last_scan = _standalone_test_cache
    if now - last_scan < _STANDALONE_TEST_SCAN_INTERVAL:
        return last_result

    import psutil
    result = False
    try:
        for proc in psutil.process_iter(['cmdline']):
            cmd = proc.info.get('cmdline')
            if cmd:
                cmd_str = ' '.join(cmd)
                if 'ab_comparison_test.py' in cmd_str or 'point_to_point_test.py' in cmd_str:
                    if 'server_x3.py' not in cmd_str:
                        result = True
                        break
    except Exception:
        pass
    _standalone_test_cache = (result, now)
    return result


async def motion_loop():
    """Dedicated ~30 Hz motion command consumer with velocity ramping (Idea 111).

    Maintains a ramped velocity that smoothly tracks commanded targets.
    - Runs at ~30 Hz (0.033 s sleep) to reduce serial write CPU overhead vs 100 Hz.
    - Continuously publishes cmd_vel as an active keep-alive heartbeat.
    - Safety watchdog: zero targets if input silent for too long.
    """
    global _p2p_proc, _ab_test_proc, motors_enabled
    # Ramp rates per tick at ~30 Hz
    _ACCEL      = 0.5     # m/s  per tick
    _DECEL      = 1.0     # m/s  per tick
    _OACCEL     = 0.8     # rad/s per tick
    _ODECEL     = 1.6     # rad/s per tick

    _target_vx = _target_vy = _target_omega = 0.0
    _ramp_vx   = _ramp_vy   = _ramp_omega   = 0.0

    _last_cmd_time = time.monotonic()
    _watchdog_fired = False

    while True:
        if _shutting_down:
            break
        if motion_queue is None:
            await asyncio.sleep(0.033)
            continue

        # 1. Drain latest command from queue
        cmd = None
        try:
            cmd = motion_queue.get_nowait()
        except asyncio.QueueEmpty:
            pass

        if cmd is not None:
            _target_vx, _target_vy, _target_omega, instant = cmd
            if instant:
                _ramp_vx = _target_vx
                _ramp_vy = _target_vy
                _ramp_omega = _target_omega
            _last_cmd_time = time.monotonic()
            _watchdog_fired = False

        # 2. Watchdog: zero targets if input has been silent too long
        if not is_auto_driving and not _watchdog_fired:
            if time.monotonic() - _last_cmd_time > MOTION_WATCHDOG_TIMEOUT:
                _target_vx = _target_vy = _target_omega = 0.0
                _watchdog_fired = True
                logger.debug("motion_loop: watchdog fired — ramping to stop")

        # 3. Step ramp toward target
        _ramp_vx    = _step_toward(_ramp_vx,    _target_vx,    _ACCEL,  _DECEL)
        _ramp_vy    = _step_toward(_ramp_vy,    _target_vy,    _ACCEL,  _DECEL)
        _ramp_omega = _step_toward(_ramp_omega, _target_omega, _OACCEL, _ODECEL)

        # 4. Continuous active heartbeat to hardware at 30 Hz (only if no test process is running)
        # Idea 111: reduced from 20 Hz (0.05 s) to 30 Hz (0.033 s) — reduces serial write CPU overhead
        # and prevents serial port lock contention without sacrificing control responsiveness.
        test_running = (_p2p_proc is not None and _p2p_proc.poll() is None) or \
                       (_ab_test_proc is not None and _ab_test_proc.poll() is None) or \
                       _is_standalone_test_running()
        if drive and not test_running and not _shutting_down:
            try:
                if motors_enabled:
                    drive.move(_ramp_vx, _ramp_vy, _ramp_omega)
                else:
                    if hasattr(drive, '_cmd_vel_pub'):
                        from geometry_msgs.msg import Twist
                        msg = Twist()
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.angular.z = 0.0
                        drive._cmd_vel_pub.publish(msg)
                    elif hasattr(drive, 'ros_board'):
                        drive.ros_board.set_motor(0, 0, 0, 0)
                    else:
                        drive.stop()
            except Exception as e:
                # During shutdown the rclpy context may already be torn down;
                # publishing then raises RCLError. Never let that propagate — an
                # unhandled error here aborts the process before cleanup() can kill
                # the ROS2 bringup stack, orphaning Mcnamu/base_node on the serial.
                if _shutting_down:
                    break
                logger.warning(f"motion_loop: drive.move failed: {e}")

        await asyncio.sleep(0.033)  # ~30 Hz (Idea 111)


async def oled_loop():
    """Refresh OLED with WiFi SSID and IP every 5 seconds."""
    while True:
        if _shutting_down:
            break
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


async def map_push_loop():
    """Push SLAM OccupancyGrid to all connected clients at ~1 Hz when SLAM is active.

    Sends a MAPU binary frame whenever the grid has been updated since the last push,
    giving real-time map updates regardless of foxglove_bridge QoS configuration.
    """
    _last_grid_id = None
    while True:
        if _shutting_down:
            break
        await asyncio.sleep(1.0)
        if not connected_clients or ros_bridge is None:
            continue
        if _slam_proc is None or _slam_proc.poll() is not None:
            continue
        grid = ros_bridge.get_occupancy_grid()
        if grid is None:
            continue
        # Use grid dimensions + data length as a cheap change fingerprint
        grid_id = (grid["width"], grid["height"], len(grid["data"]))
        if grid_id == _last_grid_id:
            continue
        _last_grid_id = grid_id
        frame_bytes = _encode_mapu(grid)
        if frame_bytes:
            websockets.broadcast(connected_clients, frame_bytes)


def _start_http_server():
    """Serve src/web/ over HTTP so Web Workers can load via http:// instead of file://."""
    web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')

    class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
        def end_headers(self):
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            super().end_headers()

        def log_message(self, fmt, *args):  # noqa: suppress per-request noise
            super().log_message(fmt, *args)

    handler = functools.partial(NoCacheHandler, directory=web_dir)
    httpd = http.server.HTTPServer(('0.0.0.0', HTTP_PORT), handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    logger.info(f"GUI available at http://0.0.0.0:{HTTP_PORT}/GUI.html")


def _move_locked(servo, servo_id, counts, profile_v):
    """servo.move under _servo_lock (the sampler is reading on the same bus)."""
    with _servo_lock:
        servo.move(servo_id, counts, profile_v)


def _tilt_target_moving(target, position, hardware_moving, elapsed, continuous, tolerance):
    """Continuous mode does not poll Moving; finish only near the encoder target."""
    if target is None:
        return False
    return elapsed < LIDAR_SWEEP_SETTLE_S or (
        abs(position - target) > tolerance if continuous else hardware_moving)


def _tilt_sampler(servo, servo_id, counts_to_deg, rate_hz=TILT_SAMPLE_HZ):
    """Poll the tilt servo at a fixed period and publish the angle.

    Runs in its own thread so its cadence does not depend on the asyncio loop
    getting round to it.  It is the ONLY reader of the servo; lidar_scan_loop
    consumes the snapshot and issues goal writes under the same _servo_lock.

    Publishes both /lidar_tilt/joint_states (the /scan safety gate's source)
    and /joint_states (robot_state_publisher, replacing the
    joint_state_publisher relay -- see ROS2Bridge.full_joint_pub).
    """
    global _tilt_gate_moving
    from sensor_msgs.msg import JointState
    from rclpy.duration import Duration

    period = 1.0 / max(1.0, rate_hz)
    next_t = time.monotonic()
    warned = False
    # Loop-period histogram.  The whole point of this thread is REGULARITY --
    # a high mean rate with a long tail is still useless to deskew, which
    # interpolates across whatever the worst gap happens to be.  Logging the
    # tail makes GIL stalls visible instead of leaving them as a mystery in
    # somebody's rate measurement later.
    stat_t = time.monotonic()
    periods = []
    last_pub = None
    read_ms_sum = 0.0
    read_n = 0
    # Wheel joints are declared so this message is a drop-in for what
    # joint_state_publisher used to emit; nothing drives them, exactly as
    # before.  lidar_tilt_joint MUST be last so the slices below line up.
    names = ['front_right_joint', 'front_left_joint',
             'back_right_joint', 'back_left_joint', 'lidar_tilt_joint']

    while not _shutting_down:
        next_t += period
        try:
            with _servo_lock:
                # Anchor the measurement to ROS time BEFORE the position read.
                # Stamping at publication includes read_moving, lock contention
                # and scheduling delays, which become false motion in deskew.
                ros_read_start = (ros_bridge._node.get_clock().now()
                                  if ros_bridge is not None else None)
                t0 = time.monotonic()
                pos = servo.read_pos(servo_id)
                t1 = time.monotonic()
                # Only pay for the second round trip when the state machine is
                # actually waiting on it.
                moving = servo.read_moving(servo_id) if _tilt_need_moving else False
            # Midpoint of the round trip, not the moment we finished parsing:
            # at 1.49 ms and 45 deg/s the difference is only 0.03 deg, but it
            # costs nothing and it is the defensible instant.
            t_meas = 0.5 * (t0 + t1)
            read_ms_sum += (t1 - t0)
            read_n += 1
            warned = False
        except Exception as e:
            with _tilt_lock:
                _tilt_sample["errors"] += 1
                errs = _tilt_sample["errors"]
            if not warned:
                warned = True
                logger.warning("tilt sampler: servo read failed (%d total): %s",
                               errs, e)
            # Do NOT publish on failure.  A silent tilt source makes
            # lidar_3d_processor_node LATCH the last known angle, which is the
            # safe behaviour; publishing a stale or zero angle would tell it
            # the mount is level when it may be pitched over.
            time.sleep(max(0.005, next_t - time.monotonic()))
            continue

        deg = counts_to_deg(pos)
        with _tilt_lock:
            _tilt_sample.update(counts=pos, deg=deg, moving=bool(moving),
                                t=t_meas)
            _tilt_sample["reads"] += 1

        if (ros_read_start is not None and ros_bridge is not None
                and hasattr(ros_bridge, 'joint_pub')):
            try:
                stamp = (ros_read_start + Duration(
                    nanoseconds=round((t1 - t0) * 0.5e9))).to_msg()
                tilt_rad = math.radians(deg)
                gate = 1.0 if _tilt_gate_moving else 0.0

                # The mount's own topic: tilt only, so nothing here can ever be
                # mistaken for a level reading of a joint we did not measure.
                m = JointState()
                m.header.stamp = stamp
                m.name = ['lidar_tilt_joint']
                m.position = [tilt_rad]
                m.velocity = [gate]
                ros_bridge.joint_pub.publish(m)

                # The merged topic robot_state_publisher listens to.
                f = JointState()
                f.header.stamp = stamp
                f.name = names
                f.position = [0.0, 0.0, 0.0, 0.0, tilt_rad]
                f.velocity = [0.0, 0.0, 0.0, 0.0, gate]
                ros_bridge.full_joint_pub.publish(f)
            except Exception as e:
                logger.debug("tilt sampler publish failed: %s", e)

        now_p = time.monotonic()
        if last_pub is not None:
            periods.append((now_p - last_pub) * 1e3)
        last_pub = now_p
        if now_p - stat_t >= 60.0 and periods:
            periods.sort()
            n = len(periods)
            logger.info(
                "tilt sampler: %.1f Hz over %.0f s (target %.0f) | period ms "
                "p50 %.1f p95 %.1f p99 %.1f max %.1f | read %.2f ms | "
                "%d errors",
                n / (now_p - stat_t), now_p - stat_t, rate_hz,
                periods[n // 2], periods[int(n * 0.95)], periods[int(n * 0.99)],
                periods[-1], read_ms_sum / max(1, read_n) * 1e3,
                _tilt_sample["errors"])
            stat_t, periods = now_p, []
            read_ms_sum = read_n = 0

        slack = next_t - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            # Fell behind (a long GIL hold, or the bus stalled).  Re-base rather
            # than trying to catch up, which would burst reads back to back.
            next_t = time.monotonic()

    logger.info("tilt sampler: stopped after %d reads, %d errors",
                _tilt_sample["reads"], _tilt_sample["errors"])


async def lidar_scan_loop():
    """Drive the Dynamixel XL430 tilt mount and publish lidar_tilt_joint.

    Step-and-stare, not a continuous nod.  A continuous sweep moves the mount
    while the lidar is mid-revolution: at 30 deg/s and 8 Hz the scan plane
    pitches 3.75 deg within one revolution, smearing every return by ~20 cm at
    3 m.  Holding still for each step instead means every ray in a dwell shares
    one exact transform, so no deskewing or continuous-time trajectory is
    needed -- and the mount's own encoder, not wheel odometry, supplies the
    geometry.

    Publishes /lidar_tilt/joint_states at 10 Hz with velocity used as a
    "still moving" flag (0 = holding, 1 = travelling) so lidar_3d_processor_node
    can discard the moving part of each step.
    """
    if 'XL430' not in globals():
        return

    try:
        servo = XL430('/dev/openrb150')
    except DynamixelError as e:
        logger.warning("lidar_scan_loop: No Dynamixel servo found on "
                       "/dev/openrb150: %s", e)
        return

    counts_per_deg = COUNTS_PER_DEG
    servo_id = 1
    horiz = 2048
    direction = None
    limits = (0, 4095)
    # The XL430 settles 7-11 counts short of any goal (gearbox backlash,
    # measured 2026-08-28 -- Position I Gain does nothing).  A 3-count level
    # check therefore fired on every single startup and re-homed to the same
    # off-by-10 spot.  Take the band from the calibration file.
    home_tol = 12
    try:
        calib_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'lidar_tilt_calibration_dynamixel.json')
        with open(calib_path) as f:
            calib = json.load(f)
        horiz = int(calib["horizontal_counts"])
        servo_id = int(calib.get("servo_id", 1))
        home_tol = max(3, int(calib.get("tolerance_counts", home_tol)))
        direction = calib.get("tilt_direction")
        lim = (calib.get("servo_state_at_calibration") or {}).get("position_limit_counts")
        if lim and len(lim) == 2:
            limits = (int(lim[0]), int(lim[1]))
    except Exception as e:
        logger.warning(f"lidar_scan_loop: Could not load calibration: {e}")

    try:
        if servo.ping(servo_id) is None:
            logger.warning("lidar_scan_loop: Dynamixel id %d did not answer on "
                           "/dev/openrb150", servo_id)
            servo.close()
            return
        # Position mode is an EEPROM setting and can only be changed with
        # torque disabled. Reassert it here so the GUI never inherits a mode
        # left behind by a commissioning/test utility.
        if servo.is_loaded(servo_id):
            servo.set_load(servo_id, False)
        servo.set_operating_mode(servo_id, OPERATING_MODE_POSITION)
        # Profile acceleration is persistent RAM state.  Without setting it,
        # sweep feel depends on whichever commissioning tool ran last; that
        # made nominally identical 30 deg/s sweeps alternately sluggish or
        # abrupt.  Reassert a moderate ramp on every server start.
        servo.set_profile_acceleration(servo_id, LIDAR_SWEEP_ACCELERATION)
        limits = tuple(int(v) for v in servo.read_position_limits(servo_id))
    except DynamixelError as e:
        logger.warning("lidar_scan_loop: Dynamixel initialization failed: %s", e)
        servo.close()
        return

    if direction not in (1, -1):
        # The URDF's lidar_tilt_joint is positive nose-DOWN.  Which way the
        # servo counts run has never been watched, so this is a guess: a wrong
        # one MIRRORS the 3D cloud about the horizontal plane.  It does not
        # affect the /scan safety gate, which only looks at |angle|.
        logger.warning("lidar_scan_loop: tilt_direction unverified in the "
                       "calibration file; assuming +1 (more counts = nose "
                       "down). A wrong sign mirrors the 3D map vertically.")
        direction = 1

    def deg_to_counts(deg):
        return int(round(horiz + deg * counts_per_deg * direction))

    def counts_to_deg(counts):
        return (counts - horiz) * direction / counts_per_deg

    # Sweep symmetrically about the CALIBRATED horizontal, then clamp to what
    # the servo can actually reach.  The previous version re-centred on
    # wherever the mount happened to be sitting, which with a level mount near
    # the bottom of its travel silently threw away the entire downward half.
    lo_counts = max(limits[0], deg_to_counts(-LIDAR_SWEEP_DEG))
    hi_counts = min(limits[1], deg_to_counts(LIDAR_SWEEP_DEG))
    if lo_counts > hi_counts:
        lo_counts, hi_counts = hi_counts, lo_counts
    lo_deg, hi_deg = sorted((counts_to_deg(lo_counts), counts_to_deg(hi_counts)))
    step_counts = max(1, int(round(LIDAR_SWEEP_STEP_DEG * counts_per_deg)))
    n_steps = max(1, int((hi_counts - lo_counts) / step_counts) + 1)
    logger.info(
        "lidar_scan_loop: sweep %.1f..%.1f deg (%d..%d counts, level=%d), "
        "%d steps of %.2f deg, ~%.0f s per sweep"
        % (lo_deg, hi_deg, lo_counts, hi_counts, horiz, n_steps,
           step_counts / counts_per_deg,
           n_steps * (LIDAR_SWEEP_SETTLE_S + LIDAR_SWEEP_DWELL_S)))
    if abs(hi_deg - lo_deg) < LIDAR_SWEEP_DEG:
        logger.warning(
            "lidar_scan_loop: the servo's angle limits %s clip the requested "
            "+/-%.0f deg sweep down to %.1f..%.1f deg. Vertical coverage is "
            "asymmetric; re-seat the mount or re-run --calibrate.",
            limits, LIDAR_SWEEP_DEG, lo_deg, hi_deg)

    # Home on startup. x3_dynamixel_tilt_home.service only runs at boot, and the loop
    # below only returns the mount to level if THIS process was the one
    # sweeping, so a restart while tilted leaves it parked off-level forever --
    # and a tilted mount gates /scan off indefinitely, blinding slam_toolbox
    # and AMCL.  Seen live 2026-08-23: restarted mid-sweep, mount sat at
    # 12.9 deg with /scan dark until homed by hand.
    try:
        pos0 = await asyncio.to_thread(servo.read_pos, servo_id)
        if pos0 is not None and abs(pos0 - horiz) > home_tol:
            logger.warning(
                "lidar_scan_loop: mount is at %d counts (%.1f deg off level) at "
                "startup -- homing to %d so /scan is not left gated off.",
                pos0, (pos0 - horiz) / counts_per_deg, horiz)
            await asyncio.to_thread(servo.set_load, servo_id, True)
            await asyncio.to_thread(servo.move, servo_id, horiz,
                                    deg_s_to_profile_velocity(LIDAR_SWEEP_SPEED_DEG_S))
            await asyncio.to_thread(servo.wait_until_settled, servo_id, 5.0)
            pos1 = await asyncio.to_thread(servo.read_pos, servo_id)
            logger.info("lidar_scan_loop: homed, mount now at %s counts", pos1)
    except Exception as e:
        logger.warning("lidar_scan_loop: startup homing failed: %s", e)

    global lidar_3d_scan_enabled
    global servo_stats
    global _tilt_need_moving, _tilt_gate_moving

    # Hand the read path to its own thread from here on.  Everything above ran
    # before the sampler existed, so it could touch the bus freely; below this
    # line every servo access must hold _servo_lock.
    threading.Thread(target=_tilt_sampler, name='tilt_sampler',
                     args=(servo, servo_id, counts_to_deg),
                     daemon=True).start()
    logger.info("lidar_scan_loop: tilt sampler started at %.0f Hz "
                "(publishing /lidar_tilt/joint_states AND /joint_states)",
                TILT_SAMPLE_HZ)

    scanning = False
    active_mode = lidar_sweep_mode
    target_counts = None
    settled_at = 0.0
    target_commanded_at = 0.0
    target_was_moving = False
    step_idx = 0
    sweep_dir = 1          # +1 climbing toward hi_counts, -1 coming back down
    last_vin_check = 0.0

    while True:
        if _shutting_down:
            break

        try:
            # The sampler thread owns the read path now; this loop consumes its
            # snapshot instead of racing it for the bus.
            with _tilt_lock:
                pos = _tilt_sample["counts"]
                hardware_moving = _tilt_sample["moving"]

            now = time.monotonic()
            if now - last_vin_check > 5.0:
                try:
                    def _read_vin():
                        with _servo_lock:
                            return servo.read_vin(servo_id)
                    vin = await asyncio.to_thread(_read_vin)
                    servo_stats["voltage"] = "%.2f V" % vin if vin else "--"
                except Exception:
                    servo_stats["voltage"] = "--"
                last_vin_check = now

            if pos is not None:
                # The XL430's Moving flag asserts a few milliseconds after a
                # goal write. Keep a short grace period so an immediate false
                # reading cannot make us skip straight to the next waypoint.
                moving = _tilt_target_moving(
                    target_counts, pos, hardware_moving,
                    now - target_commanded_at, active_mode == "continuous", home_tol)
                if hardware_moving:
                    target_was_moving = True
                if (target_counts is not None and not moving and
                        (target_was_moving or now - target_commanded_at >=
                         LIDAR_SWEEP_SETTLE_S)):
                    if settled_at < target_commanded_at:
                        settled_at = now
                servo_stats["state"] = "Busy" if moving else "Free"

                # Tell the sampler what to publish and what to spend a second
                # round trip on.  The Moving flag is only consulted by the
                # step-and-stare settle logic, so it is only worth reading
                # while a goal is outstanding in step mode.
                _tilt_need_moving = (target_counts is not None and
                                     active_mode != "continuous")
                # Continuous sweeps never advertise a horizontal parked scan,
                # including at reversals. The 3D deskew gate is independent.
                _tilt_gate_moving = moving or (scanning and active_mode == "continuous")

                # Abort arm of the interlock: a goal can arrive from RViz,
                # the CLI, or navigation_fsm without ever passing through the
                # websocket handler, so the running sweep has to watch for it
                # too and get the scan plane back to level.
                if scanning and _sweep_started_at and (
                        now - _sweep_started_at > LIDAR_SWEEP_MAX_S):
                    logger.warning(
                        "lidar_scan_loop: sweep hit the %.0f s cap — stopping. "
                        "A full pass is ~43 s, so this means the disable was "
                        "lost or the client vanished.", LIDAR_SWEEP_MAX_S)
                    lidar_3d_scan_enabled = False
                if scanning:
                    conflict = _tilt_nav_conflict()
                    if conflict:
                        logger.warning(
                            "lidar_scan_loop: ABORTING sweep -- %s. Returning "
                            "the mount to level so /scan resumes.", conflict)
                        lidar_3d_scan_enabled = False

                if lidar_3d_scan_enabled:
                    if not scanning:
                        # Enable torque and prove the XL430 accepted it before
                        # starting a sweep; reads still work when torque is off,
                        # while goal-position writes produce no movement.
                        try:
                            def _vin_temp():
                                with _servo_lock:
                                    return (servo.read_vin(servo_id),
                                            servo.read_temp(servo_id))
                            vin, temp = await asyncio.to_thread(_vin_temp)
                        except Exception:
                            vin, temp = None, None
                        def _enable_torque():
                            with _servo_lock:
                                servo.set_load(servo_id, True)
                        await asyncio.to_thread(_enable_torque)
                        await asyncio.sleep(0.05)
                        try:
                            def _is_loaded():
                                with _servo_lock:
                                    return servo.is_loaded(servo_id)
                            loaded = await asyncio.to_thread(_is_loaded)
                        except Exception:
                            loaded = None
                        if loaded is False:
                            logger.error(
                                "lidar_scan_loop: servo refuses torque "
                                "(vin=%s V, temp=%s C). Check the Dynamixel "
                                "power supply and hardware-error status. "
                                "Sweep aborted.",
                                "%.2f" % vin if vin else "?", temp)
                            lidar_3d_scan_enabled = False
                            continue
                        if vin is not None and vin < 9.0:
                            logger.warning(
                                "lidar_scan_loop: servo vin=%.2f V is below "
                                "the XL430's 9.0 V rated minimum -- it may "
                                "stall or fault mid-sweep.", vin)
                        logger.info("lidar_scan_loop: sweep starting "
                                    "(vin=%s V, temp=%s C, torque on)",
                                    "%.2f" % vin if vin else "?", temp)
                        scanning = True
                        step_idx = 0
                        sweep_dir = 1
                        target_counts = None
                        active_mode = lidar_sweep_mode
                        logger.info("lidar_scan_loop: style=%s at %.1f deg/s",
                                    active_mode, lidar_sweep_speed_deg_s)
                        # Fire-and-forget: an 8 s `ros2 param set` must not
                        # stall the 10 Hz joint publisher the /scan gate
                        # depends on.
                        asyncio.create_task(_apply_sweep_gate())
                    if active_mode != lidar_sweep_mode:
                        # Switched mid-sweep from the GUI.  Re-plan from the
                        # next waypoint rather than finishing the old style's
                        # pass, so an A/B comparison starts immediately.
                        active_mode = lidar_sweep_mode
                        target_counts = None
                        step_idx = 0
                        sweep_dir = 1
                        logger.info("lidar_scan_loop: sweep style switched to "
                                    "%s at %.1f deg/s mid-pass",
                                    active_mode, lidar_sweep_speed_deg_s)
                    profile_v = deg_s_to_profile_velocity(lidar_sweep_speed_deg_s)
                    if active_mode == "continuous":
                        # No dwell and no stations: run limit to limit and
                        # turn around on arrival.  Every scan captured in
                        # between straddles speed*0.156 deg of pitch, and the
                        # processor is told (or made) to keep it anyway.
                        if target_counts is None or (not moving and settled_at):
                            target_counts = hi_counts if sweep_dir > 0 else lo_counts
                            await asyncio.to_thread(
                                _move_locked, servo, servo_id, target_counts,
                                profile_v)
                            target_commanded_at = now
                            target_was_moving = False
                            settled_at = 0.0
                            sweep_dir = -sweep_dir
                    elif target_counts is None or (not moving and now >= settled_at + LIDAR_SWEEP_DWELL_S):
                        target_counts = min(hi_counts,
                                            lo_counts + step_idx * step_counts)
                        await asyncio.to_thread(
                            _move_locked, servo, servo_id, target_counts,
                            profile_v)
                        target_commanded_at = now
                        target_was_moving = False
                        settled_at = 0.0
                        # Ping-pong between the limits.  Resetting step_idx to
                        # zero here creates one uninterrupted 90 deg flyback,
                        # defeating the station timing and looking like a
                        # backwards jerk at the end of every pass.
                        step_idx += sweep_dir
                        if step_idx >= n_steps:
                            sweep_dir = -1
                            step_idx = max(0, n_steps - 2)
                        elif step_idx < 0:
                            sweep_dir = 1
                            step_idx = min(n_steps - 1, 1)
                elif scanning:
                    scanning = False
                    target_counts = horiz
                    await asyncio.to_thread(
                        _move_locked, servo, servo_id, horiz,
                        deg_s_to_profile_velocity(LIDAR_SWEEP_SPEED_DEG_S))
                    target_commanded_at = time.monotonic()
                    target_was_moving = False
                    settled_at = 0.0
                    logger.info("lidar_scan_loop: sweep stopped, returning to level")
        except Exception as e:
            logger.debug(f"lidar_scan_loop error: {e}")

        await asyncio.sleep(0.1)

async def main():
    global motion_queue
    _start_http_server()
    initialize_hardware()
    motion_queue = asyncio.Queue(maxsize=2)

    threading.Thread(target=_system_sampler, name='system_sampler',
                     daemon=True).start()

    # Graceful shutdown: on SIGINT/SIGTERM (e.g. `systemctl restart`) set the flag so
    # the async loops exit, gather() returns, and the finally-block cleanup() runs —
    # sending a motor stop and cleanly killing the ROS2 bringup. Without this the loops
    # spin forever and systemd escalates to SIGKILL (no graceful cleanup).
    def _on_shutdown_signal():
        global _shutting_down
        if not _shutting_down:
            logger.info("Shutdown signal received — stopping loops for graceful cleanup")
        _shutting_down = True
    _loop = asyncio.get_running_loop()
    for _s in (signal.SIGINT, signal.SIGTERM):
        try:
            _loop.add_signal_handler(_s, _on_shutdown_signal)
        except (NotImplementedError, RuntimeError):
            pass  # platform without signal-handler support

    # Disable websockets deflate compression to avoid blocking main thread zlib operations (Idea 117)
    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT, compression=None):
        logger.info(f"Server started on ws://0.0.0.0:{WS_PORT}")
        await asyncio.gather(broadcast_loop(), motion_loop(), oled_loop(), map_push_loop(), lidar_scan_loop())

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
