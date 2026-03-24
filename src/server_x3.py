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
import socket
import subprocess
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

# Add root directory to sys.path to allow importing 'robot_state'
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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
from robot_state import RobotState

# =============================================================================
# CONFIGURATION
# =============================================================================
parser = argparse.ArgumentParser(description='Yahboom X3 Control Server')
parser.add_argument('--sim', action='store_true', help='Run in simulation mode')
parser.add_argument('--ros2', action='store_true',
                    help='ROS2 bridge mode: skip serial hardware (lidar+motors), '
                         'read /scan and publish /cmd_vel via rclpy instead')
args = parser.parse_args()
SIM_MODE   = args.sim
ROS2_MODE  = args.ros2

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
drive = None
lidar = None
camera = None
robot_state = None
model = None
oled = None
_gazebo_proc = None   # subprocess handle when --sim auto-launches Gazebo

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
    LINEAR_SCALE  = 0.5   # Rosmaster 1.0 → 0.5 m/s
    ANGULAR_SCALE = 2.0   # Rosmaster 5.0 → 2.0 rad/s  (tune if needed)

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import Twist

        rclpy.init(args=None)
        self._node = Node('x3_ws_bridge')
        self._lock = threading.Lock()
        self._points: list[float] = []

        self._node.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._cmd_vel_pub = self._node.create_publisher(Twist, '/cmd_vel', 10)

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()
        logger.info("ROS2Bridge: spinning — subscribed /scan, publishing /cmd_vel")

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
    cmd = (
        f'source /opt/ros/humble/setup.bash && '
        f'source {install_setup} && '
        f'ros2 launch yahboomcar_nav x3_gazebo.launch.py'
    )
    proc = subprocess.Popen(
        ['bash', '-c', cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,   # process group — lets us kill all children
    )
    logger.info(f"Gazebo launched (pid {proc.pid}) — waiting for topics...")
    return proc


def initialize_hardware():
    global ros_board, drive, lidar, camera, robot_state, model, oled, _gazebo_proc

    logger.info("="*50)
    logger.info("Initializing Yahboom X3 Hardware")
    logger.info("="*50)

    if SIM_MODE:
        # Gazebo simulation: auto-launch Gazebo then bridge to its topics.
        _gazebo_proc = _launch_gazebo()
        bridge = ROS2Bridge()
        drive = bridge
        lidar = bridge
    elif ROS2_MODE:
        # ROS2 bridge mode: Mcnamu_driver_X3 owns the serial port.
        # Skip Rosmaster / MecanumDrive / YDLidarDriver — use ROS2Bridge instead.
        logger.info("ROS2 mode: skipping serial hardware, using ROS2Bridge")
        bridge = ROS2Bridge()
        drive = bridge
        lidar = bridge
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

    # 3. Camera (Orbbec Astra Pro RGB + optional depth)
    logger.info("Initializing Camera...")
    camera = AstraCamera(width=640, height=480, sim_mode=SIM_MODE, enable_depth=False)

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

    # 6. Robot State (pose tracking — x/y/theta, updated by IMU when available)
    robot_state = RobotState()

    # 7. OLED Display (SSD1306 on I2C bus 1 — Jetson Orin pins 3/5)
    logger.info("Initializing OLED display...")
    oled = OLEDDisplay(i2c_port=7, i2c_address=0x3C, sim_mode=SIM_MODE)
    oled.show(["X3 Robot", "Starting...", ""])

    logger.info("="*50)
    logger.info("Initialization Complete")
    logger.info("="*50)

def cleanup():
    logger.info("Cleaning up...")
    if (SIM_MODE or ROS2_MODE) and drive is not None:
        drive.cleanup()  # ROS2Bridge.cleanup() shuts down rclpy
    if _gazebo_proc is not None:
        logger.info("Shutting down Gazebo...")
        try:
            os.killpg(os.getpgid(_gazebo_proc.pid), signal.SIGTERM)
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

def _apply_tank_as_mecanum():
    """Translate left/right slider powers into holonomic vx + omega."""
    vx = (current_left_power + current_right_power) / 2.0
    omega = (current_right_power - current_left_power) / 2.0
    if drive:
        drive.move(vx, 0.0, omega)

# =============================================================================
# WEBSOCKET HANDLER
# =============================================================================

async def handle_client(websocket):
    global detection_enabled, depth_enabled, lidar_enabled, is_auto_driving
    global current_left_power, current_right_power
    global model, active_model_name

    logger.info("Client connected")
    connected_clients.add(websocket)
    if camera:
        camera._has_clients = True  # P7: allow capture loop to store frames
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
                    _apply_tank_as_mecanum()

                elif msg_type == "set_move":
                    # Holonomic move: vx, vy, omega (direct)
                    vx = float(data.get("vx", 0.0))
                    vy = float(data.get("vy", 0.0))
                    omega = float(data.get("omega", 0.0))
                    if drive:
                        drive.move(vx, vy, omega)

                elif msg_type == "move":
                    # D-pad direction buttons
                    direction = data.get("direction")
                    if drive:
                        if direction == "forward":
                            drive.move(0.5, 0.0, 0.0)
                        elif direction == "backward":
                            drive.move(-0.5, 0.0, 0.0)
                        elif direction == "left":
                            drive.move(0.0, 0.0, 0.5)   # Rotate CCW
                        elif direction == "right":
                            drive.move(0.0, 0.0, -0.5)  # Rotate CW
                        elif direction == "strafe_left":
                            drive.move(0.0, -0.5, 0.0)
                        elif direction == "strafe_right":
                            drive.move(0.0, 0.5, 0.0)
                        elif direction == "stop":
                            drive.move(0.0, 0.0, 0.0)

                elif msg_type == "stop":
                    current_left_power = 0.0
                    current_right_power = 0.0
                    if drive:
                        drive.move(0.0, 0.0, 0.0)

                elif msg_type == "toggle_detection":
                    detection_enabled = data.get("enabled", False)
                    logger.info(f"Detection: {detection_enabled}")

                elif msg_type == "toggle_lidar":
                    lidar_enabled = data.get("enabled", False)
                    if lidar and not lidar.sim_mode:
                        if lidar_enabled:
                            lidar.start()
                        else:
                            lidar.stop()
                    logger.info(f"Lidar: {'enabled' if lidar_enabled else 'disabled'}")

                elif msg_type == "toggle_depth":
                    depth_enabled = data.get("enabled", False)
                    if camera:
                        if depth_enabled and camera._depth_stream is None:
                            camera._open_depth()
                        elif not depth_enabled and camera._depth_stream is not None:
                            camera._close_depth()
                    logger.info(f"Depth streaming: {depth_enabled}")

                elif msg_type == "start_auto_drive":
                    is_auto_driving = True
                    logger.info("Auto-drive started (stub)")

                elif msg_type == "stop_auto_drive":
                    is_auto_driving = False
                    current_left_power = 0.0
                    current_right_power = 0.0
                    if drive:
                        drive.move(0.0, 0.0, 0.0)

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

            # 6. Robot pose
            pose = {
                "x":     robot_state.x     if robot_state else 0.0,
                "y":     robot_state.y     if robot_state else 0.0,
                "theta": robot_state.theta if robot_state else 0.0,
            }

            # 7. Encoders + battery (P9: battery voltage cached at 1 Hz, not 20 Hz)
            if ros_board:
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
                "nav_phase": "AUTO" if is_auto_driving else "IDLE",
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

            websockets.broadcast(connected_clients, json.dumps(msg))

        await asyncio.sleep(0.05)  # 20 FPS cap

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
    initialize_hardware()
    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT):
        logger.info(f"Server started on ws://0.0.0.0:{WS_PORT}")
        await asyncio.gather(broadcast_loop(), oled_loop())

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
