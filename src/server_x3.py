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
args = parser.parse_args()
SIM_MODE = args.sim

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

connected_clients = set()

# =============================================================================
# INITIALIZATION
# =============================================================================

def initialize_hardware():
    global ros_board, drive, lidar, camera, robot_state, model, oled

    logger.info("="*50)
    logger.info("Initializing Yahboom X3 Hardware")
    logger.info("="*50)

    # 1. Motor Controller (Serial) - uses auto-detected SERIAL_PORT
    logger.info(f"Connecting to Rosmaster on {SERIAL_PORT}...")
    ros_board = Rosmaster(sim_mode=SIM_MODE)

    # 2. Mecanum Drive Wrapper
    drive = MecanumDrive(ros_board)
    logger.info("Mecanum Drive initialized")

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

    # 5. YDLidar
    logger.info(f"Initializing Lidar on {LIDAR_PORT}...")
    lidar = YDLidarDriver(port=LIDAR_PORT, sim_mode=SIM_MODE)

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
    if ros_board: ros_board.cleanup()
    if camera: camera.cleanup()
    if lidar: lidar.cleanup()
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
        logger.info("Client disconnected")

# =============================================================================
# MAIN BROADCAST LOOP
# =============================================================================

async def broadcast_loop():
    global _cam_frame_count, _yolo_frame_count, _fps_last_time
    global fps_camera, fps_detection, last_detections, depth_enabled, lidar_enabled

    loop = asyncio.get_event_loop()
    _depth_cycle = 0  # throttle depth to ~10 fps (every other 20fps cycle)

    while True:
        if connected_clients:
            now = time.time()

            # 1. Camera frame — run blocking capture in thread pool
            frame = await loop.run_in_executor(None, camera.get_frame) if camera else None
            if frame is not None:
                _cam_frame_count += 1

            # 2. YOLO detection — also blocking, run in executor
            if detection_enabled and frame is not None and model:
                def _run_yolo():
                    results = model(frame, verbose=False, conf=CONFIDENCE_THRESHOLD, imgsz=INFERENCE_SIZE)
                    dets = []
                    for r in results:
                        for box in r.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            label = model.names[int(box.cls[0])] if model.names else str(int(box.cls[0]))
                            conf = float(box.conf[0])
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            dets.append({"label": label, "bbox": [x1, y1, x2, y2], "conf": conf})
                    return dets
                last_detections = await loop.run_in_executor(None, _run_yolo)
                _yolo_frame_count += 1

            # 3. FPS update every second
            elapsed = now - _fps_last_time
            if elapsed >= 1.0:
                fps_camera = round(_cam_frame_count / elapsed, 1)
                fps_detection = round(_yolo_frame_count / elapsed, 1)
                _cam_frame_count = 0
                _yolo_frame_count = 0
                _fps_last_time = now

            # 4. Encode RGB image
            img_str = ""
            if frame is not None:
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                img_str = base64.b64encode(buffer).decode('utf-8')

            # 4b. Depth frame — throttled to 10fps, run in executor
            depth_str = ""
            _depth_cycle += 1
            if depth_enabled and camera and (_depth_cycle % 2 == 0):
                depth_frame = await loop.run_in_executor(None, camera.get_depth_frame)
                if depth_frame is not None:
                    _, dbuf = cv2.imencode('.jpg', depth_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    depth_str = base64.b64encode(dbuf).decode('utf-8')

            # 5. Lidar points (only when toggle is on)
            scan_points = lidar.get_points_xy() if (lidar and lidar_enabled) else []

            # 6. Robot pose (x/y/theta — stays at 0,0,0 until encoder/IMU integration)
            pose = {
                "x": robot_state.x if robot_state else 0.0,
                "y": robot_state.y if robot_state else 0.0,
                "theta": robot_state.theta if robot_state else 0.0
            }

            if ros_board:
                m1_enc, m2_enc, m3_enc, m4_enc = ros_board.get_motor_encoder()
                batt_v = ros_board.get_battery_voltage()
            else:
                m1_enc = m2_enc = m3_enc = m4_enc = 0
                batt_v = 12.0

            batt_pct = max(0.0, min(100.0, (batt_v - 8.1) / (12.6 - 8.1) * 100.0))
            
            avg_pwr = (abs(current_left_power) + abs(current_right_power)) / 2.0
            est_current = 0.5 + (avg_pwr * 6.0)
            est_watts = batt_v * est_current

            # 7. Build readout (matches GUI handleMessage "readout" handler)
            msg = {
                "type": "readout",
                "image": img_str,
                "depth_image": depth_str,
                "lidar_points": scan_points,
                "robot_pose": pose,
                "target_pose": {"x": None, "y": None, "distance_cm": None},
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
                "is_demo_mode": False,
                "nav_phase": "AUTO" if is_auto_driving else "IDLE",
                "active_model_name": active_model_name,
                "fps_camera": fps_camera,
                "fps_detection": fps_detection,
                "detections": last_detections,
                "battery": {"voltage": batt_v, "amps": est_current, "watts": est_watts},
                "power": {
                    "voltage": batt_v,
                    "current": est_current,
                    "power": est_watts,
                    "battery_pct": batt_pct
                },
                "latest_log": None
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
