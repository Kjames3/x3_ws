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
    Rosmaster, MecanumDrive, YDLidarDriver, AstraCamera, SERIAL_PORT
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
YOLO_MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'models', 'yolo11n_cans.pt')
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

detection_enabled = False
is_auto_driving = False
last_detections = []

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
    global ros_board, drive, lidar, camera, robot_state, model

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

    # 4. YOLO Model
    try:
        logger.info(f"Loading YOLO: {YOLO_MODEL}")
        model = YOLO(YOLO_MODEL)
    except Exception as e:
        logger.error(f"YOLO Load Failed: {e}")

    # 5. YDLidar
    logger.info(f"Initializing Lidar on {LIDAR_PORT}...")
    lidar = YDLidarDriver(port=LIDAR_PORT, sim_mode=SIM_MODE)

    # 6. Robot State (pose tracking — x/y/theta, updated by IMU when available)
    robot_state = RobotState()

    logger.info("="*50)
    logger.info("Initialization Complete")
    logger.info("="*50)

def cleanup():
    logger.info("Cleaning up...")
    if ros_board: ros_board.cleanup()
    if camera: camera.cleanup()
    if lidar: lidar.cleanup()

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
    global detection_enabled, is_auto_driving
    global current_left_power, current_right_power

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

                elif msg_type == "start_auto_drive":
                    is_auto_driving = True
                    logger.info("Auto-drive started (stub)")

                elif msg_type == "stop_auto_drive":
                    is_auto_driving = False
                    current_left_power = 0.0
                    current_right_power = 0.0
                    if drive:
                        drive.move(0.0, 0.0, 0.0)

                # Silently ignore GUI-only messages (model switching, capture, demo, etc.)
                elif msg_type in ("set_model", "set_classes", "set_labels",
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
    global fps_camera, fps_detection, last_detections

    while True:
        if connected_clients:
            now = time.time()

            # 1. Camera frame
            frame = camera.get_frame() if camera else None
            if frame is not None:
                _cam_frame_count += 1

            # 2. YOLO detection
            if detection_enabled and frame is not None and model:
                results = model(frame, verbose=False, conf=CONFIDENCE_THRESHOLD, imgsz=INFERENCE_SIZE)
                last_detections = []
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        label = model.names[int(box.cls[0])] if model.names else str(int(box.cls[0]))
                        conf = float(box.conf[0])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        last_detections.append({
                            "label": label,
                            "bbox": [x1, y1, x2, y2],
                            "conf": conf
                        })
                _yolo_frame_count += 1

            # 3. FPS update every second
            elapsed = now - _fps_last_time
            if elapsed >= 1.0:
                fps_camera = round(_cam_frame_count / elapsed, 1)
                fps_detection = round(_yolo_frame_count / elapsed, 1)
                _cam_frame_count = 0
                _yolo_frame_count = 0
                _fps_last_time = now

            # 4. Encode image
            img_str = ""
            if frame is not None:
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                img_str = base64.b64encode(buffer).decode('utf-8')

            # 5. Lidar points
            scan_points = lidar.get_points_xy() if lidar else []

            # 6. Robot pose (x/y/theta — stays at 0,0,0 until encoder/IMU integration)
            pose = {
                "x": robot_state.x if robot_state else 0.0,
                "y": robot_state.y if robot_state else 0.0,
                "theta": robot_state.theta if robot_state else 0.0
            }

            # 7. Build readout (matches GUI handleMessage "readout" handler)
            msg = {
                "type": "readout",
                "image": img_str,
                "lidar_points": scan_points,
                "robot_pose": pose,
                "target_pose": {"x": None, "y": None, "distance_cm": None},
                "left_power": current_left_power,
                "right_power": current_right_power,
                "detection_enabled": detection_enabled,
                "is_auto_driving": is_auto_driving,
                "is_demo_mode": False,
                "nav_phase": "AUTO" if is_auto_driving else "IDLE",
                "fps_camera": fps_camera,
                "fps_detection": fps_detection,
                "detections": last_detections,
                "battery": None,
                "latest_log": None
            }

            websockets.broadcast(connected_clients, json.dumps(msg))

        await asyncio.sleep(0.05)  # 20 FPS cap

async def main():
    initialize_hardware()
    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT):
        logger.info(f"Server started on ws://0.0.0.0:{WS_PORT}")
        await broadcast_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
