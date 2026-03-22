
import logging
import threading
import sys
import os

# Add root directory to sys.path to allow importing 'drivers'
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import struct
import numpy as np
import cv2

# Configure logger for this module
logger = logging.getLogger(__name__)

# =============================================================================
# HARDWARE CONFIGURATION CONSTANTS
# =============================================================================

import os

# Serial Config
if os.path.exists("/dev/ttyCH341USB0"):
    SERIAL_PORT = "/dev/ttyCH341USB0"
else:
    SERIAL_PORT = "/dev/ttyUSB0"  # Fallback for standard kernel driver

SERIAL_BAUDRATE = 115200

# Robot Mechanicals (Mecanum)
WHEEL_SEPARATION_WIDTH = 0.17  # meters (half width?) Need verification
WHEEL_SEPARATION_LENGTH = 0.13 # meters
WHEEL_DIAMETER = 0.065 # meters

# =============================================================================
# ROSMASTER SERIAL DRIVER
# =============================================================================

from Rosmaster_Lib import Rosmaster as YahboomRosmaster

class Rosmaster:
    """
    Wrapper for Yahboom ROSMASTER X3 Controller Board using official driver.
    """
    def __init__(self, port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE, sim_mode=False):
        self.port = port
        self.baudrate = baudrate
        self.sim_mode = sim_mode
        self._bot = None
        
        if not self.sim_mode:
            self._connect()

    def _connect(self):
        try:
            self._bot = YahboomRosmaster(car_type=1, com=self.port)
            logger.info(f"Connected to ROSMASTER on {self.port}")
        except Exception as e:
            logger.error(f"Failed to connect to ROSMASTER: {e}")
            self._bot = None

    def set_motor(self, m1, m2, m3, m4):
        """
        Set speed for 4 motors.
        Range: -1.0 to 1.0 mapped to -100 to 100
        
        M1: Front Left
        M2: Front Right
        M3: Rear Left
        M4: Rear Right
        """
        if self.sim_mode or not self._bot:
            return

        try:
            s1 = int(m1 * 100)
            s2 = int(m2 * 100)
            s3 = int(m3 * 100)
            s4 = int(m4 * 100)
            
            self._bot.set_motor(s1, s2, s3, s4)
        except Exception as e:
            logger.error(f"Serial write error: {e}")

    def set_car_motion(self, vx, vy, vz):
        if self.sim_mode or not self._bot:
            return
        try:
            self._bot.set_car_motion(vx, vy, vz)
        except Exception as e:
            logger.error(f"Serial write error: {e}")

    def stop(self):
        self.set_motor(0, 0, 0, 0)
    
    def cleanup(self):
        self.stop()
        if self._bot:
            del self._bot
            self._bot = None


# =============================================================================
# MECANUM KINEMATICS
# =============================================================================

class MecanumDrive:
    def __init__(self, rosmaster_driver):
        self.driver = rosmaster_driver
    
    def move(self, vx, vy, omega):
        """
        Holonomic Movement.
        vx: Forward velocity (-1.0 to 1.0)
        vy: Sideways velocity (Right +, Left -)
        omega: Rotation (CCW +, CW -)
        """
        # The Yahboom X3 board natively performs Mecanum inverse kinematics
        # Note: Polarity and axis mappings might require tweaking based on
        # actual robot frame orientation (e.g. if vy is inverted).
        # We will map (vx, vy, omega) directly to set_car_motion(vx, vy, vz)
        self.driver.set_car_motion(vx, vy, omega)


# =============================================================================
# SENSOR CLASSES (Adapters)
# =============================================================================

# Reuse existing Camera and Lidar classes for now
from drivers import NativeCamera, Picamera2Driver


# =============================================================================
# ORBBEC ASTRA PRO CAMERA
# =============================================================================

class AstraCamera:
    """
    Driver for Orbbec Astra Pro SC camera.

    RGB stream: accessed via OpenCV using the /dev/camera_depth symlink
    created by the Yahboom udev rule (99-yahboom-camera.rules).

    Depth stream: accessed via OpenNI2 SDK (pip install openni).
    Falls back gracefully if OpenNI2 is not installed.
    """

    RGB_DEVICE = "/dev/camera_depth"

    def __init__(self, width=640, height=480, sim_mode=False, enable_depth=False):
        self.width = width
        self.height = height
        self.sim_mode = sim_mode
        self.enable_depth = enable_depth

        self._cap = None          # OpenCV VideoCapture for RGB
        self._oni_device = None   # OpenNI2 device
        self._depth_stream = None # OpenNI2 depth stream
        self._lock = threading.Lock()

        if not sim_mode:
            self._open_rgb()
            if enable_depth:
                self._open_depth()

    def _open_rgb(self):
        cap = cv2.VideoCapture(self.RGB_DEVICE)
        if not cap.isOpened():
            logger.error(f"AstraCamera: failed to open RGB at {self.RGB_DEVICE}. "
                         "Is the udev rule installed? Run: sudo udevadm trigger")
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap = cap
        logger.info(f"AstraCamera: RGB opened at {self.RGB_DEVICE} ({self.width}x{self.height})")

    def _open_depth(self):
        try:
            from openni import openni2
            openni2.initialize()
            self._oni_device = openni2.Device.open_any()
            self._depth_stream = self._oni_device.create_depth_stream()
            self._depth_stream.start()
            logger.info("AstraCamera: depth stream started via OpenNI2")
        except ImportError:
            logger.warning("AstraCamera: openni not installed — depth unavailable (pip install openni)")
        except Exception as e:
            logger.error(f"AstraCamera: depth init failed: {e}")

    def get_frame(self):
        """Return the latest RGB frame as a BGR numpy array, or None."""
        if self.sim_mode or self._cap is None:
            return None
        with self._lock:
            ret, frame = self._cap.read()
        if not ret:
            return None
        return frame

    def get_depth_frame(self):
        """
        Return a colourised depth image (BGR uint8) or None.
        White = near, dark = far.
        """
        if self._depth_stream is None:
            return None
        try:
            from openni import openni2
            frame = self._depth_stream.read_frame()
            buf = frame.get_buffer_as_uint16()
            depth = np.frombuffer(buf, dtype=np.uint16).reshape(
                frame.height, frame.width)
            # Normalise to 0-255 and colourise
            depth_8 = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            coloured = cv2.applyColorMap(depth_8, cv2.COLORMAP_JET)
            return coloured
        except Exception as e:
            logger.error(f"AstraCamera: depth read error: {e}")
            return None

    def cleanup(self):
        if self._cap:
            self._cap.release()
            self._cap = None
        if self._depth_stream:
            self._depth_stream.stop()
        try:
            from openni import openni2
            openni2.unload()
        except Exception:
            pass
        logger.info("AstraCamera: released")


class YDLidarDriver:
    """
    Driver for YDLidar 4ROS (via ydlidar pip package).
    """
    def __init__(self, port="/dev/ttyUSB0", sim_mode=False):
        self.port = port
        self.sim_mode = sim_mode
        self._scan = []
        self._lock = threading.Lock()
        
        if not sim_mode:
            try:
                import ydlidar
                # Setup code here
                pass
            except ImportError:
                logger.warning("ydlidar not installed")

    def get_points_xy(self):
        return []
        
    def cleanup(self):
        pass
