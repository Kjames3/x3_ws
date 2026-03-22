
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
# We will replace NativeLidar with YDLidarDriver in next iteration
from drivers import NativeCamera, Picamera2Driver

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
