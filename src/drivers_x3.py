
import logging
import threading
import os
import math
import time
import struct
import numpy as np
import cv2

# Configure logger for this module
logger = logging.getLogger(__name__)

# =============================================================================
# HARDWARE CONFIGURATION CONSTANTS
# =============================================================================

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
            self._bot.create_receive_threading()
            self._bot.set_auto_report_state(True)
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

    def get_battery_voltage(self):
        if self.sim_mode or not self._bot:
            return 12.0
        return self._bot.get_battery_voltage()

    def get_motor_encoder(self):
        if self.sim_mode or not self._bot:
            return 0, 0, 0, 0
        return self._bot.get_motor_encoder()

    def get_imu_data(self):
        """Return (gx, gy, gz, ax, ay, az) in rad/s and m/s²."""
        if self.sim_mode or not self._bot:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        gx, gy, gz = self._bot.get_gyroscope_data()
        ax, ay, az = self._bot.get_accelerometer_data()
        return gx, gy, gz, ax, ay, az

    def get_imu_attitude(self):
        """Return (roll, pitch, yaw) in radians from the board's onboard fusion filter."""
        if self.sim_mode or not self._bot:
            return 0.0, 0.0, 0.0
        return self._bot.get_imu_attitude_data(ToAngle=False)

    def get_magnetometer_data(self):
        """Return (mx, my, mz). Valid on both MPU9250 and ICM20948 (both 9-axis)."""
        if self.sim_mode or not self._bot:
            return 0.0, 0.0, 0.0
        return self._bot.get_magnetometer_data()

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

    # Orbbec Astra Pro SC USB IDs
    ORBBEC_RGB_VENDOR  = "2bc5"
    ORBBEC_RGB_PRODUCT = "0501"

    def __init__(self, width=640, height=480, sim_mode=False, enable_depth=False):
        self.width = width
        self.height = height
        self.sim_mode = sim_mode
        self.enable_depth = enable_depth

        self._cap = None          # OpenCV VideoCapture for RGB
        self._oni_device = None   # OpenNI2 device
        self._depth_stream = None # OpenNI2 depth stream
        self._lock = threading.Lock()

        # Pre-allocated depth processing buffers (set in _open_depth) — P8
        self._depth_buf_8   = None   # uint8 normalised
        self._depth_buf_col = None   # BGR colourised

        # Background capture thread — keeps camera buffer drained so get_frame() is instant
        self._running = True
        self._latest_frame = None
        self._capture_thread = None
        self._has_clients = False  # set by server; skips lock+copy when nobody is watching (P7)

        if not sim_mode:
            self._open_rgb()
            if enable_depth:
                self._open_depth()

    def _find_rgb_device(self):
        """
        Return the correct /dev/videoX path for the Orbbec RGB camera.
        Priority:
          1. /dev/camera_depth  (udev symlink, most reliable)
          2. Scan /dev/video0..7 and match by USB vendor/product via sysfs
          3. Fall back to /dev/video0
        """
        import glob

        # 1. udev symlink
        if os.path.exists("/dev/camera_depth"):
            return "/dev/camera_depth"

        # 2. sysfs scan — find which videoX belongs to the Orbbec RGB
        for video_path in sorted(glob.glob("/dev/video?")):
            dev_name = os.path.basename(video_path)   # e.g. "video0"
            # Walk sysfs looking for the matching idVendor/idProduct
            for sys_path in glob.glob(f"/sys/class/video4linux/{dev_name}/device/../../../*"):
                vendor_file = os.path.join(sys_path, "idVendor")
                product_file = os.path.join(sys_path, "idProduct")
                try:
                    with open(vendor_file) as f:
                        vendor = f.read().strip()
                    with open(product_file) as f:
                        product = f.read().strip()
                    if vendor == self.ORBBEC_RGB_VENDOR and product == self.ORBBEC_RGB_PRODUCT:
                        return video_path
                except Exception:
                    continue

        # 3. last resort
        logger.warning("AstraCamera: could not identify Orbbec RGB device via sysfs, trying /dev/video0")
        return "/dev/video0"

    def _open_rgb(self):
        device = self._find_rgb_device()
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            logger.error(f"AstraCamera: failed to open RGB at {device}.")
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap = cap
        logger.info(f"AstraCamera: RGB opened at {device} ({self.width}x{self.height})")
        # Start background thread that continuously drains the camera buffer
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def _capture_loop(self):
        """Continuously drain the camera buffer so get_frame() always returns fresh data.

        P7: when no clients are watching, we still drain the buffer (so frames don't
        pile up on reconnect) but skip the lock + copy to save CPU.
        """
        while self._running and self._cap is not None:
            ret, frame = self._cap.read()
            if ret and self._has_clients:
                with self._lock:
                    self._latest_frame = frame

    # Search order for libOpenNI2.so
    OPENNI2_SEARCH_PATHS = [
        os.path.join(os.path.dirname(os.path.abspath(__file__))),  # alongside this file
        "/usr/local/lib",
        "/usr/lib",
    ]

    def _open_depth(self):
        try:
            from openni import openni2
            # Attempt 1: no-arg init — uses LD_LIBRARY_PATH / OPENNI2_REDIST / system install.
            # This is the right path when the Yahboom/Orbbec SDK is installed system-wide.
            initialized = False
            try:
                openni2.initialize()
                initialized = True
                logger.debug("AstraCamera: OpenNI2 initialized via system default")
            except Exception:
                pass
            # Attempt 2: explicit search (pip package dir first, then system paths).
            if not initialized:
                import openni as _openni_mod
                pkg_dir = os.path.dirname(os.path.abspath(_openni_mod.__file__))
                search = [pkg_dir] + self.OPENNI2_SEARCH_PATHS
                lib_path = next(
                    (p for p in search
                     if os.path.exists(os.path.join(p, "libOpenNI2.so"))),
                    None,
                )
                if lib_path is None:
                    logger.error("AstraCamera: libOpenNI2.so not found. "
                                 "Copy ARM64 libs from the Yahboom SDK to ~/x3_ws/src/")
                    return
                openni2.initialize(lib_path)
                logger.debug(f"AstraCamera: OpenNI2 initialized from {lib_path}")
            self._oni_device = openni2.Device.open_any()
            self._depth_stream = self._oni_device.create_depth_stream()
            self._depth_stream.start()
            # Pre-allocate processing buffers now that we know the resolution — P8
            vm = self._depth_stream.get_video_mode()
            dh, dw = vm.resolutionY, vm.resolutionX
            self._depth_buf_8   = np.empty((dh, dw),    dtype=np.uint8)
            self._depth_buf_col = np.empty((dh, dw, 3), dtype=np.uint8)
            logger.info(f"AstraCamera: depth stream started via OpenNI2 (lib: {lib_path})")
        except ImportError:
            logger.warning("AstraCamera: openni not installed — depth unavailable (pip install openni)")
        except Exception as e:
            logger.error(f"AstraCamera: depth init failed: {e}")

    def _close_depth(self):
        """Stop and release the OpenNI2 depth stream."""
        stream = self._depth_stream
        if stream is not None:
            try:
                stream.stop()
            except Exception:
                pass
            self._depth_stream = None
        if self._oni_device:
            self._oni_device = None
        try:
            from openni import openni2
            openni2.unload()
        except Exception:
            pass
        logger.info("AstraCamera: depth stream closed")

    def get_frame(self):
        """Return the latest RGB frame as a BGR numpy array, or None."""
        if self.sim_mode:
            return None
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

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
            buf   = frame.get_buffer_as_uint16()
            depth = np.frombuffer(buf, dtype=np.uint16).reshape(frame.height, frame.width)
            # P8: write into pre-allocated buffers to avoid 3 allocations per depth frame
            cv2.normalize(depth, self._depth_buf_8, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            coloured = cv2.applyColorMap(self._depth_buf_8, cv2.COLORMAP_JET, self._depth_buf_col)
            return cv2.flip(coloured, 1)
        except Exception as e:
            logger.error(f"AstraCamera: depth read error: {e}")
            return None

    def cleanup(self):
        self._running = False
        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
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


# =============================================================================
# OLED DISPLAY (SSD1306 via I2C — Jetson Orin 40-pin: Pin3=SDA, Pin5=SCL)
# =============================================================================

class OLEDDisplay:
    """
    0.91-inch SSD1306 OLED, 128x32 pixels, I2C.
    Jetson Orin 40-pin header: Pin 1=3.3V, Pin 3=SDA, Pin 5=SCL, GND=GND.
    On Jetson Orin the 40-pin I2C maps to bus 7.
    Fits 3 lines of text with the default 8px bitmap font.
    """

    LINE_HEIGHT = 10  # pixels per line; 3 lines × 10px = 30px fits in 32px height

    def __init__(self, i2c_port=7, i2c_address=0x3C, sim_mode=False):
        self.sim_mode = sim_mode
        self._device = None
        self._font = None
        self._lock = threading.Lock()

        if not sim_mode:
            try:
                from luma.core.interface.serial import i2c as luma_i2c
                from luma.oled.device import ssd1306
                from PIL import ImageFont

                serial = luma_i2c(port=i2c_port, address=i2c_address)
                # width=128, height=32 must match the physical panel
                self._device = ssd1306(serial, width=128, height=32)
                self._font = ImageFont.load_default()
                logger.info(f"OLED: SSD1306 128x32 on I2C bus {i2c_port}, addr 0x{i2c_address:02X}")

            except ImportError:
                logger.warning("luma.oled not installed — OLED unavailable (pip install luma.oled)")
            except Exception as e:
                logger.error(f"OLED init failed: {e}")

    def show(self, lines):
        """Render up to 3 lines of text on the 128x32 display."""
        if self._device is None:
            return
        try:
            from luma.core.render import canvas
            with self._lock:
                with canvas(self._device) as draw:
                    for i, line in enumerate(lines[:3]):
                        draw.text((0, i * self.LINE_HEIGHT), str(line),
                                  fill="white", font=self._font)
        except Exception as e:
            logger.error(f"OLED show error: {e}")

    def clear(self):
        if self._device is None:
            return
        try:
            self._device.clear()
        except Exception:
            pass

    def cleanup(self):
        self.clear()
        self._device = None


class YDLidarDriver:
    """
    Driver for YDLidar 4ROS using the ydlidar Python SDK.
    Install on Jetson:
        sudo apt-get install -y cmake swig
        cd ~/Downloads/YDLidar-SDK && pip3 install .
    """

    # YDLidar 4ROS parameters (TOF, 512000 bps, 20K sample rate per spec sheet)
    BAUDRATE     = 512000
    SAMPLE_RATE  = 20      # kHz — TOF, 20000 samples/s per datasheet
    SCAN_FREQ    = 8.0     # Hz — within 5~12 Hz spec range
    MAX_RANGE    = 30.0    # metres — per datasheet
    MIN_RANGE    = 0.05    # metres — per datasheet

    # Point-cloud shaping — tune these to match the physical mounting
    FLIP_HORIZONTAL = True          # mirror X axis (left-right flip)
    SCAN_ANGLE_MAX  = math.radians(135)  # keep ±135° (front 270° arc); rear 90° excluded

    def __init__(self, port="/dev/ttyUSB0", sim_mode=False):
        self.port = port
        self.sim_mode = sim_mode
        self._points = np.empty(0, dtype=np.float32)  # flat [x0,y0,x1,y1,...] — P4
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._laser = None

        # Hardware stays off until start() is called via the GUI toggle

    def _start(self):
        try:
            import ydlidar
            ydlidar.os_init()
            self._laser = ydlidar.CYdLidar()
            self._laser.setlidaropt(ydlidar.LidarPropSerialPort,     self.port)
            self._laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.BAUDRATE)
            self._laser.setlidaropt(ydlidar.LidarPropLidarType,      ydlidar.TYPE_TOF)
            self._laser.setlidaropt(ydlidar.LidarPropDeviceType,     ydlidar.YDLIDAR_TYPE_SERIAL)
            self._laser.setlidaropt(ydlidar.LidarPropScanFrequency,  self.SCAN_FREQ)
            self._laser.setlidaropt(ydlidar.LidarPropSampleRate,     self.SAMPLE_RATE)
            self._laser.setlidaropt(ydlidar.LidarPropSingleChannel,  True)
            self._laser.setlidaropt(ydlidar.LidarPropMaxAngle,       180.0)
            self._laser.setlidaropt(ydlidar.LidarPropMinAngle,      -180.0)
            self._laser.setlidaropt(ydlidar.LidarPropMaxRange,       self.MAX_RANGE)
            self._laser.setlidaropt(ydlidar.LidarPropMinRange,       self.MIN_RANGE)
            self._laser.setlidaropt(ydlidar.LidarPropIntenstiy,      False)

            if not self._laser.initialize():
                logger.error("YDLidar: initialization failed")
                return
            if not self._laser.turnOn():
                logger.error("YDLidar: turnOn failed")
                return

            self._running = True
            self._thread = threading.Thread(target=self._scan_loop, daemon=True)
            self._thread.start()
            logger.info(f"YDLidar: started on {self.port} @ {self.BAUDRATE} baud")

        except ImportError:
            logger.warning("ydlidar not installed — "
                           "run: cd ~/Downloads/YDLidar-SDK && pip3 install .")
        except Exception as e:
            logger.error(f"YDLidar: init failed: {e}")

    def _scan_loop(self):
        """Scan loop — vectorised with numpy (P4).

        Stores points as a flat float32 array [x0, y0, x1, y1, …] so that
        JSON serialisation and decimation are both cheaper than list-of-lists.
        """
        import ydlidar
        scan = ydlidar.LaserScan()
        x_sign = -1.0 if self.FLIP_HORIZONTAL else 1.0
        
        # LLOL idea: rolling buffer — flush every SUB_SCAN_DEG degrees
        SUB_SCAN_DEG = 45  # tune: smaller = lower latency, less context per update
        _partial_xs, _partial_ys = [], []
        _last_angle_deg = None

        while self._running and ydlidar.os_isOk():
            if self._laser.doProcessSimple(scan):
                raw = scan.points
                if not raw:
                    continue
                angles = np.fromiter((pt.angle for pt in raw), dtype=np.float32, count=len(raw))
                ranges = np.fromiter((pt.range for pt in raw), dtype=np.float32, count=len(raw))
                mask = (
                    (ranges > self.MIN_RANGE) &
                    (ranges < self.MAX_RANGE) &
                    (np.abs(angles) <= self.SCAN_ANGLE_MAX)
                )
                a, r = angles[mask], ranges[mask]
                xs = (x_sign * r * np.cos(a)).astype(np.float32)
                ys = (r * np.sin(a)).astype(np.float32)
                
                # --- LLOL sub-scan: flush when arc crosses SUB_SCAN_DEG boundary ---
                if len(a) > 0:
                    current_deg = np.degrees(a[-1])
                    if _last_angle_deg is None or abs(current_deg - _last_angle_deg) >= SUB_SCAN_DEG:
                        _last_angle_deg = current_deg
                        flat = np.empty(len(xs) * 2, dtype=np.float32)
                        flat[0::2] = xs
                        flat[1::2] = ys
                        with self._lock:
                            self._points = flat  # publish partial arc immediately
                        _partial_xs, _partial_ys = [], []
            else:
                time.sleep(0.001)

    def get_points_xy(self, max_points=512):
        """Return a flat [x0, y0, x1, y1, …] Python list, decimated to max_points."""
        with self._lock:
            pts = self._points
        n = len(pts) // 2
        if n == 0:
            return []
        if n <= max_points:
            return pts.tolist()
        step = max(1, n // max_points)
        return pts.reshape(-1, 2)[::step].ravel().tolist()

    def stop(self):
        """Pause scanning: stop the thread and turn off the laser (keeps device initialised)."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._laser:
            try:
                self._laser.turnOff()
            except Exception:
                pass
        with self._lock:
            self._points = np.empty(0, dtype=np.float32)
        logger.info("YDLidar: scan stopped")

    def start(self):
        """Resume scanning after stop(). Re-initialises from scratch if needed."""
        if self.sim_mode:
            return
        if self._laser is None:
            self._start()
            return
        try:
            if not self._laser.turnOn():
                logger.error("YDLidar: turnOn failed on restart")
                return
            self._running = True
            self._thread = threading.Thread(target=self._scan_loop, daemon=True)
            self._thread.start()
            logger.info("YDLidar: scan restarted")
        except Exception as e:
            logger.error(f"YDLidar: restart failed: {e}")

    def cleanup(self):
        self.stop()
        if self._laser:
            try:
                self._laser.disconnecting()
            except Exception:
                pass
        logger.info("YDLidar: disconnected")
