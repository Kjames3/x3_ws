
import logging
import threading
import sys
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
def _find_rosmaster_port():
    """Locate the Rosmaster mainboard, never the LX16A servo bus.

    Both boards are CH340s with identical VID:PID and no serial number, so
    /dev/ttyCH341USB0 and USB1 swap with hub enumeration order. Picking by
    index would, half the time, push Rosmaster motor frames into the lidar
    tilt servo. Prefer the udev symlink (src/63-rosmaster.rules keys on the
    chip revision); if it is missing, at least exclude whatever /dev/lx16a
    currently points at.
    """
    if os.path.exists("/dev/rosmaster"):
        return "/dev/rosmaster"
    servo = os.path.realpath("/dev/lx16a") if os.path.exists("/dev/lx16a") else None
    for candidate in ("/dev/ttyCH341USB0", "/dev/ttyCH341USB1", "/dev/ttyUSB0"):
        if os.path.exists(candidate) and os.path.realpath(candidate) != servo:
            return candidate
    return "/dev/ttyUSB0"  # Fallback for standard kernel driver


SERIAL_PORT = _find_rosmaster_port()

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
          2. Scan /dev/video0..9 and match by USB vendor/product via sysfs.
             The sysfs 'device' symlink points to the USB interface node
             (e.g. 1-2.2.1.1:1.0).  One level up (..) is the USB device node
             which holds idVendor/idProduct.
          3. Fall back to the first /dev/videoN that exists
        """
        import glob

        # 1. udev symlink
        if os.path.exists("/dev/camera_depth"):
            return "/dev/camera_depth"

        # 2. sysfs scan — find which videoX belongs to the Orbbec RGB
        for video_path in sorted(glob.glob("/dev/video?")):
            dev_name = os.path.basename(video_path)
            # device → USB interface; device/.. → USB device with idVendor/idProduct
            vendor_file  = f"/sys/class/video4linux/{dev_name}/device/../idVendor"
            product_file = f"/sys/class/video4linux/{dev_name}/device/../idProduct"
            try:
                with open(vendor_file) as f:
                    vendor = f.read().strip()
                with open(product_file) as f:
                    product = f.read().strip()
                if vendor == self.ORBBEC_RGB_VENDOR and product == self.ORBBEC_RGB_PRODUCT:
                    logger.debug(f"AstraCamera: found Orbbec RGB at {video_path} via sysfs")
                    return video_path
            except Exception:
                continue

        # 3. last resort — use the first video device that exists
        for n in range(10):
            path = f"/dev/video{n}"
            if os.path.exists(path):
                logger.warning(f"AstraCamera: sysfs detection failed, falling back to {path}")
                return path
        logger.warning("AstraCamera: no /dev/videoN found")
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
            lib_path = "system default"
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
            try:
                err_msg = getattr(e, 'message', None) or str(getattr(e, 'status', "depth init error"))
            except Exception:
                err_msg = "Unknown OpenNI2 error (possibly camera in use by another process)"
            logger.error(f"AstraCamera: depth init failed: {err_msg}")

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
            err_msg = getattr(e, 'message', None) or str(getattr(e, 'status', e))
            logger.error(f"AstraCamera: depth read error: {err_msg}")
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


# =============================================================================
# INA226 BATTERY MONITOR (I2C)
# =============================================================================

# INA226 register map + fixed scales from the datasheet.
_INA226_REG_SHUNT_V = 0x01   # signed, 2.5 uV/LSB
_INA226_REG_BUS_V   = 0x02   # unsigned, 1.25 mV/LSB
_INA226_SHUNT_LSB_V = 2.5e-6
_INA226_BUS_LSB_V   = 1.25e-3

# Shunt resistance of the breakout actually fitted to this robot.  Measured
# 2026-08-31 against the live chip: the shunt register sits at ~3.1 mV with the
# robot idle, which is 1.55 A across 2 mOhm and matches the 1.21-1.36 A idle
# draw in the full-discharge trace (logs/battery/battery_20260815-132549.csv).
# A 0.1 Ohm shunt would make that same reading 31 mA / 0.39 W, which cannot run
# an Orin Nano -- so the value is pinned by an order of magnitude, not a guess.
_INA226_SHUNT_OHMS = 0.002


class INA226BatteryMonitor:
    """
    Driver for INA226 Voltage/Current monitor over I2C.

    Wiring (verified live): the INA226 is the ONLY device on **i2c-7**
    (40-pin header pins 3=SDA / 5=SCL) at address 0x40.  Pins 27/28 are i2c-1,
    which carries the OLED plus the devkit's own INA3221 -- also at 0x40 -- so
    do not move this chip there.

    Current and power are computed **in software** from the shunt-voltage
    register rather than read from the chip's current/power registers.  Those
    registers return 0 unless the calibration register (0x05) is programmed,
    and it is 0 on this board; computing from the shunt needs no config write,
    so it cannot clobber a setting some other process depends on.
    """
    def __init__(self, i2c_bus=7, i2c_addr=0x40, max_voltage=12.6, min_voltage=9.6,
                 shunt_ohms=_INA226_SHUNT_OHMS):
        self.bus_num = i2c_bus
        self.addr = i2c_addr
        self.max_voltage = max_voltage
        self.min_voltage = min_voltage
        self.shunt_ohms = shunt_ohms
        self.bus = None
        try:
            import smbus2
            self.bus = smbus2.SMBus(self.bus_num)
            logger.info(f"INA226 initialized on I2C bus {self.bus_num} at addr 0x{self.addr:02X}")
        except ImportError:
            logger.warning("smbus2 not installed — INA226 unavailable (run: pip install smbus2)")
        except Exception as e:
            logger.error(f"INA226 init failed on bus {self.bus_num}: {e}")

    def _read_reg(self, reg):
        """One raw 16-bit register.  INA226 is big-endian, smbus2 is little."""
        word = self.bus.read_word_data(self.addr, reg)
        return ((word << 8) & 0xFF00) + (word >> 8)

    def get_voltage(self):
        if self.bus is None:
            return 0.0
        try:
            return self._read_reg(_INA226_REG_BUS_V) * _INA226_BUS_LSB_V
        except Exception as e:
            logger.error(f"INA226 read error: {e}")
            return 0.0

    def get_current(self):
        """Measured pack current in amps.  Positive = discharge.

        Returns None (not 0.0) when the chip cannot be read, so a caller can
        tell "no sensor" apart from "genuinely drawing nothing" -- the battery
        estimator must not coulomb-count a failed read as an idle pack.
        """
        if self.bus is None:
            return None
        try:
            raw = self._read_reg(_INA226_REG_SHUNT_V)
            if raw > 32767:            # register is signed two's complement
                raw -= 65536
            return (raw * _INA226_SHUNT_LSB_V) / self.shunt_ohms
        except Exception as e:
            logger.error(f"INA226 shunt read error: {e}")
            return None

    def get_power(self):
        """Measured pack power in watts, or None if the chip cannot be read."""
        amps = self.get_current()
        if amps is None:
            return None
        return self.get_voltage() * amps

    def get_percentage(self, voltage=None):
        """Crude voltage-only gauge.  Kept ONLY as a cold-start fallback.

        This pack is 4S LiFePO4: 85%->98% SoC spans 5 mV, which is below the
        sensor's own 1.25 mV LSB, while sag at idle is ~78 mV.  On the real
        discharge trace this mapping read 48.9% with 18.0% actually left.  Use
        battery.BatteryEstimator (coulomb counting) wherever there is current.
        """
        if voltage is None:
            voltage = self.get_voltage()
        if voltage <= 0.0:
            return 0.0

        # Simple linear mapping
        pct = (voltage - self.min_voltage) / (self.max_voltage - self.min_voltage) * 100.0
        return max(0.0, min(100.0, pct))
        
    def cleanup(self):
        if self.bus:
            self.bus.close()
            self.bus = None


# ---------------------------------------------------------------------------
# VL53L5CX multizone time-of-flight array
# ---------------------------------------------------------------------------
# Wiring (decided 2026-09-09 from the live bus audit, see notes below):
#   VIN  -> header pin 1  (3.3V).  NOT pins 2/4: those are 5V, the Orin header
#           is not 5V tolerant, and several breakouts reference their level
#           shifter to VIN.  Pin 1 is free now the OLED is unplugged.
#   SDA/SCL -> header pins 27/28 = **i2c-1**, the quiet bus.  Do NOT put it on
#           i2c-7 next to the ICM-42688-P: that part is read at 200 Hz and this
#           one pulls a ~90 KB firmware blob at every init.
#   Address 0x29, fixed in silicon and not strappable.  Two sensors therefore
#   need separate LPn lines to re-address one at boot.
#
# The pure-smbus2 approach used for the INA226 and the ICM does NOT work here:
# the VL53L5CX has no usable register map until the host uploads that firmware
# image, so this wraps `vl53l5cx_ctypes` (the ST ULD C API via ctypes).
_VL53L5CX_I2C_BUS = 1
_VL53L5CX_I2C_ADDR = 0x29

def _load_tof_geometry():
    """Load tof_geometry.py BY PATH, not as `yahboomcar_bringup.tof_geometry`.

    The colcon install space also provides a `yahboomcar_bringup` package, and
    under systemd (which sources install/setup.bash) that installed copy SHADOWS
    the source tree.  A stale install without tof_geometry then makes the import
    fail with ModuleNotFoundError -- which the old `except ImportError: pass`
    here swallowed, leaving frame_to_points=None and read_points() silently
    returning None forever on the robot while working fine from a plain shell.
    Loading the file directly sidesteps package precedence entirely.
    """
    import importlib.util
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'yahboomcar_bringup', 'yahboomcar_bringup',
                        'tof_geometry.py')
    spec = importlib.util.spec_from_file_location('x3_tof_geometry', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# Deliberately NOT wrapped in try/except: tof_geometry is pure numpy shipped in
# this repo, so a failure here is a broken checkout, not an optional dependency.
_tof_geom = _load_tof_geometry()
frame_to_points = _tof_geom.frame_to_points
tof_zone_low_edge_points = _tof_geom.zone_low_edge_points
tof_valid_mask = _tof_geom.valid_mask
FOV_DEG = _tof_geom.FOV_DEG
VALID_STATUS = _tof_geom.VALID_STATUS
DEFAULT_MAX_RANGE_M = _tof_geom.DEFAULT_MAX_RANGE_M
DEFAULT_MIN_RANGE_M = _tof_geom.DEFAULT_MIN_RANGE_M


_vl53l5cx_patched = False


def _patch_vl53l5cx_for_64bit():
    """Make vl53l5cx_ctypes usable on 64-bit ARM.  Call before constructing.

    vl53l5cx_ctypes 0.0.3 declares NO restype or argtypes anywhere, so ctypes
    assumes every function returns a 32-bit int.  ``get_configuration()``
    actually returns a malloc'd pointer, which is therefore truncated to 32
    bits; the first dereference inside ``vl53l5cx_is_alive()`` then segfaults
    the interpreter.  It fails BEFORE any I2C traffic, so it looks nothing like
    a wiring fault -- an i2cdetect that finds 0x29 tells you nothing about it.

    Upstream never hit this because a 32-bit Raspberry Pi OS pointer survives
    the truncation.  Handing the configuration back as a c_void_p *instance*
    (not a Python int) also makes ctypes pass all 64 bits on every later call,
    which is why no argtypes are needed on the rest of the API.
    """
    global _vl53l5cx_patched
    if _vl53l5cx_patched:
        return
    import ctypes
    import vl53l5cx_ctypes
    lib = vl53l5cx_ctypes._VL53
    for name in ("get_configuration", "get_motion_configuration"):
        fn = getattr(lib, name, None)
        if fn is None:
            continue
        fn.restype = ctypes.c_void_p
        setattr(lib, name,
                (lambda f: lambda *a: ctypes.c_void_p(f(*a)))(fn))
    _vl53l5cx_patched = True


class VL53L5CXArray:
    """VL53L5CX 4x4/8x8 ToF array on i2c-1 @ 0x29 (bench path; the robot now
    uses TeensyToFArrays below).

    Covers the low blind band the Phase-0 numbers pin down exactly: the scan
    plane sits at 0.340 m, the camera is floor-blind under ~0.57 m, and the
    costmap's ``min_obstacle_height`` is 0.12 m -- so a box on the floor in
    front of the wheels is invisible to every other sensor on the robot.

    Degrades the same way INA226BatteryMonitor does: a missing module or a
    missing sensor logs and leaves the object usable but empty, rather than
    taking the server down.  ``read_frame()`` returns None when there is no
    sensor, which is NOT the same as an all-invalid frame -- callers must not
    collapse the two.
    """

    RESOLUTION_HZ_MAX = {4: 60, 8: 15}   # ULD ceiling; 8x8 above 15 Hz is rejected

    def __init__(self, i2c_bus=_VL53L5CX_I2C_BUS, i2c_addr=_VL53L5CX_I2C_ADDR,
                 resolution=8, ranging_freq_hz=10, sharpener_percent=5,
                 max_range_m=None, sim_mode=False,
                 transpose=False, flip_h=False, flip_v=False):
        self.bus_num = i2c_bus
        self.addr = i2c_addr
        self.resolution = int(resolution)
        self.ranging_freq_hz = int(ranging_freq_hz)
        self.sharpener_percent = int(sharpener_percent)
        self.max_range_m = DEFAULT_MAX_RANGE_M if max_range_m is None else float(max_range_m)
        self.sim_mode = sim_mode
        self.order = dict(transpose=transpose, flip_h=flip_h, flip_v=flip_v)
        self.tof = None
        self._sim_phase = 0.0

        if self.resolution not in self.RESOLUTION_HZ_MAX:
            raise ValueError("resolution must be 4 or 8")
        if self.ranging_freq_hz > self.RESOLUTION_HZ_MAX[self.resolution]:
            # The ULD silently clamps this; clamping loudly instead means a
            # configured 30 Hz 8x8 does not quietly run at 15 and look like a
            # dropped-frame bug somewhere downstream.
            logger.warning(
                f"VL53L5CX: {self.ranging_freq_hz} Hz is above the "
                f"{self.RESOLUTION_HZ_MAX[self.resolution]} Hz ceiling for "
                f"{self.resolution}x{self.resolution}; clamping")
            self.ranging_freq_hz = self.RESOLUTION_HZ_MAX[self.resolution]

        if sim_mode:
            logger.info("VL53L5CX: simulation mode (synthetic floor + wall)")
            return
        try:
            import vl53l5cx_ctypes
            from smbus2 import SMBus
            _patch_vl53l5cx_for_64bit()
            # The library takes an OPEN SMBus, not a bus number -- there is no
            # bus_id argument, and passing one raises TypeError inside the
            # constructor AFTER __del__ is already live, which surfaces as a
            # confusing "no attribute _configuration" traceback first.
            self._bus = SMBus(self.bus_num)
            # ~84 KB firmware upload, about 8 s on this bus.  Do not call this
            # on a hot path and do not construct two of these for one sensor.
            self.tof = vl53l5cx_ctypes.VL53L5CX(i2c_addr=self.addr,
                                                i2c_dev=self._bus)
            self.tof.set_resolution(self.resolution * self.resolution)
            self.tof.set_ranging_frequency_hz(self.ranging_freq_hz)
            # CONTINUOUS, not the ULD's default AUTONOMOUS. Measured 2026-09-14,
            # 8x8 on the 100 kHz i2c-1: autonomous with a 64 ms integration
            # time made the sensor itself withhold data-ready for ~132 ms per
            # frame (3.8 Hz); continuous removes that wait entirely (7.4 Hz).
            # Integration time only applies in autonomous mode, so it is not set.
            # What remains is the bus: ~133 ms to read 1621 bytes at 100 kHz.
            self.tof.set_ranging_mode(vl53l5cx_ctypes.RANGING_MODE_CONTINUOUS)
            self.tof.set_sharpener_percent(self.sharpener_percent)
            self.tof.start_ranging()
            logger.info(
                f"VL53L5CX initialized on i2c-{self.bus_num} @ 0x{self.addr:02X}, "
                f"{self.resolution}x{self.resolution} @ {self.ranging_freq_hz} Hz")
        except ImportError:
            logger.warning("vl53l5cx_ctypes not installed — ToF array unavailable "
                           "(run: pip install vl53l5cx-ctypes)")
        except Exception as e:
            logger.error(f"VL53L5CX init failed on i2c-{self.bus_num}: {e}")
            self.tof = None

    @property
    def available(self):
        return self.sim_mode or self.tof is not None

    def data_ready(self):
        if self.sim_mode:
            return True
        if self.tof is None:
            return False
        try:
            return bool(self.tof.data_ready())
        except Exception as e:
            logger.error(f"VL53L5CX data_ready error: {e}")
            return False

    def read_frame(self):
        """Raw frame as ``(distance_mm, target_status)``, or None if no sensor.

        Both arrays are length resolution^2 in the ULD's own zone order -- the
        reordering lives in tof_geometry so the bench tool and the ROS node
        cannot disagree about it.
        """
        if self.sim_mode:
            return self._sim_frame()
        if self.tof is None:
            return None
        try:
            d = self.tof.get_data()
            n2 = self.resolution * self.resolution
            # These fields are (NB_TARGET_PER_ZONE, 64), so the FIRST index is
            # the target, not the zone.  Slicing [:n2] takes whole target rows
            # and silently yields a (1, 64) array that still prints as a grid
            # -- index [0] to get the per-zone row.
            return (np.asarray(d.distance_mm[0][:n2], dtype=np.float64),
                    np.asarray(d.target_status[0][:n2], dtype=np.int32))
        except Exception as e:
            logger.error(f"VL53L5CX read error: {e}")
            return None

    def read_points(self, min_range_m=DEFAULT_MIN_RANGE_M):
        """Valid returns as an ``(N, 3)`` float32 cloud in the SENSOR frame.

        Returns None when there is no sensor at all, and an empty (0, 3) array
        when the sensor is fine but nothing came back in range.  Keep those
        distinct: an empty cloud means "nothing measured", never "nothing there".
        """
        frame = self.read_frame()
        if frame is None:
            return None
        pts, _ = frame_to_points(frame[0], frame[1], resolution=self.resolution,
                                 min_range_m=min_range_m,
                                 max_range_m=self.max_range_m, **self.order)
        return pts

    def _sim_frame(self):
        """Synthetic frame: a flat floor plus a wall, no hardware required.

        Deliberately geometric rather than random -- a sim frame that produces
        a recognisable plane is one you can actually validate the node against
        before the part arrives.
        """
        n = self.resolution
        if frame_to_points is None:                  # geometry import failed
            return (np.full(n * n, 1000.0), np.full(n * n, 5, dtype=np.int32))
        from yahboomcar_bringup.tof_geometry import ray_table
        rays = ray_table(n)
        self._sim_phase += 0.05
        wall_x = 1.2 + 0.3 * math.sin(self._sim_phase)
        mount_h, pitch = 0.055, math.radians(15.0)   # matches the default mount
        cp, sp = math.cos(pitch), math.sin(pitch)
        # Rotate each ray into the base frame (pitch down about +y) and find the
        # nearer of the floor (z=0) and a wall at x=wall_x.
        bx = rays[:, 0] * cp + rays[:, 2] * sp
        bz = -rays[:, 0] * sp + rays[:, 2] * cp
        with np.errstate(divide='ignore', invalid='ignore'):
            t_floor = np.where(bz < -1e-9, -mount_h / bz, np.inf)
            t_wall = np.where(bx > 1e-9, wall_x / bx, np.inf)
        t = np.minimum(t_floor, t_wall)
        dist = t * 1000.0
        status = np.where(np.isfinite(dist) & (dist <= self.max_range_m * 1000.0),
                          5, 255).astype(np.int32)
        dist = np.where(status == 5, dist, 0.0)
        return dist, status

    def cleanup(self):
        if self.tof is not None:
            try:
                self.tof.stop_ranging()
            except Exception:
                pass
            self.tof = None
        # Own the SMBus we opened for the library; leaving it open holds an fd
        # on i2c-1 and blocks a re-init in the same process.
        bus = getattr(self, "_bus", None)
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass
            self._bus = None


TEENSY_TOF_PORT = '/dev/teensy_tof'   # src/65-teensy-tof.rules


class TeensyToFArrays:
    """Upper + lower VL53L5CX 8x8 arrays streamed by a Teensy over USB serial.

    Firmware: scripts/teensy_tof_test/teensy_tof_test.ino (8x8, 15 Hz,
    continuous, sharpener 5%, one I2C bus per sensor).  Frames are NDJSON,
    validated by src/teensy_tof_serial.py so the bench tool and the server
    parse the protocol identically.

    A reader thread owns the port and keeps only the newest frame per sensor;
    ``latest()`` never blocks.  Unplugging the Teensy (or the hub) is survived
    by reopening every 2 s.  ``/dev/ttyACMn`` numbering swaps with the
    OpenRB-150 between boots, so use /dev/teensy_tof (65-teensy-tof.rules).
    """

    SENSORS = ('upper', 'lower')

    def __init__(self, port=TEENSY_TOF_PORT, max_range_m=None, on_frame=None):
        """``on_frame(name, seq, dist, status)`` runs on the reader thread for
        EVERY frame -- use it for consumers that must not drop frames when the
        asyncio loop is busy (the ROS clouds).  Exceptions are logged, not raised.
        """
        from teensy_tof_serial import LineDecoder
        self.on_frame = on_frame
        self.port = port
        self.max_range_m = DEFAULT_MAX_RANGE_M if max_range_m is None else float(max_range_m)
        self.connected = False
        self._decoder_cls = LineDecoder
        self._lock = threading.Lock()
        self._frames = {}      # name -> (seq, dist ndarray, status ndarray, monotonic)
        self._active = {}      # name -> bool from the firmware's status/stats records
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='teensy-tof', daemon=True)
        self._thread.start()

    @property
    def available(self):
        return True   # the thread keeps retrying; per-sensor freshness is in latest()

    def latest(self, name):
        """``(seq, distance_mm, target_status, age_s)`` or None if never received."""
        with self._lock:
            f = self._frames.get(name)
        if f is None:
            return None
        return f[0], f[1], f[2], time.monotonic() - f[3]

    def sensor_active(self, name):
        return self._active.get(name)

    def _run(self):
        try:
            import serial
        except ImportError:
            logger.warning("pyserial not installed — Teensy ToF unavailable")
            return
        logged_missing = False
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, 115200, timeout=0.2,
                                   write_timeout=1, exclusive=True) as ser:
                    ser.write(b'r')   # raw frames, in case a monitor left benchmark mode on
                    self.connected = True
                    logged_missing = False
                    logger.info(f"Teensy ToF connected on {self.port}")
                    decoder = self._decoder_cls()
                    while not self._stop.is_set():
                        chunk = ser.read(max(ser.in_waiting, 1))
                        for rec in decoder.feed(chunk):
                            self._handle(rec)
            except Exception as e:
                if self.connected or not logged_missing:
                    logger.warning(f"Teensy ToF on {self.port}: {e}; retrying every 2 s")
                    logged_missing = True
            self.connected = False
            self._stop.wait(2.0)

    def _handle(self, rec):
        name = rec['sensor']
        if rec['type'] == 'frame':
            dist = np.asarray(rec['distance_mm'], dtype=np.float64)
            status = np.asarray(rec['target_status'], dtype=np.int32)
            with self._lock:
                self._frames[name] = (rec['seq'], dist, status, time.monotonic())
            if self.on_frame is not None:
                try:
                    self.on_frame(name, rec['seq'], dist, status)
                except Exception as e:
                    logger.error(f"Teensy ToF on_frame({name}): {e}")
        elif 'active' in rec:
            if self._active.get(name) != rec['active'] and not rec['active']:
                logger.warning(f"Teensy ToF: {name} sensor reported inactive")
            self._active[name] = bool(rec['active'])

    def cleanup(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
