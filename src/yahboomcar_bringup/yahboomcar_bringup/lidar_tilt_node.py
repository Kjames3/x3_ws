#!/usr/bin/env python3
"""lidar_tilt_node — publish the LX-16A tilt mount's angle as a JointState.

Without this node, `lidar_tilt_joint` is never published, so
`joint_state_publisher` pins it at 0 and robot_state_publisher emits a TF that
says the lidar is level no matter where the servo actually is.  Every point
`lidar_3d_processor_node` projects would then be placed as if the scan plane
were horizontal — a flat, wrong "3D" map — and the /scan safety gate would
believe the mount is level forever.

    ros2 run yahboomcar_bringup lidar_tilt_node
    ros2 run yahboomcar_bringup lidar_tilt_node --ros-args -p simulate:=true

Topics (absolute, NOT ~/ -- these must match what server_x3.py publishes and
what joint_state_publisher's source_list points at)
    /lidar_tilt/joint_states  (sensor_msgs/JointState)  measured tilt, ~20 Hz
    /lidar_tilt/cmd_deg       (std_msgs/Float64)        commanded tilt, degrees
    /lidar_tilt/state         (std_msgs/String)         JSON status

Services
    /lidar_tilt/sweep  (std_srvs/Trigger)  step-and-stare sweep
    /lidar_tilt/home   (std_srvs/Trigger)  drive back to level

Sign convention: the URDF's `lidar_tilt_joint` rotates about +Y, so a POSITIVE
joint angle pitches the lidar nose-DOWN.  Whether increasing servo counts move
it that way is recorded as `tilt_direction` in the calibration file, and is
currently unverified (null) — see `_resolve_direction`.  An unknown sign does
NOT compromise the /scan safety gate, which only ever looks at |angle|.

Only one process can hold /dev/lx16a.  Running this node means
`src/lidar_tilt.py` and any server_x3.py servo path must stay off the bus.
"""

import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

# Counts↔degrees for the LX-16A: 0–1000 counts over 240°.  This is NOT the
# scale Rosmaster_Lib.__arm_convert_angle assumes (that one is calibrated for
# the YB-SD15M at 12.2 counts/deg) — feeding LX-16A counts through that maths
# is silently ~3x wrong.
COUNTS_PER_DEG = 1000.0 / 240.0
DEG_PER_COUNT = 240.0 / 1000.0


def _find_driver_dir(start=None):
    """Locate the directory holding lx16a_servo.py.

    Installed nodes live deep under <ws>/install/..., so walk up looking for
    either `<ancestor>/src/lx16a_servo.py` or `<ancestor>/lx16a_servo.py`.
    """
    here = os.path.abspath(start or __file__)
    for _ in range(12):
        here = os.path.dirname(here)
        if not here or here == os.sep:
            break
        for cand in (os.path.join(here, 'src'), here):
            if os.path.isfile(os.path.join(cand, 'lx16a_servo.py')):
                return cand
    for cand in (os.path.expanduser('~/x3_ws/src'), '/home/jetson/x3_ws/src'):
        if os.path.isfile(os.path.join(cand, 'lx16a_servo.py')):
            return cand
    return None


class LidarTiltNode(Node):

    def __init__(self):
        super().__init__('lidar_tilt_node')

        self.declare_parameter('port', '/dev/lx16a')
        self.declare_parameter('servo_id', 3)
        self.declare_parameter('joint_name', 'lidar_tilt_joint')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('publish_rate', 20.0)
        # simulate: no serial at all — report the commanded angle.  Lets the
        # whole projection/octomap pipeline be exercised in Gazebo or on a
        # laptop with no servo attached.
        self.declare_parameter('simulate', False)
        # Step-and-stare sweep.  Holding the mount still for each step means
        # every ray in a dwell shares one exact transform: no motion distortion
        # and no continuous-time trajectory to estimate.
        self.declare_parameter('sweep_min_deg', -30.0)
        self.declare_parameter('sweep_max_deg', 30.0)
        self.declare_parameter('sweep_step_deg', 1.0)
        self.declare_parameter('sweep_dwell_s', 0.35)
        self.declare_parameter('move_time_ms', 150)

        self.port = self.get_parameter('port').value
        self.servo_id = int(self.get_parameter('servo_id').value)
        self.joint_name = self.get_parameter('joint_name').value
        self.simulate = bool(self.get_parameter('simulate').value)

        self.cal = self._load_calibration()
        self.level_counts = int(self.cal.get('horizontal_counts', 500))
        self.direction = self._resolve_direction()

        self._lock = threading.Lock()
        self._sweeping = False
        self._commanded_deg = 0.0
        self._last_deg = 0.0
        self._last_counts = self.level_counts
        self._read_failures = 0
        # Step-and-stare only pays off if consumers ignore the moving part of
        # each step.  `settled` goes false the moment a move is commanded and
        # true again once the servo's travel time (plus a margin) has elapsed.
        self._settle_deadline = 0.0

        self.servo = None
        if not self.simulate:
            self._open_servo()

        self.js_pub = self.create_publisher(JointState, '/lidar_tilt/joint_states', 10)
        self.state_pub = self.create_publisher(String, '/lidar_tilt/state', 10)
        self.create_subscription(Float64, '/lidar_tilt/cmd_deg', self._on_cmd, 10)
        self.create_service(Trigger, '/lidar_tilt/sweep', self._on_sweep)
        self.create_service(Trigger, '/lidar_tilt/home', self._on_home)

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            "lidar_tilt_node up: %s, level=%d counts, direction=%s%s"
            % (self.port if not self.simulate else '(simulated)',
               self.level_counts,
               self.direction if self.direction else 'UNVERIFIED',
               '' if self.servo or self.simulate else ' [servo not open]'))

    # ── setup ───────────────────────────────────────────────────────────
    def _load_calibration(self):
        path = self.get_parameter('calibration_file').value
        if not path:
            drv = _find_driver_dir()
            if drv:
                path = os.path.join(os.path.dirname(drv), 'config',
                                    'lidar_tilt_calibration.json')
        if path and os.path.isfile(path):
            try:
                with open(path) as f:
                    cal = json.load(f)
                self.get_logger().info("calibration: %s" % path)
                return cal
            except Exception as e:
                self.get_logger().error("bad calibration %s: %s" % (path, e))
        self.get_logger().warn(
            "no calibration file found — assuming level = 500 counts, which "
            "is almost certainly wrong.  Run src/lidar_tilt.py --calibrate.")
        return {}

    def _resolve_direction(self):
        """+1 if increasing counts pitch the lidar nose-DOWN (URDF positive)."""
        d = self.cal.get('tilt_direction')
        if d in (1, -1):
            return int(d)
        # Unverified.  Assume +1 so the reported magnitude is right; the /scan
        # safety gate only uses |angle| so it stays correct either way.  Only
        # the 3D projection cares about the sign, and it will simply mirror the
        # cloud about the horizontal plane if this guess is backwards.
        self.get_logger().error(
            "tilt_direction is null in the calibration file: the SIGN of the "
            "tilt is unverified.  Assuming +1 (more counts = nose down).  The "
            "/scan gate is unaffected (it uses |angle|), but a wrong guess "
            "MIRRORS the 3D cloud vertically.  Commanding is refused until "
            "this is set — watch the mount move and record +1 or -1.")
        return None

    def _open_servo(self):
        drv = _find_driver_dir()
        if drv and drv not in sys.path:
            sys.path.insert(0, drv)
        try:
            from lx16a_servo import LX16A  # noqa: E402
        except ImportError as e:
            self.get_logger().error(
                "cannot import lx16a_servo (%s); searched from %s.  Falling "
                "back to simulate mode." % (e, drv))
            self.simulate = True
            return
        if not os.path.exists(self.port):
            self.get_logger().error(
                "%s does not exist — is the CH340 bridge plugged in and "
                "62-lx16a.rules installed?  Falling back to simulate mode."
                % self.port)
            self.simulate = True
            return
        try:
            self.servo = LX16A(self.port)
        except Exception as e:
            self.get_logger().error("cannot open %s: %s" % (self.port, e))
            self.simulate = True

    # ── conversions ─────────────────────────────────────────────────────
    def _counts_to_deg(self, counts):
        sign = self.direction if self.direction else 1
        return (counts - self.level_counts) * DEG_PER_COUNT * sign

    def _deg_to_counts(self, deg):
        sign = self.direction if self.direction else 1
        return int(round(self.level_counts + deg * COUNTS_PER_DEG * sign))

    # ── loop ────────────────────────────────────────────────────────────
    def _tick(self):
        deg = self._commanded_deg
        counts = self._deg_to_counts(deg)
        if self.servo is not None and not self._sweeping:
            try:
                counts = self.servo.read_pos(self.servo_id)
                deg = self._counts_to_deg(counts)
                self._read_failures = 0
            except Exception as e:
                self._read_failures += 1
                if self._read_failures in (1, 50, 500):
                    self.get_logger().warn(
                        "servo read failed (%d in a row): %s — falling back to "
                        "the commanded angle" % (self._read_failures, e))
        with self._lock:
            self._last_deg = deg
            self._last_counts = counts

        settled = time.monotonic() >= self._settle_deadline

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [math.radians(deg)]
        # velocity carries the settled flag (0 = holding still, 1 = moving) so
        # a consumer watching only /joint_states can still tell the difference.
        js.velocity = [0.0 if settled else 1.0]
        self.js_pub.publish(js)

        self.state_pub.publish(String(data=json.dumps({
            'counts': int(counts),
            'deg': round(float(deg), 3),
            'level_counts': self.level_counts,
            'direction_known': self.direction is not None,
            'simulated': bool(self.simulate or self.servo is None),
            'sweeping': self._sweeping,
            'settled': settled,
        })))

    # ── commands ────────────────────────────────────────────────────────
    def _can_command(self):
        if self.direction is None:
            self.get_logger().error(
                "refusing to command tilt: tilt_direction is unverified")
            return False
        if self.servo is None:
            self.get_logger().warn("simulate mode: command accepted, no motion")
        return True

    def _move_to(self, deg, time_ms=None):
        time_ms = time_ms or int(self.get_parameter('move_time_ms').value)
        self._commanded_deg = float(deg)
        # +30% margin: the LX-16A finishes its interpolation at time_ms but
        # settles a beat later, and it reliably stops a few counts short.
        self._settle_deadline = time.monotonic() + (time_ms / 1000.0) * 1.3
        if self.servo is None:
            return
        counts = self._deg_to_counts(deg)
        lo, hi = 0, 1000
        limits = (self.cal.get('servo_state_at_calibration') or {}).get(
            'angle_limit_counts')
        if limits and len(limits) == 2:
            lo, hi = int(limits[0]), int(limits[1])
        clamped = max(lo, min(hi, counts))
        if clamped != counts:
            self.get_logger().warn(
                "tilt %.1f deg = %d counts is outside the servo's limits "
                "[%d, %d]; clamping to %d" % (deg, counts, lo, hi, clamped))
        try:
            self.servo.move(self.servo_id, clamped, time_ms=time_ms)
        except Exception as e:
            self.get_logger().error("servo move failed: %s" % e)

    def _on_cmd(self, msg):
        if self._sweeping:
            self.get_logger().warn("ignoring cmd_deg: a sweep is running")
            return
        if not self._can_command():
            return
        self._move_to(msg.data)

    def _on_home(self, request, response):
        if not self._can_command():
            response.success = False
            response.message = 'tilt_direction unverified'
            return response
        self._move_to(0.0, time_ms=800)
        response.success = True
        response.message = 'homing to level (%d counts)' % self.level_counts
        return response

    def _on_sweep(self, request, response):
        if self._sweeping:
            response.success = False
            response.message = 'sweep already running'
            return response
        if not self._can_command():
            response.success = False
            response.message = 'tilt_direction unverified'
            return response
        threading.Thread(target=self._sweep, daemon=True).start()
        response.success = True
        response.message = 'sweep started'
        return response

    def _sweep(self):
        """Step-and-stare: move, settle, hold while the lidar collects."""
        lo = float(self.get_parameter('sweep_min_deg').value)
        hi = float(self.get_parameter('sweep_max_deg').value)
        step = abs(float(self.get_parameter('sweep_step_deg').value)) or 1.0
        dwell = float(self.get_parameter('sweep_dwell_s').value)
        settle = int(self.get_parameter('move_time_ms').value) / 1000.0 * 1.3
        n = int(abs(hi - lo) / step) + 1
        self.get_logger().info(
            "sweep: %.1f to %.1f deg in %.2f deg steps (%d stations, ~%.0f s)"
            % (lo, hi, step, n, n * (dwell + settle)))
        self._sweeping = True
        try:
            for i in range(n):
                if not rclpy.ok():
                    break
                deg = lo + i * step
                self._move_to(deg)
                # Sleep plainly — the node's executor is spinning in the main
                # thread, so calling spin_once() here would be a second spin on
                # the same node.  _tick keeps publishing throughout.
                # `dwell` is the HOLD time, so wait out the travel first.
                time.sleep(settle + dwell)
            self._move_to(0.0, time_ms=800)
            time.sleep(1.2)
            self.get_logger().info("sweep complete, returned to level")
        finally:
            self._sweeping = False


def main(args=None):
    rclpy.init(args=args)
    node = LidarTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.servo is not None:
            try:
                node.servo.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
