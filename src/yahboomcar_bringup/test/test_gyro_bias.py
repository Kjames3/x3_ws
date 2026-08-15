#!/usr/bin/env python3
"""Offline tests for the gyro Z bias estimator and stop-motion gate.

These bind the real methods from Mcnamu_driver_X3 onto a stub so the logic can
be exercised with no Rosmaster serial port, no ROS graph and no robot. The
stub supplies only what the two methods touch: get_parameter, get_clock and
get_logger.

The measured bias on the physical robot was -0.003404 rad/s (-11.70 deg/min),
which is the value used in the drift regression below.
"""
import importlib.util
import math
import os
import sys
import types
import unittest

MEASURED_BIAS = -0.003404      # rad/s, sampled from the robot's MPU9250
MEASURED_NOISE_STD = 0.000579  # rad/s, standard deviation of those samples

DEFAULTS = {
    'gyro_bias_calib_samples': 50,
    'gyro_bias_ema_alpha': 0.002,
    'gyro_bias_max_rate': 0.05,
    'gyro_zero_deadband': 0.005,
    'stationary_settle_time': 0.5,
    'encoder_still_threshold': 0.01,
    'gyro_zero_gate_enabled': True,
}


def _load_driver_module():
    """Import Mcnamu_driver_X3 with its hardware dependency stubbed out."""
    here = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.join(here, os.pardir, 'yahboomcar_bringup')
    path = os.path.abspath(os.path.join(pkg_dir, 'Mcnamu_driver_X3.py'))

    pkg = types.ModuleType('ybc_stub')
    pkg.__path__ = [os.path.abspath(pkg_dir)]
    sys.modules['ybc_stub'] = pkg

    rosmaster = types.ModuleType('ybc_stub.Rosmaster_Lib')
    rosmaster.Rosmaster = object
    sys.modules['ybc_stub.Rosmaster_Lib'] = rosmaster

    spec = importlib.util.spec_from_file_location('ybc_stub.Mcnamu_driver_X3', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['ybc_stub.Mcnamu_driver_X3'] = mod
    spec.loader.exec_module(mod)
    return mod


DRIVER = _load_driver_module()


class _FakeParam:
    def __init__(self, value):
        self.value = value


class _FakeNow:
    """Nanosecond clock stand-in supporting the `(a - b).nanoseconds` idiom."""

    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return types.SimpleNamespace(
            nanoseconds=(self.seconds - other.seconds) * 1e9)


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(msg)

    def warn(self, msg):
        self.messages.append(msg)


class FakeDriver:
    """Minimal stub carrying the real methods under test."""

    def __init__(self, **overrides):
        self.params = dict(DEFAULTS)
        self.params.update(overrides)
        self.now = 0.0
        self._logger = _FakeLogger()
        self._gyro_bias = 0.0
        self._gyro_bias_sum = 0.0
        self._gyro_bias_count = 0
        self._gyro_bias_ready = False
        self._last_nonzero_cmd_time = _FakeNow(0.0)

    # -- shims the real methods call --
    def get_parameter(self, name):
        return _FakeParam(self.params[name])

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: _FakeNow(self.now))

    def get_logger(self):
        return self._logger

    # -- methods under test, bound from the production class --
    _is_stationary = DRIVER.yahboomcar_driver._is_stationary
    _gyro_z_compensated = DRIVER.yahboomcar_driver._gyro_z_compensated

    # -- helpers --
    def tick(self, gz_ros, vx=0.0, vy=0.0, dt=0.1):
        self.now += dt
        return self._gyro_z_compensated(gz_ros, vx, vy)

    def command_motion(self):
        self._last_nonzero_cmd_time = _FakeNow(self.now)

    def settle(self, gz_ros=0.0):
        """Advance past stationary_settle_time so samples start counting."""
        while not self._is_stationary(0.0, 0.0):
            self.tick(gz_ros)

    def calibrate(self, gz_ros=MEASURED_BIAS):
        """Run until the bias estimate is ready."""
        self.settle(gz_ros)
        while not self._gyro_bias_ready:
            self.tick(gz_ros)


class TestStationaryDetection(unittest.TestCase):
    def test_not_stationary_immediately_after_a_motion_command(self):
        d = FakeDriver()
        d.now = 10.0
        d.command_motion()
        d.now += 0.2  # still inside the 0.5 s settle window
        self.assertFalse(d._is_stationary(0.0, 0.0))

    def test_stationary_after_settle_time(self):
        d = FakeDriver()
        d.now = 10.0
        d.command_motion()
        d.now += 0.6
        self.assertTrue(d._is_stationary(0.0, 0.0))

    def test_turning_wheels_block_stationary(self):
        d = FakeDriver()
        d.now = 10.0
        self.assertFalse(d._is_stationary(0.5, 0.0))
        self.assertFalse(d._is_stationary(0.0, 0.5))


class TestBiasCalibration(unittest.TestCase):
    def test_bias_converges_to_measured_value(self):
        d = FakeDriver()
        d.calibrate()
        self.assertTrue(d._gyro_bias_ready)
        self.assertAlmostEqual(d._gyro_bias, MEASURED_BIAS, places=6)
        self.assertTrue(any('bias calibrated' in m for m in d._logger.messages))

    def test_noisy_bias_estimate_is_accurate_enough(self):
        """With realistic sensor noise the residual must beat 1 deg/min."""
        import random
        random.seed(1234)
        d = FakeDriver()
        d.settle(MEASURED_BIAS)
        while not d._gyro_bias_ready:
            d.tick(random.gauss(MEASURED_BIAS, MEASURED_NOISE_STD))
        residual_deg_min = abs(math.degrees(d._gyro_bias - MEASURED_BIAS) * 60.0)
        self.assertLess(residual_deg_min, 1.0)

    def test_motion_samples_never_pollute_the_estimate(self):
        d = FakeDriver()
        # A sustained real rotation while "commanded idle" (robot being pushed):
        # far above gyro_bias_max_rate, so it must be rejected outright.
        for _ in range(200):
            d.tick(1.0)
        self.assertFalse(d._gyro_bias_ready)
        self.assertEqual(d._gyro_bias, 0.0)

    def test_no_calibration_while_the_robot_is_driving(self):
        d = FakeDriver()
        for _ in range(200):
            d.command_motion()
            d.tick(MEASURED_BIAS, vx=0.3)
        self.assertFalse(d._gyro_bias_ready)


class TestStopMotionGate(unittest.TestCase):
    def test_stationary_output_is_exactly_zero(self):
        d = FakeDriver()
        d.calibrate()
        for _ in range(100):
            self.assertEqual(d.tick(MEASURED_BIAS), 0.0)

    def test_gate_suppresses_bias_during_calibration_window(self):
        """Before the estimate is ready the raw bias must not leak into odom."""
        d = FakeDriver()
        d.settle(MEASURED_BIAS)
        for _ in range(10):
            self.assertEqual(d.tick(MEASURED_BIAS), 0.0)
        self.assertFalse(d._gyro_bias_ready)

    def test_real_rotation_passes_through_the_gate(self):
        """A genuine hand-turn must not be swallowed by the deadband."""
        d = FakeDriver()
        d.calibrate()
        turn = 0.4  # rad/s, a slow but real rotation
        out = d.tick(turn)
        self.assertAlmostEqual(out, turn - MEASURED_BIAS, places=6)

    def test_commanded_rotation_is_not_gated(self):
        d = FakeDriver()
        d.calibrate()
        d.command_motion()
        out = d.tick(0.002, vx=0.0)  # tiny rate, but we ARE commanding motion
        self.assertNotEqual(out, 0.0)

    def test_gate_can_be_disabled(self):
        d = FakeDriver(gyro_zero_gate_enabled=False)
        d.calibrate()
        # Bias correction still applies, but no hard zero.
        self.assertNotEqual(d.tick(MEASURED_BIAS + 0.001), 0.0)


class TestDriftRegression(unittest.TestCase):
    """The end-to-end property that motivated the fix."""

    def _integrate_yaw_deg(self, rates, dt=0.1):
        return math.degrees(sum(r * dt for r in rates))

    def test_idle_drift_is_eliminated(self):
        import random
        random.seed(7)
        d = FakeDriver()
        duration_s = 30 * 60.0   # 30 minutes parked
        steps = int(duration_s / 0.1)

        uncorrected = []
        corrected = []
        for _ in range(steps):
            sample = random.gauss(MEASURED_BIAS, MEASURED_NOISE_STD)
            uncorrected.append(sample)
            corrected.append(d.tick(sample))

        before = abs(self._integrate_yaw_deg(uncorrected))
        after = abs(self._integrate_yaw_deg(corrected))

        # Reproduces the observed failure: ~360 deg over ~31 min.
        self.assertGreater(before, 300.0)
        # And the fix must leave essentially nothing.
        self.assertLess(after, 1.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
