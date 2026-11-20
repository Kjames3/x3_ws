"""IMU heading hold for the X3's mecanum base.

Pure Python, no ROS, so it can be tested against a simulated drifting body.
Mcnamu_driver_X3 feeds it the IMU yaw rate (integrate_rate) and passes every
cmd_vel through correct() before the wheel mixing.

Why: on the new chassis a commanded pure strafe or straight drive
(angular.z = 0) yaws the body, because the rear wheels track worse under
load. While the operator commands translation without rotation, this holds
the yaw the body had when the translation started by adding a small
angular-rate correction (PI on the yaw error).

A zero Twist always passes through as exactly zero: stop commands bypass the
safety filter and must stay pure zero.
"""
import math


class HeadingHold:
    def __init__(self, kp=2.0, ki=0.5, max_corr=0.3, engage_speed=0.03,
                 omega_deadband=0.02, imu_timeout=0.2, i_limit=0.15):
        self.kp = kp
        self.ki = ki
        self.max_corr = max_corr
        self.engage_speed = engage_speed
        self.omega_deadband = omega_deadband
        self.imu_timeout = imu_timeout
        self.i_limit = i_limit
        self.yaw = 0.0              # integrated IMU yaw, unbounded (rad)
        self._imu_t = None          # time of the last IMU sample
        self._ref = None            # yaw to hold while engaged
        self._integral = 0.0
        self._last_t = None         # time of the previous correct() call

    def integrate_rate(self, t, wz):
        """Integrate yaw from one IMU rate sample (t in s, wz in rad/s)."""
        prev = self._imu_t
        self._imu_t = t
        if prev is None:
            return
        dt = t - prev
        if dt <= 0.0 or dt > 0.1:
            return
        self.yaw += wz * dt

    def reset(self):
        self._ref = None
        self._integral = 0.0
        self._last_t = None

    def correct(self, t, vx, vy, omega):
        """Return the angular rate to command (omega plus the correction)."""
        if math.hypot(vx, vy) < self.engage_speed:
            self.reset()
            return omega
        if abs(omega) > self.omega_deadband:
            self.reset()
            return omega
        if self._imu_t is None or abs(t - self._imu_t) > self.imu_timeout:
            self.reset()
            return omega
        if self._ref is None:
            self._ref = self.yaw
            self._integral = 0.0
            self._last_t = t
        err = self._ref - self.yaw
        dt = t - self._last_t
        self._last_t = t
        if dt > 0.2:
            self._integral = 0.0
        elif dt > 0.0:
            self._integral += err * dt
            self._integral = max(-self.i_limit, min(self.i_limit, self._integral))
        corr = self.kp * err + self.ki * self._integral
        corr = max(-self.max_corr, min(self.max_corr, corr))
        return omega + corr
