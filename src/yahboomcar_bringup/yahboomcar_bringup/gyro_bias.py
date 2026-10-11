"""Gyro glitch rejection and zero-rate bias tracking for icm42688_node.

Pure Python, no ROS and no I2C, so it can be tested and replayed offline.

Why (straight out-and-back, 2026-10-10, artifacts/exp2/straight01):
  - the bias in config/icm42688_calibration.json (2026-09-04) had gone stale by
    +0.0026 rad/s on z (8.8 deg/min), so /odom heading drifted 26 deg in 3 min
    with the robot never turning;
  - one sample in 35,617 read 34.9 rad/s (full scale) between two quiet ones,
    which is 10 deg in anything that integrates the stream.

Thresholds come from that run, in 1 s windows at 200 Hz, worst axis:
  parked   gyro sd 0.0010-0.0011 rad/s, accel sd ~0.02 m/s^2
  driving  gyro sd >= 0.046 rad/s,      accel sd >= 0.4 m/s^2
"""
import math

# A step this large between consecutive 5 ms samples is ~1000 rad/s^2, far
# beyond anything the chassis or the 42 Hz anti-alias filter can produce.
GLITCH_STEP_RAD_S = 5.0
GLITCH_RESYNC = 3          # consecutive rejects before the new level is believed

QUIET_GYRO_SD = 0.004      # rad/s, ~4x parked, ~11x below the quietest driving
QUIET_ACCEL_SD = 0.1       # m/s^2
QUIET_WINDOWS = 2          # consecutive quiet windows before the bias moves
MAX_BIAS_RAD_S = 0.02      # a quiet mean beyond this is not a bias (1.1 deg/s)
BIAS_GAIN = 0.2            # per quiet window, after the first snap


class GlitchFilter:
    """Reject a gyro sample that jumps impossibly far from the last good one."""

    def __init__(self, step=GLITCH_STEP_RAD_S, resync=GLITCH_RESYNC):
        self.step = step
        self.resync = resync
        self.last = None
        self.streak = 0
        self.rejected = 0

    def accept(self, gyro):
        if self.last is not None and self.streak < self.resync and any(
                abs(g - l) > self.step for g, l in zip(gyro, self.last)):
            self.streak += 1
            self.rejected += 1
            return False
        self.last = tuple(gyro)
        self.streak = 0
        return True


class BiasTracker:
    """Estimate the gyro bias from windows in which the robot is plainly still.

    Feed every accepted sample to update(); `bias` is what to subtract. The
    first usable quiet window replaces the starting bias outright, later ones
    pull it by BIAS_GAIN, so a stale calibration file is corrected within a
    couple of seconds of boot and thermal drift is followed while parked.
    """

    def __init__(self, bias=(0.0, 0.0, 0.0), window=200, gyro_sd=QUIET_GYRO_SD,
                 accel_sd=QUIET_ACCEL_SD, quiet_windows=QUIET_WINDOWS,
                 max_bias=MAX_BIAS_RAD_S, gain=BIAS_GAIN):
        self.bias = [float(b) for b in bias]
        self.window = int(window)
        self.gyro_sd = gyro_sd
        self.accel_sd = accel_sd
        self.quiet_windows = quiet_windows
        self.max_bias = max_bias
        self.gain = gain
        self.updates = 0
        self._quiet_run = 0
        self._clear()

    def _clear(self):
        self._n = 0
        self._sum = [0.0] * 6
        self._sq = [0.0] * 6

    def reset_window(self):
        """Drop the partial window, e.g. after a rejected or missing sample."""
        self._clear()
        self._quiet_run = 0

    def update(self, gyro, accel):
        """Add one raw (bias NOT removed) sample. True when the bias moved."""
        for i, v in enumerate((*gyro, *accel)):
            self._sum[i] += v
            self._sq[i] += v * v
        self._n += 1
        if self._n < self.window:
            return False
        n = self._n
        mean = [s / n for s in self._sum]
        sd = [math.sqrt(max(q / n - m * m, 0.0)) for q, m in zip(self._sq, mean)]
        self._clear()
        quiet = (max(sd[:3]) < self.gyro_sd and max(sd[3:]) < self.accel_sd
                 and max(abs(m) for m in mean[:3]) < self.max_bias)
        if not quiet:
            self._quiet_run = 0
            return False
        self._quiet_run += 1
        if self._quiet_run < self.quiet_windows:
            return False
        k = 1.0 if self.updates == 0 else self.gain
        self.bias = [b + k * (m - b) for b, m in zip(self.bias, mean[:3])]
        self.updates += 1
        return True
