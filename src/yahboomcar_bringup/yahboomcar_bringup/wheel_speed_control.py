"""Closed-loop wheel speed for the X3's four mecanum wheels.

Pure Python, no ROS and no serial port, so it can be tested against a motor
model. Mcnamu_driver_X3 calls update() once per encoder report from the board.

Why: the driver used to send open-loop PWM = 200 * v + 28. On the apartment's
wood floor (2026-10-11, artifacts/base-2026-10-11/ident-r1) that ran at about
0.22 + 2 * v m/s: a 0.01 m/s command drove at 0.22 m/s and 0.20 at 0.65. The
28 PWM that breaks static friction on carpet is 17 PWM too much once the wheels
turn on wood, and the 200 PWM per m/s slope is over twice the real one.

Measured on that run, all four wheels within 2%:
  encoder reports every 40 ms; 10,100 counts per metre of rim travel
  counts DEcrease when a wheel drives the robot forward
  forward: speed = 0.01145 * (PWM - 11) m/s; strafing needs ~22 PWM of offset
  the robot follows a PWM step with a time constant of about 0.28 s

Control per wheel: feedforward from that model, plus PI on the speed error.
The gains are the IMC rule for a first-order plant with ~60 ms of delay and a
0.2 s closed-loop time constant.
"""

COUNTS_PER_M = 10100.0
REPORT_PERIOD_S = 0.040
KFF_PWM_PER_MPS = 87.0      # 1 / 0.01145
OFFSET_PWM = 11.0           # kinetic friction, forward on wood
KP_PWM_PER_MPS = 90.0
KI_PWM_PER_M = 320.0        # Kp / 0.28 s
PWM_MAX = 100.0
INTEGRAL_MAX_PWM = 50.0
ACCEL_MPS2 = 1.5            # target slew while speeding up
DECEL_MPS2 = 3.0            # ... and while slowing down
STALL_TARGET_MPS = 0.02     # below this a wheel is not expected to turn
STALL_SPEED_MPS = 0.005
STALL_BOOST_PWM = 1.5       # added per report while a commanded wheel is stuck
STALL_BOOST_MAX_PWM = 25.0
BRAKE_MAX_PWM = 60.0
STOP_SPEED_MPS = 0.03       # with a zero target, below this the motor is released
TRACK_WINDOW_MPS = 0.25     # how far the ramped target may lead the wheel
MAX_GAP_REPORTS = 5         # more missed reports than this: speed is unknown


def _sign(v):
    return (v > 0) - (v < 0)


def mecanum_targets(vx, vy, omega, wheel_separation):
    """Rim speeds (m/s) in the board's motor order M1..M4 = FL, RL, FR, RR.

    vy > 0 is a left strafe (FL -, RL +, FR +, RR -); omega > 0 is a left turn
    (left wheels -, right wheels +). wheel_separation is half the wheelbase
    plus half the track.
    """
    r = omega * wheel_separation
    return [vx - vy - r, vx + vy - r, vx + vy + r, vx - vy + r]


def limit_targets(targets, max_speed):
    """Scale all four together when one exceeds max_speed, keeping direction."""
    top = max(abs(t) for t in targets)
    if top <= max_speed or top == 0.0:
        return list(targets)
    k = max_speed / top
    return [t * k for t in targets]


class WheelSpeedController:
    def __init__(self, counts_per_m=COUNTS_PER_M, period=REPORT_PERIOD_S,
                 kff=KFF_PWM_PER_MPS, offset=OFFSET_PWM, kp=KP_PWM_PER_MPS,
                 ki=KI_PWM_PER_M):
        self.counts_per_m = counts_per_m
        self.period = period
        self.kff, self.offset, self.kp, self.ki = kff, offset, kp, ki
        self.targets = [0.0] * 4
        self.reset()

    def reset(self):
        """Forget everything measured: after a mode switch or a gap in reports."""
        self._counts = None
        self._seq = None
        self.ramped = [0.0] * 4       # slew-limited targets
        self.speeds = [0.0] * 4       # last measured rim speeds, m/s
        self.pwm = [0] * 4
        self._integral = [0.0] * 4
        self._boost = [0.0] * 4

    def set_targets(self, targets):
        self.targets = [float(t) for t in targets]

    def update(self, counts, seq):
        """One encoder report: counts for M1..M4 and the board's report number.

        Returns the PWM for set_motor(), M1..M4, as integers in [-100, 100].
        """
        if self._counts is None:
            self._counts, self._seq = list(counts), seq
            return list(self.pwm)
        n = seq - self._seq
        if n <= 0:
            return list(self.pwm)
        if n > MAX_GAP_REPORTS:
            self.reset()
            self._counts, self._seq = list(counts), seq
            return list(self.pwm)
        dt = n * self.period
        self.speeds = [-(c - p) / (dt * self.counts_per_m)
                       for c, p in zip(counts, self._counts)]
        self._counts, self._seq = list(counts), seq

        out = []
        for i in range(4):
            target, r = self.targets[i], self.ramped[i]
            speeding_up = abs(target) > abs(r) and target * r >= 0.0
            step = (ACCEL_MPS2 if speeding_up else DECEL_MPS2) * dt
            r += max(-step, min(step, target - r))
            w = self.speeds[i]
            # Never let the ramp run away from the wheel (a speed it cannot
            # reach, a blocked wheel): the error, and with it the integral,
            # stays bounded and the way back down starts from reality.
            if target != 0.0:
                r = max(w - TRACK_WINDOW_MPS, min(w + TRACK_WINDOW_MPS, r))
                r = max(-abs(target), min(abs(target), r))
            self.ramped[i] = r

            if r == 0.0 and abs(w) < STOP_SPEED_MPS:
                self._integral[i] = self._boost[i] = 0.0
                out.append(0)
                continue

            if abs(r) >= STALL_TARGET_MPS and abs(w) < STALL_SPEED_MPS:
                self._boost[i] = max(-STALL_BOOST_MAX_PWM, min(
                    STALL_BOOST_MAX_PWM, self._boost[i] + STALL_BOOST_PWM * _sign(r)))
            elif self._boost[i]:
                # It moves: static friction is gone, and so is the push for it,
                # along with whatever the integral collected while stuck.
                self._boost[i] = self._integral[i] = 0.0

            error = r - w
            feedforward = self.kff * r + self.offset * _sign(r)
            limit = BRAKE_MAX_PWM if r == 0.0 else PWM_MAX
            raw = feedforward + self.kp * error + self._integral[i] + self._boost[i]
            u = max(-limit, min(limit, raw))
            if r > 0.0:
                u = max(0.0, u)       # slow to a lower speed by easing off,
            elif r < 0.0:
                u = min(0.0, u)       # never by driving the wheel backwards
            # Integrate only once the ramp has arrived (feedforward and the
            # proportional term carry the transit; an integral collected there
            # comes out as overshoot), and never further into the limit.
            if r == target and (u == raw or (raw > u) != (error > 0.0)):
                self._integral[i] = max(-INTEGRAL_MAX_PWM, min(
                    INTEGRAL_MAX_PWM, self._integral[i] + self.ki * error * dt))
            out.append(int(round(u)))
        self.pwm = out
        return list(out)
