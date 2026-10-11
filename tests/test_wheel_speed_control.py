"""Closed-loop wheel speed (Mcnamu_driver_X3) against a simple motor model."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'yahboomcar_bringup'))

from yahboomcar_bringup.wheel_speed_control import (  # noqa: E402
    COUNTS_PER_M, REPORT_PERIOD_S, WheelSpeedController, limit_targets, mecanum_targets)


class Base:
    """Four identical wheels: first-order speed response, Coulomb friction,
    static friction to break away, one report of delay. Numbers from ident-r1."""

    def __init__(self, gain=0.01145, kinetic=11.0, static=24.0, tau=0.28):
        self.gain, self.kinetic, self.static, self.tau = gain, kinetic, static, tau
        self.v = [0.0] * 4
        self.counts = [0.0] * 4
        self.seq = 0
        self.pending = [0] * 4          # PWM takes effect one report later

    def step(self, pwm, substeps=8):
        applied, self.pending = self.pending, list(pwm)
        h = REPORT_PERIOD_S / substeps
        for _ in range(substeps):
            for i, p in enumerate(applied):
                v = self.v[i]
                if v == 0.0 and abs(p) <= self.static:
                    continue
                direction = (v > 0) - (v < 0) or (p > 0) - (p < 0)
                steady = self.gain * (p - self.kinetic * direction)
                new = v + (steady - v) * h / self.tau
                # friction can stop a wheel but not reverse it
                if v != 0.0 and new * v < 0.0 and abs(p) <= self.static:
                    new = 0.0
                self.v[i] = new
                self.counts[i] -= new * h * COUNTS_PER_M
        self.seq += 1
        return [int(round(c)) for c in self.counts], self.seq


def run(ctrl, base, targets, seconds, log=None):
    ctrl.set_targets(targets)
    for _ in range(int(round(seconds / REPORT_PERIOD_S))):
        counts, seq = base.step(ctrl.pwm)
        ctrl.update(counts, seq)
        if log is not None:
            log.append((list(base.v), list(ctrl.pwm)))


def fresh():
    ctrl, base = WheelSpeedController(), Base()
    counts, seq = base.step([0] * 4)
    ctrl.update(counts, seq)
    return ctrl, base


def test_mecanum_targets_match_the_driver_mixing():
    assert mecanum_targets(0.2, 0.0, 0.0, 0.165) == [0.2, 0.2, 0.2, 0.2]
    assert mecanum_targets(0.0, 0.1, 0.0, 0.165) == [-0.1, 0.1, 0.1, -0.1]      # left strafe
    fl, rl, fr, rr = mecanum_targets(0.0, 0.0, 1.0, 0.165)                      # left turn
    assert fl == rl == -0.165 and fr == rr == 0.165


def test_limit_targets_scales_all_four_together():
    assert limit_targets([0.2, -0.2, 0.2, -0.2], 0.8) == [0.2, -0.2, 0.2, -0.2]
    assert limit_targets([1.6, 0.8, -0.4, 0.0], 0.8) == pytest.approx([0.8, 0.4, -0.2, 0.0])


@pytest.mark.parametrize('speed', [0.05, 0.10, 0.22, 0.45, -0.10, -0.45])
def test_holds_the_commanded_speed(speed):
    ctrl, base = fresh()
    run(ctrl, base, [speed] * 4, 2.0)
    assert all(v == pytest.approx(speed, abs=0.01) for v in base.v)


def test_the_old_open_loop_error_is_gone():
    """0.01 m/s used to run at 0.22 and 0.20 at 0.65."""
    ctrl, base = fresh()
    run(ctrl, base, [0.20] * 4, 2.5)
    assert base.v[0] == pytest.approx(0.20, abs=0.01)


def test_slow_start_breaks_static_friction_without_a_lurch():
    ctrl, base = fresh()
    log = []
    run(ctrl, base, [0.05] * 4, 3.0, log)
    speeds = [v[0] for v, _ in log]
    assert max(speeds) < 0.12                         # the old minimum was 0.22
    assert min(i for i, s in enumerate(speeds) if s > 0.02) * REPORT_PERIOD_S < 0.6
    assert speeds[-1] == pytest.approx(0.05, abs=0.01)


def test_zero_target_brakes_then_releases_the_motors():
    ctrl, base = fresh()
    run(ctrl, base, [0.45] * 4, 2.0)
    log = []
    run(ctrl, base, [0.0] * 4, 1.5, log)
    stopped = min(i for i, (v, _) in enumerate(log) if abs(v[0]) < 0.01) * REPORT_PERIOD_S
    assert stopped < 0.5                              # coasting from 0.45 takes ~1 s
    assert log[-1][1] == [0, 0, 0, 0]
    assert all(abs(p) <= 60 for _, pwm in log for p in pwm)
    assert min(v[0] for v, _ in log) > -0.05          # no drive in reverse


def test_strafe_and_turn_track_per_wheel():
    ctrl, base = fresh()
    targets = mecanum_targets(0.0, 0.15, 0.0, 0.165)
    run(ctrl, base, targets, 2.0)
    assert base.v == pytest.approx(targets, abs=0.01)
    ctrl, base = fresh()
    targets = mecanum_targets(0.1, 0.0, 0.8, 0.165)
    run(ctrl, base, targets, 2.0)
    assert base.v == pytest.approx(targets, abs=0.01)


def test_a_weak_wheel_is_brought_up_to_speed():
    ctrl, base = fresh()
    base_step = base.step

    def weak(pwm, substeps=8):                         # M3 gets 80% of its PWM
        return base_step([pwm[0], pwm[1], int(pwm[2] * 0.8), pwm[3]], substeps)
    base.step = weak
    run(ctrl, base, [0.2] * 4, 3.0)
    assert base.v == pytest.approx([0.2] * 4, abs=0.01)


def test_unreachable_speed_does_not_wind_up():
    ctrl, base = fresh()
    run(ctrl, base, [2.0] * 4, 3.0)                    # PWM 100 gives ~1.0 m/s
    assert ctrl.pwm == [100] * 4
    log = []
    run(ctrl, base, [0.2] * 4, 1.5, log)
    assert base.v[0] == pytest.approx(0.2, abs=0.02)
    assert min(v[0] for v, _ in log) > 0.12            # no undershoot from a stale integral


def test_a_gap_in_reports_resets_instead_of_spiking():
    ctrl, base = fresh()
    run(ctrl, base, [0.2] * 4, 1.0)
    counts, seq = base.step(ctrl.pwm)
    pwm = ctrl.update([c - 5000 for c in counts], seq + 20)    # 0.8 s of silence
    assert pwm == [0, 0, 0, 0]
    assert ctrl.speeds == [0.0] * 4


def test_repeated_or_first_report_does_nothing():
    ctrl = WheelSpeedController()
    ctrl.set_targets([0.2] * 4)
    assert ctrl.update([0, 0, 0, 0], 7) == [0, 0, 0, 0]
    first = ctrl.update([-80, -80, -80, -80], 8)
    assert ctrl.update([-80, -80, -80, -80], 8) == first
