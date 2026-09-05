import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src',
                                'yahboomcar_bringup'))

from yahboomcar_bringup.virtual_scan import (  # noqa: E402
    DEFAULT_Z_MAX, DEFAULT_Z_MIN, ScanAccumulator, synthesize)


def test_empty_cloud_is_all_unknown():
    ranges, counts = synthesize(np.empty((0, 3)))
    assert np.all(np.isinf(ranges)) and counts.sum() == 0


def test_nearest_return_wins_within_a_bin():
    pts = np.array([[2.0, 0.0, 0.2], [1.0, 0.0, 0.2], [3.0, 0.0, 0.2]])
    ranges, counts = synthesize(pts, n_bins=360)
    assert ranges[180] == pytest.approx(1.0)
    assert counts[180] == 3


def test_floor_and_ceiling_returns_are_rejected():
    """The band, not the sweep, is what keeps the floor out of the scan."""
    pts = np.array([[1.5, 0.0, 0.0],            # floor
                    [1.5, 0.0, DEFAULT_Z_MIN - 0.01],
                    [1.5, 0.0, DEFAULT_Z_MAX + 0.01],
                    [1.5, 0.0, 2.4]])           # ceiling
    ranges, counts = synthesize(pts)
    assert counts.sum() == 0 and np.all(np.isinf(ranges))


def test_a_return_anywhere_in_the_band_is_kept_whatever_its_pitch():
    lo = synthesize(np.array([[1.5, 0.0, DEFAULT_Z_MIN + 0.01]]))[0][180]
    hi = synthesize(np.array([[1.5, 0.0, DEFAULT_Z_MAX - 0.01]]))[0][180]
    assert lo == pytest.approx(1.5) and hi == pytest.approx(1.5)


def test_range_reported_is_horizontal_not_slant():
    pts = np.array([[3.0, 0.0, DEFAULT_Z_MAX - 0.01]])
    assert synthesize(pts)[0][180] == pytest.approx(3.0)


def test_bearing_bins_map_to_the_right_direction():
    forward = synthesize(np.array([[2.0, 0.0, 0.2]]))[0]
    left = synthesize(np.array([[0.0, 2.0, 0.2]]))[0]
    assert np.isfinite(forward[180]) and np.isfinite(left[270])
    assert np.isinf(forward[270]) and np.isinf(left[180])


def test_accumulator_expires_a_bin_it_stops_seeing():
    """The F7 failure mode: a stale obstacle must not persist silently."""
    acc = ScanAccumulator(max_age=1.0)
    acc.update(np.array([[2.0, 0.0, 0.2]]), now=0.0)
    assert np.isfinite(acc.read(0.5)[0][180])
    assert np.isinf(acc.read(1.5)[0][180])


def test_accumulator_reports_a_receding_obstacle_not_the_old_minimum():
    acc = ScanAccumulator(max_age=10.0)
    acc.update(np.array([[1.0, 0.0, 0.2]]), now=0.0)
    acc.update(np.array([[3.0, 0.0, 0.2]]), now=1.0)
    assert acc.read(1.0)[0][180] == pytest.approx(3.0)


def test_accumulator_ages_are_reported_per_bin():
    acc = ScanAccumulator(max_age=10.0)
    acc.update(np.array([[2.0, 0.0, 0.2]]), now=0.0)
    acc.update(np.array([[0.0, 2.0, 0.2]]), now=2.0)
    _, ages = acc.read(3.0)
    assert ages[180] == pytest.approx(3.0) and ages[270] == pytest.approx(1.0)
    assert np.isinf(ages[0])


def test_unmeasured_bins_never_claim_an_age():
    acc = ScanAccumulator()
    assert np.all(np.isinf(acc.read(0.0)[1]))
    assert acc.coverage(0.0) == 0.0
