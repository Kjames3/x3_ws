"""Golden replay: real live-C3 measurements through the tracker the robot runs.

tests/data/c3_live_lab_2026-09-28.ndjson is two slices of a --oak-det-log
recorded in the lab after the preview-geometry and capture-time fixes
(artifacts/c3-drive-2026-09-28/c3-after-geometry.ndjson, 665 packets):

  enter_stand  robot parked; a person walks in and stands ~2 m ahead
  turning      the person stands ~2.3 m ahead while the robot turns in place,
               then walks out to 4 m

Each packet keeps the capture time, the odom pose used, and the per-person
measurement (fwd, left, depth, spread).  Depth images are not needed, so the
whole tracker stage replays in milliseconds.

The numbers in GOLDEN are what the code produced when this test was written,
not a spec.  A deliberate tracker or ARM_C_CONFIG change will move them: check
the new numbers are an improvement, then update GOLDEN in the same commit.
Print fresh ones with:  python3 tests/test_c3_golden_replay.py
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src'))
import c3_live  # noqa: E402
import c3_person_tracker as pt  # noqa: E402

FIXTURE = Path(__file__).parent / 'data' / 'c3_live_lab_2026-09-28.ndjson'
FALSE_SPEED = 0.15      # c3_score.FALSE_SPEED: above this a still person "moves"

GOLDEN = {
    'enter_stand': {'packets': 274, 'with_meas': 155, 'coverage': 0.981,
                    'confirmed_ids': 2, 'ghost_packets': 7, 'speed_median': 0.017},
    'turning':     {'packets': 391, 'with_meas': 243, 'coverage': 0.971,
                    'confirmed_ids': 7, 'ghost_packets': 16, 'speed_median': 0.294},
}


def replay(segment, config):
    """[(packet, confirmed tracks)] for one segment.

    Builds each measurement the way C3Live._measurements does: robot-frame
    (fwd, left) rotated into odom by the packet's pose, with the camera
    covariance rotated alongside.
    """
    tracker = pt.KalmanTracker(config)
    floor = config['meas_sigma_floor_m']
    out = []
    for line in FIXTURE.read_text().splitlines():
        pkt = json.loads(line)
        if pkt['seg'] != segment:
            continue
        pose = pkt['pose']
        c, s = math.cos(pose['theta']), math.sin(pose['theta'])
        rot = np.array([[c, -s], [s, c]])
        meas = [(rot @ np.array([m['fwd'], m['left']]) + np.array([pose['x'], pose['y']]),
                 rot @ pt.measurement_cov_camera(m['z'], floor, m['spread']) @ rot.T)
                for m in pkt['meas']]
        tracks = tracker.update(pkt['t'], meas)
        out.append((pkt, [t for t in tracks if t['confirmed']]))
    return out


def summarize(rows):
    with_meas = [(p, c) for p, c in rows if p['meas']]
    speeds = [t['speed'] for _, c in rows for t in c]
    return {
        'packets': len(rows),
        'with_meas': len(with_meas),
        # Share of packets that saw a person and came out with a confirmed track.
        'coverage': round(sum(bool(c) for _, c in with_meas) / len(with_meas), 3),
        'confirmed_ids': len({t['id'] for _, c in rows for t in c}),
        # Packets with no person measured that still carry a (coasting) track.
        'ghost_packets': sum(bool(c) for p, c in rows if not p['meas']),
        'speed_median': round(float(np.median(speeds)), 3),
    }


@pytest.fixture(scope='module', params=sorted(GOLDEN))
def segment(request):
    return request.param, replay(request.param, c3_live.ARM_C_CONFIG)


def test_fixture_is_intact(segment):
    name, rows = segment
    assert len(rows) == GOLDEN[name]['packets']
    assert sum(bool(p['meas']) for p, _ in rows) == GOLDEN[name]['with_meas']
    times = [p['t'] for p, _ in rows]
    assert times == sorted(times)


def test_matches_golden(segment):
    name, rows = segment
    got, want = summarize(rows), GOLDEN[name]
    assert got['coverage'] == pytest.approx(want['coverage'], abs=0.02), got
    assert got['speed_median'] == pytest.approx(want['speed_median'], abs=0.03), got
    assert abs(got['confirmed_ids'] - want['confirmed_ids']) <= 1, got
    assert abs(got['ghost_packets'] - want['ghost_packets']) <= 3, got


def test_coverage_floor(segment):
    # Independent of GOLDEN: a person the camera measured must have a ring.
    _, rows = segment
    assert summarize(rows)['coverage'] >= 0.95


def test_tracks_are_confirmed_only_after_enough_hits(segment):
    _, rows = segment
    hits = 0
    for pkt, confirmed in rows:
        hits += bool(pkt['meas'])
        if confirmed:
            break
    assert hits >= c3_live.ARM_C_CONFIG['confirm_hits']


def test_coasting_track_expires(segment):
    # After the person leaves, a ring may coast for confirmed_max_age_s, no longer.
    _, rows = segment
    last_meas_t = None
    for pkt, confirmed in rows:
        if pkt['meas']:
            last_meas_t = pkt['t']
        elif confirmed and last_meas_t is not None:
            age = pkt['t'] - last_meas_t
            assert age <= c3_live.ARM_C_CONFIG['confirmed_max_age_s'] + 1e-6, age


def test_standing_person_reads_still_with_the_robot_parked():
    rows = replay('enter_stand', c3_live.ARM_C_CONFIG)
    t0 = rows[0][0]['t']
    # 15-35 s: the person has arrived and is standing.
    speeds = [t['speed'] for p, c in rows if 15.0 <= p['t'] - t0 < 35.0 for t in c]
    assert len(speeds) > 100
    assert float(np.median(speeds)) < FALSE_SPEED


def test_live_config_is_the_one_under_test():
    # c3_calibration found the replay default over-confident next to the live
    # config; scoring one while flying the other is the trap this guards.
    assert c3_live.ARM_C_CONFIG != pt.DEFAULT_CONFIG
    assert c3_live.ARM_C_CONFIG['accel_sigma_mps2'] == 1.5
    assert c3_live.ARM_C_CONFIG['meas_sigma_floor_m'] == 0.05
    assert set(c3_live.ARM_C_CONFIG) == set(pt.DEFAULT_CONFIG)


if __name__ == '__main__':
    for seg in sorted(GOLDEN):
        print(seg, summarize(replay(seg, c3_live.ARM_C_CONFIG)))
