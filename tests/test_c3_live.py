import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import c3_live  # noqa: E402


class FakeOak:
    """A person walking left-to-right 2 m ahead at 0.5 m/s, 15 Hz packets."""
    def __init__(self):
        self.t0 = time.monotonic()
        self.depth = np.full((640, 480), 2.0, np.float32)

    def get_depth_intrinsics(self):
        return (400.0, 400.0, 240.0, 320.0, 480, 640)

    def get_raw_depth_frame(self):
        return self.depth

    @property
    def _latest_detections_t(self):
        return round((time.monotonic() - self.t0) * 15) / 15 + self.t0

    def get_detection_observation(self):
        left = 0.5 * (self._latest_detections_t - self.t0) - 0.5   # m, moving right->left
        u = 240 - left * 400 / 2.0
        return [{'label': 'person', 'bbox': [int(u - 60), 100, int(u + 60), 600]}], self._latest_detections_t


def test_tracks_a_walking_person_in_robot_frame():
    trk = c3_live.C3Live(FakeOak(), lambda: {'x': 1.0, 'y': 2.0, 'theta': 0.7}, mount_x=0.1)
    trk.start()
    time.sleep(1.5)
    trk.stop()
    tracks, stats = trk.get_tracks()
    assert stats['updates'] > 10 and '_prev' not in stats
    d = stats['diag']
    assert d['person_packets'] == d['dets'] == d['meas'] > 10
    assert d['ring_packets'] >= d['person_packets'] - 3   # 3 hits to confirm
    assert d['reset_pose'] == d['reset_depth'] == d['reset_gap'] == 0
    assert len(tracks) == 1
    t = tracks[0]
    assert abs(t['fwd'] - 2.1) < 0.1
    assert abs(t['vy'] - 0.5) < 0.2 and abs(t['vx']) < 0.2
    assert t['pred_sigma_m'] > t['pos_sigma_m']


class TurningOak:
    """Robot turning at 2 rad/s about a person standing still 3 m away in odom
    (at (3, 0)). Each NN packet shows the scene as it was LATENCY_S earlier,
    as the real OAK does; depth history carries capture times."""
    LATENCY_S = 0.3
    W = 2.0

    def __init__(self):
        self.t0 = time.monotonic()
        self.fx = 400.0
        self._cap = None

    def theta(self, t):
        return 0.4 * math.sin(self.W * (t - self.t0) / 0.4)   # +/-0.4 rad sweep, peak 2 rad/s

    def pose(self):
        return {'x': 0.0, 'y': 0.0, 'theta': self.theta(time.monotonic())}

    @property
    def _latest_detections_t(self):
        return round((time.monotonic() - self.t0) * 15) / 15 + self.t0

    def get_detection_capture_time(self):
        # The capture time of the packet last handed out (as the driver does).
        return self._cap

    def get_depth_intrinsics(self):
        return (self.fx, self.fx, 240.0, 320.0, 480, 640)

    def get_raw_depth_frame(self):
        return np.full((640, 480), 3.0, np.float32)

    def get_depth_near(self, t, max_gap_s=0.05):
        # Depth is distance along the optical axis: 3 cos(theta) at capture time.
        return (t, np.full((640, 480), 3.0 * math.cos(self.theta(t)), np.float32))

    def get_detection_observation(self):
        self._cap = self._latest_detections_t - self.LATENCY_S
        th = self.theta(self._cap)
        # Person at odom (3, 0) seen from a robot at the origin with heading th.
        fwd, left = 3.0 * math.cos(th), -3.0 * math.sin(th)
        u = 240 - left / fwd * self.fx
        return [{'label': 'person', 'bbox': [int(u - 60), 100, int(u + 60), 600]}], None


def test_still_person_stays_still_while_the_robot_turns():
    oak = TurningOak()
    trk = c3_live.C3Live(oak, oak.pose, mount_x=0.0)
    trk.start()
    time.sleep(2.5)
    trk.stop()
    tracks, stats = trk.get_tracks()
    assert stats['diag']['timed'] > 20
    assert len(tracks) == 1
    assert tracks[0]['speed'] < 0.3, tracks[0]
    # Drawn relative to the robot as it is now, not at capture time.
    th = oak.theta(time.monotonic())
    fwd, left = 3.0 * math.cos(th), -3.0 * math.sin(th)
    assert math.hypot(tracks[0]['fwd'] - fwd, tracks[0]['left'] - left) < 0.3, (tracks[0], fwd, left)
    assert stats['diag']['latency_ms_avg'] > 200
