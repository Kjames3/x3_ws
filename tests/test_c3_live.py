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
    assert len(tracks) == 1
    t = tracks[0]
    assert abs(t['fwd'] - 2.1) < 0.1
    assert abs(t['vy'] - 0.5) < 0.2 and abs(t['vx']) < 0.2
    assert t['pred_sigma_m'] > t['pos_sigma_m']
