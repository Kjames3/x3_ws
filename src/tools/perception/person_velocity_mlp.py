"""Velocity MLP on measured person positions, with externally supplied track IDs.

Uses the deployed model/scaler and 10 Hz feature convention. No Kalman-smoothed
positions enter the MLP. Call only for measured observations, never coasting.
This adapter is independent of detector, association and robot hardware.
"""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

from collections import deque
import math

import numpy as np
import torch

from velocity_estimator import VelocityEstimator
from velocity_timing import resample_positions, MAX_SAMPLE_GAP_S


def planar_basis(rotation):
    """Camera (forward, left) unit axes expressed in world xy."""
    rot = np.asarray(rotation, dtype=float)
    forward = rot[:2, 2]
    length = np.linalg.norm(forward)
    if not np.isfinite(rot).all() or length < 1e-6:
        raise ValueError('camera planar axes are degenerate')
    forward = forward / length
    # World positions are already flattened into the ground plane. Use an
    # orthonormal yaw basis rather than projected, potentially skew optical axes.
    return np.column_stack((forward, [-forward[1], forward[0]]))


class PersonVelocityMLP:
    def __init__(self, model_path=None, estimator=None):
        if estimator is None and model_path is None:
            raise ValueError('model_path is required to select the model/scaler pair')
        self.estimator = estimator or VelocityEstimator(None, None, model_path=model_path)
        if self.estimator._model is None:
            raise ValueError('velocity model did not load')
        self.histories = {}
        self.previous = {}

    def reset(self):
        self.histories.clear()
        self.previous.clear()

    def retain(self, active_ids):
        """Bound memory to identities still retained by the caller's tracker."""
        keep = set(active_ids)
        self.histories = {k: v for k, v in self.histories.items() if k in keep}
        self.previous = {k: v for k, v in self.previous.items() if k in keep}

    def update(self, track_id, stamp, measured_xy, rotation):
        xy = np.asarray(measured_xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all() or not math.isfinite(stamp):
            self.histories.pop(track_id, None)
            self.previous.pop(track_id, None)
            return None
        h = self.histories.setdefault(track_id, deque(maxlen=64))
        if h and stamp <= h[-1][0]:
            return None
        if h and stamp - h[-1][0] > MAX_SAMPLE_GAP_S:
            h.clear()
            self.previous.pop(track_id, None)
        h.append((stamp, (float(xy[0]), float(xy[1]), 0.)))
        window = resample_positions(h)
        if window is None:
            return None
        basis = planar_basis(rotation)
        local = window[:, :2] @ basis
        history = [(-left, 0., forward) for forward, left in local]
        est = self.estimator
        features, stopped = est._build_window_features(history)
        if stopped:
            velocity = np.zeros(2)
        else:
            x = ((features - est.scaler_X_mean) * est.scaler_X_inv_scale).astype(np.float32)
            with torch.no_grad():
                y = est._model(torch.from_numpy(x)).numpy()[0]
            velocity = np.clip(y * est.scaler_y_scale + est.scaler_y_mean, -2.5, 2.5)
            velocity = velocity * min(1.0, len(h) / 10.0)
        if track_id in self.previous:
            prev_t, prev_world = self.previous[track_id]
            previous_local = prev_world @ basis
            delta = 3. * (stamp - prev_t)
            velocity = np.clip(velocity, previous_local - delta, previous_local + delta)
        world_velocity = basis @ velocity
        self.previous[track_id] = (stamp, world_velocity)
        return dict(id=track_id, x=float(xy[0]), y=float(xy[1]),
                    vx=float(world_velocity[0]), vy=float(world_velocity[1]),
                    speed=float(np.linalg.norm(world_velocity)), measured=True,
                    confirmed=True, capture_t=stamp)
