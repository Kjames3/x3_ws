"""traj_predictor: numpy GRU matches torch, history resampling, CV fallback."""
import json

import numpy as np
import pytest

from traj_predictor import NumpyTrajGRU, TrajPredictor, features

torch = pytest.importorskip('torch')
from train_traj_gru import TrajGRU, export_numpy  # noqa: E402


@pytest.mark.parametrize('layers,nll', [(1, True), (2, False)])
def test_numpy_matches_torch(tmp_path, layers, nll):
    torch.manual_seed(0)
    m = TrajGRU(30, hidden=16, layers=layers, nll=nll).eval()
    sx = torch.rand(6) + 0.5
    meta = dict(fut_n=30, hidden=16, layers=layers, nll=nll, rate=10.0, hist_n=11)
    export_numpy(m.state_dict(), sx, meta, tmp_path / 'm.npz')
    x = np.random.default_rng(1).normal(size=(5, 11, 6)).astype(np.float32)
    with torch.no_grad():
        o, s = m(torch.tensor(x) / sx)
    no, ns = NumpyTrajGRU(tmp_path / 'm.npz')(x)
    np.testing.assert_allclose(no, o.numpy(), atol=1e-5)
    if nll:
        np.testing.assert_allclose(ns, s.numpy(), atol=1e-5)


def _walk(p, tid, v=(1.0, 0.0), t0=0.0, n=15, rate=15.0):
    for i in range(n):
        t = t0 + i / rate
        p.observe(tid, t, v[0] * t, v[1] * t, *v)


def test_fallback_is_cv_without_model():
    p = TrajPredictor(None)
    _walk(p, 'a')
    out = p.predict('a')
    assert not out['used_model']
    v, sig = p.cbf_velocity('a', 1.0)
    assert v == pytest.approx((1.0, 0.0)) and sig is None


def test_short_history_falls_back(tmp_path):
    p = TrajPredictor('config/traj_gru_deploy.npz')
    assert p.model is not None
    _walk(p, 'a', n=5)                      # 0.27 s < 1.0 s of history
    assert not p.predict('a')['used_model']
    _walk(p, 'a', t0=5 / 15.0, n=15)
    assert p.predict('a')['used_model']


def test_gap_restarts_track():
    p = TrajPredictor(None)
    _walk(p, 'a')
    p.observe('a', 10.0, 0, 0, 0, 0)
    assert len(p._obs['a']) == 1


def test_resampled_history_is_uniform():
    p = TrajPredictor(None)
    _walk(p, 'a', rate=7.0, n=12)
    h = p.history('a')
    assert h.shape == (11, 4)
    np.testing.assert_allclose(np.diff(h[:, 0]), 0.1, atol=1e-9)


def test_deployed_model_straight_walk_stays_near_cv():
    """A clean 1 m/s straight walk: the model may damp, but not turn or reverse."""
    p = TrajPredictor('config/traj_gru_deploy.npz')
    _walk(p, 'a', v=(0.0, 1.0), n=20)
    v, sig = p.cbf_velocity('a', 1.0)
    assert 0.5 < v[1] < 1.3 and abs(v[0]) < 0.2
    assert sig is not None and 0.01 < sig < 1.0


def test_rotation_invariance_roughly():
    """Trained with random rotations: rotating the walk should rotate the answer."""
    p = TrajPredictor('config/traj_gru_deploy.npz')
    _walk(p, 'x', v=(1.0, 0.0), n=20)
    _walk(p, 'y', v=(0.0, 1.0), n=20)
    vx, _ = p.cbf_velocity('x')
    vy, _ = p.cbf_velocity('y')
    assert vx[0] == pytest.approx(vy[1], abs=0.15)
