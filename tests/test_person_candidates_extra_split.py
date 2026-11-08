"""person_candidates.extract(extra_split_z=...): seated people merged with furniture."""
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src' / 'yahboomcar_bringup'))

from yahboomcar_bringup import person_candidates as pc  # noqa: E402


def _scene(rng):
    # A 3 m long low sofa / bench (0.2-0.8 m) that a seated person (head ~1.05 m) touches.
    sx = rng.uniform(1.0, 4.0, 3000); sy = rng.uniform(-0.2, 0.2, 3000); sz = rng.uniform(0.2, 0.8, 3000)
    a = rng.uniform(0, 2 * np.pi, 600); r = rng.uniform(0, 0.17, 600)
    px = 2.5 + r * np.cos(a); py = 0.3 + r * np.sin(a); pz = rng.uniform(0.3, 1.05, 600)
    return np.r_[sx, px], np.r_[sy, py], np.r_[sz, pz]


def _near(c, x, y, tol=0.3):
    return len(c) and np.hypot(c[:, 0] - x, c[:, 1] - y).min() < tol


def test_seated_person_only_found_with_extra_split():
    x, y, z = _scene(np.random.default_rng(0))
    assert not _near(pc.extract(x, y, z), 2.5, 0.3)
    assert _near(pc.extract(x, y, z, extra_split_z=0.9), 2.5, 0.3)


def test_extra_split_keeps_default_pieces():
    x, y, z = _scene(np.random.default_rng(1))
    base = pc.extract(x, y, z)
    extra = pc.extract(x, y, z, extra_split_z=0.9)
    for b in base:
        assert _near(extra, b[0], b[1], 1e-6)
