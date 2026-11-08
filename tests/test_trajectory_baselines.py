import numpy as np

import trajectory_baselines as tb


def _line(v, n=11, dt=0.1):
    t = np.arange(n) * dt
    return np.column_stack([t * v[0], t * v[1], np.full(n, v[0]), np.full(n, v[1])])


def test_cv_exact_on_straight_line():
    h = _line((1.0, 0.5))
    for name in ('cv_est', 'cv_fit', 'ctrv'):
        p = tb.predict(name, h, 10, 0.1)
        np.testing.assert_allclose(p[-1], [2.0, 1.0], atol=1e-6)


def test_ctrv_follows_circle():
    w, s, dt = 0.8, 1.0, 0.1
    th = np.arange(31) * dt * w
    xy = np.column_stack([np.sin(th), 1 - np.cos(th)]) * s / w
    h = np.column_stack([xy, np.zeros((31, 2))])
    hist, fut = h[:11], xy[11:31]
    e_ctrv = np.linalg.norm(tb.predict('ctrv', hist, 20, dt) - fut, axis=1)[-1]
    e_cv = np.linalg.norm(tb.predict('cv_fit', hist, 20, dt) - fut, axis=1)[-1]
    assert e_ctrv < 0.5 * e_cv


def test_segments_split_and_windows():
    t = np.r_[np.arange(0, 2, 0.08), np.arange(3, 5, 0.08)]
    tr = np.column_stack([t, t, 0 * t, 0 * t + 1, 0 * t])
    segs = list(tb.segments(tr))
    assert len(segs) == 2
    w = list(tb.windows(segs[0], 11, 5))
    assert len(w) == len(segs[0]) - 15 and w[0][0].shape == (11, 4) and w[0][1].shape == (5, 2)


def test_dedupe_drops_copy():
    a = np.random.rand(20, 5)
    assert len(tb.dedupe({1: a, 2: a.copy(), 3: a + 1})) == 2
