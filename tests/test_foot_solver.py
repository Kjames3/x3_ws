"""Geometry edge cases and independent numerical oracle for the shadow solver."""
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from foot_diagnostic import minimum_norm_velocity, shadow_cbf


def test_geometry_and_speed_boundary():
    solve = minimum_norm_velocity
    assert np.allclose(solve([], [], .15), [0, 0])
    assert np.allclose(solve([[1, 0]], [-.15], .15), [.15, 0])
    assert solve([[1, 0]], [-.15001], .15) is None
    assert np.allclose(solve([[1, 0], [0, 1]], [-.06, -.08], .15), [.06, .08])
    assert solve([[1, 0], [0, 1]], [-.12, -.12], .15) is None
    assert solve([[1, 0], [-1, 0]], [-.01, -.01], .15) is None
    assert np.allclose(solve([[1, 0], [2, 0]], [-.05, -.2], .15), [.1, 0])
    assert np.allclose(solve([[1, 0], [-1, 0]], [-.1, .1], .15), [.1, 0])
    assert solve([[0, 0]], [-.001], .15) is None
    assert np.allclose(solve([[0, 0]], [0], .15), [0, 0])
    assert solve([[float('nan'), 0]], [0], .15) is None
    assert np.allclose(solve([[1, 0], [1, 1e-9]], [-.1, -.10000000001], .15), [.1, 0], atol=1e-8)


def test_random_constraints_against_independent_optimizer():
    rng = np.random.default_rng(420)
    for _ in range(200):
        a = rng.normal(size=(rng.integers(1, 7), 2))
        b = rng.uniform(-.2, .3, len(a))
        result = minimum_norm_velocity(a, b, .15)
        oracle = minimize(lambda u: .5*u@u, np.zeros(2), jac=lambda u: u,
                          constraints=[{'type': 'ineq', 'fun': lambda u: a@u+b,
                                        'jac': lambda u: a},
                                       {'type': 'ineq', 'fun': lambda u: .15**2-u@u,
                                        'jac': lambda u: -2*u}],
                          method='SLSQP', options={'maxiter': 200, 'ftol': 1e-12})
        if result is not None:
            assert np.min(a@result+b) >= -1e-9
            assert np.linalg.norm(result) <= .15+1e-9
            assert oracle.success
            assert np.allclose(result, oracle.x, atol=1e-6)
        else:
            assert not (oracle.success and np.min(a@oracle.x+b) >= -1e-9
                        and np.linalg.norm(oracle.x) <= .15+1e-9)


def test_recorded_capture_preserves_suggestions_and_infeasibility():
    path = Path(__file__).resolve().parents[1] / 'evidence/foot-diagnostic/20261001-212903-foot-timing/telemetry.ndjson'
    seen = set()
    counts = {}
    for line in path.read_text().splitlines():
        d = json.loads(line)['diagnostic']
        key = (d['session_id'], d['seq'])
        if key in seen or d['shadow']['status'] == 'stale':
            continue
        seen.add(key)
        actual = shadow_cbf(d['feet'])
        expected = d['shadow']
        assert actual['status'] == expected['status']
        counts[actual['status']] = counts.get(actual['status'], 0)+1
        if actual['velocity'] is not None:
            assert np.allclose(list(actual['velocity'].values()),
                               list(expected['velocity'].values()), atol=1e-5)
    assert counts['infeasible'] == 18
    assert counts['suggestion'] == 65
