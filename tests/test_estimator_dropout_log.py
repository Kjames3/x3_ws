import logging
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from estimator_dropout_log import DropoutLog


def run(frames, **kw):
    msgs = []

    class L:
        def info(self, m, *a): msgs.append(('info', m % a))
        def warning(self, m, *a): msgs.append(('warning', m % a))
    d = DropoutLog(log=L(), **kw)
    for f in frames:
        d.frame(*f[:2], **(f[2] if len(f) > 2 else {}))
    return d, msgs


def test_short_empty_run_is_not_reported():
    d, m = run([(0.0, 1, dict(nearest=1.0, ids=[3])), (0.1, 0, dict(reasons={'gated_visible': 1})),
                (0.2, 1, dict(nearest=1.0, ids=[3]))])
    assert m == [] and d.dropouts == 0


def test_dropout_reports_duration_reasons_and_ids():
    fr = [(0.0, 1, dict(nearest=0.81, ids=[10]))]
    fr += [(0.1 * k, 0, dict(reasons={'gated_visible': 1} if k < 4 else {'no_centroids': 1})) for k in range(1, 8)]
    fr += [(0.8, 1, dict(nearest=1.0, ids=[30]))]
    d, m = run(fr)
    assert d.dropouts == 1 and abs(d.longest_s - 0.7) < 1e-9
    kind, text = m[0]
    assert kind == 'info' and '0.70 s' in text and '0.81 m' in text and 'gated_visible x3' in text
    assert 'no_centroids x4' in text and '[10]' in text and '[30]' in text


def test_long_dropout_warns_once_while_still_empty():
    fr = [(0.0, 1, dict(nearest=0.5, ids=[1]))] + [(0.1 * k, 0, dict(reasons={'no_depth_in_band': 1})) for k in range(1, 30)]
    d, m = run(fr)
    assert [k for k, _ in m] == ['warning'] and 'no_depth_in_band' in m[0][1]


def test_totals_count_every_empty_frame():
    d, _ = run([(0.0, 0, dict(reasons={'a': 1})), (0.1, 0, dict(reasons={'a': 1, 'b': 1}))])
    assert d.totals['a'] == 2 and d.totals['b'] == 1
