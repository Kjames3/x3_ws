"""Say why the velocity estimator stopped reporting a person.

The estimator publishes nothing on a frame with no estimates, so a gap in
/pedestrian_states is the only trace of a dropout and it carries no cause.
This keeps the per-frame reasons while the estimator is empty and logs one
line when a person comes back (or a warning if it has been empty for 1 s).
"""
import logging
from collections import Counter

logger = logging.getLogger(__name__)


class DropoutLog:
    def __init__(self, min_report_s=0.3, warn_after_s=1.0, log=None):
        self.min_report_s = min_report_s
        self.warn_after_s = warn_after_s
        self.log = log or logger
        self._since = None
        self._reasons = Counter()
        self._warned = False
        self._seen = (None, ())      # (range m, ids) of the last frame with a person
        self.totals = Counter()      # reason -> frames, for the whole run
        self.dropouts = 0
        self.longest_s = 0.0

    def frame(self, t, n_person, reasons=None, nearest=None, ids=()):
        """One estimator frame. ``reasons``: why there was no person (dict/Counter)."""
        if n_person > 0:
            if self._since is not None:
                dur = t - self._since
                if dur >= self.min_report_s:
                    self.dropouts += 1
                    self.longest_s = max(self.longest_s, dur)
                    self.log.info(
                        "person dropout %.2f s (last seen %s, ids %s): %s; back at %s, ids %s",
                        dur, _m(self._seen[0]), list(self._seen[1]), _fmt(self._reasons),
                        _m(nearest), list(ids))
                self._since, self._warned = None, False
                self._reasons = Counter()
            self._seen = (nearest, tuple(ids))
            return
        for k, n in (reasons or {'unknown': 1}).items():
            self.totals[k] += n
        if self._since is None:
            self._since = t
        self._reasons.update(reasons or {'unknown': 1})
        if not self._warned and t - self._since >= self.warn_after_s:
            self._warned = True
            self.log.warning("no person estimate for %.1f s so far (last seen %s, ids %s): %s",
                             t - self._since, _m(self._seen[0]), list(self._seen[1]),
                             _fmt(self._reasons))


def _m(v):
    return 'n/a' if v is None else '%.2f m' % v


def _fmt(c):
    return ', '.join('%s x%d' % kv for kv in c.most_common()) or 'none'
