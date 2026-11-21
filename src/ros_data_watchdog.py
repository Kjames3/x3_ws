"""Detect the ROS data-plane stall: topics advertised, nothing delivered.

On ~1 in 3 startups (and once on 2026-11-21 after a plain x3_server restart)
every subscription connects but no message ever arrives: /livox/lidar, /scan,
/odom and /surroundings/points all read 0 Hz while the Livox answers ping and
its driver runs. The GUI just shows "3D unavailable/stale" and nothing reacts.
The manual fix is `systemctl restart x3_server`, then foxglove_bridge.

This module is only the detector. ROS callbacks call beat(name); a 1 Hz task
calls check() and acts on transitions. A topic is STALLED when it has not
arrived for `stall_after` s, or never arrived `startup_grace` s after start.
Recovery (restarting the bringup and the bridge) is a later step and is not
done here.
"""
from __future__ import annotations

import threading
import time


class DataWatchdog:
    def __init__(self, topics, stall_after=5.0, startup_grace=30.0, clock=time.monotonic):
        self._clock = clock
        self._t0 = clock()
        self._last = {t: None for t in topics}
        self._count = {t: 0 for t in topics}
        self._lock = threading.Lock()
        self.stall_after = stall_after
        self.startup_grace = startup_grace
        self._stalled_since = None

    def beat(self, topic):
        """Called from ROS callbacks (any thread)."""
        now = self._clock()
        with self._lock:
            self._last[topic] = now
            self._count[topic] = self._count.get(topic, 0) + 1

    def check(self):
        """Return {'ok', 'stalled', 'ages', 'stalled_for', 'counts'}; ok is None during startup grace."""
        now = self._clock()
        with self._lock:
            last = dict(self._last)
            counts = dict(self._count)
        ages, stalled = {}, []
        in_grace = now - self._t0 < self.startup_grace
        for t, at in last.items():
            ages[t] = None if at is None else round(now - at, 1)
            if at is None:
                if not in_grace:
                    stalled.append(t)
            elif now - at > self.stall_after:
                stalled.append(t)
        if stalled:
            if self._stalled_since is None:
                self._stalled_since = now
        else:
            self._stalled_since = None
        ok = None if (in_grace and not stalled and any(a is None for a in last.values())) else not stalled
        return {'ok': ok, 'stalled': stalled, 'ages': ages, 'counts': counts,
                'stalled_for': 0.0 if self._stalled_since is None else round(now - self._stalled_since, 1)}
