"""Tests for back-and-forth MarkerPatrol controller."""

import math
import time

from marker_patrol import (
    MODE_LOOP,
    MODE_PING_PONG,
    MarkerPatrol,
    PATROL_ACTIVE,
    PATROL_FAILED,
    PATROL_IDLE,
    PATROL_RETURNING,
    STATE_FAILED,
    STATE_IDLE,
    STATE_SUCCEEDED,
)

STATE_EXECUTING = "EXECUTING"


class _FakeNav2:
    def __init__(self):
        self.calls = []
        self.cancels = 0
        self._next_id = 0
        self._states = {}

    def navigate_to(self, x, y, theta=0.0):
        self._next_id += 1
        gid = self._next_id
        self.calls.append((gid, x, y, theta))
        self._states[gid] = {"state": STATE_EXECUTING}
        return gid

    def cancel(self, goal_id=None):
        self.cancels += 1
        for gid in list(self._states):
            if goal_id is None or gid == goal_id:
                self._states[gid] = {"state": STATE_IDLE}

    def set_goal_state(self, gid, state, fail_reason=None):
        self._states[gid] = {"state": state, "fail_reason": fail_reason}

    def get_status(self, goal_id=None):
        if goal_id is None:
            return {"state": STATE_IDLE}
        return dict(self._states.get(goal_id, {"state": STATE_IDLE}))


def _wait_until(pred, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_start_validation():
    nav = _FakeNav2()
    patrol = MarkerPatrol(nav, poll_s=0.01)

    ok, _ = patrol.start([{"x": 0.0, "y": 0.0}])
    assert not ok

    ok, _ = patrol.start([{"x": 0.0, "y": 0.0}, {"x": math.nan, "y": 1.0}])
    assert not ok


def test_ping_pong_three_markers():
    nav = _FakeNav2()
    patrol = MarkerPatrol(nav, poll_s=0.01)
    markers = [
        {"x": 0.0, "y": 0.0, "theta": 0.0},
        {"x": 1.0, "y": 0.0, "theta": 0.5},
        {"x": 2.0, "y": 0.0, "theta": 1.0},
    ]

    ok, _ = patrol.start(markers, return_to_first=False)
    assert ok
    assert nav.calls[-1][1:3] == (0.0, 0.0)

    # 0 -> 1 -> 2 -> 1 -> 0 -> 1
    expected_xy = [(1.0, 0.0), (2.0, 0.0), (1.0, 0.0), (0.0, 0.0), (1.0, 0.0)]
    for idx, xy in enumerate(expected_xy, start=1):
        nav.set_goal_state(idx, STATE_SUCCEEDED)
        assert _wait_until(lambda n=idx + 1: len(nav.calls) == n)
        assert nav.calls[-1][1:3] == xy

    patrol.abort()
    assert patrol.status()["state"] == PATROL_IDLE


def test_stop_in_place_when_return_disabled():
    nav = _FakeNav2()
    patrol = MarkerPatrol(nav, poll_s=0.01)
    markers = [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}]

    ok, _ = patrol.start(markers, return_to_first=False)
    assert ok
    nav.set_goal_state(1, STATE_SUCCEEDED)
    assert _wait_until(lambda: len(nav.calls) == 2)

    ok, msg = patrol.stop()
    assert ok
    assert "in place" in msg.lower()
    assert patrol.status()["state"] == PATROL_IDLE
    assert nav.cancels == 1


def test_stop_returns_to_first_when_enabled():
    nav = _FakeNav2()
    patrol = MarkerPatrol(nav, poll_s=0.01)
    markers = [{"x": 0.5, "y": 1.5, "theta": 0.2}, {"x": 3.0, "y": 1.5, "theta": 0.0}]

    ok, _ = patrol.start(markers, return_to_first=False)
    assert ok
    # Toggle mid-patrol
    patrol.set_return_to_first(True)
    nav.set_goal_state(1, STATE_SUCCEEDED)
    assert _wait_until(lambda: len(nav.calls) == 2)
    assert patrol.status()["active_index"] == 1

    ok, _ = patrol.stop()
    assert ok
    assert patrol.status()["state"] == PATROL_RETURNING
    assert nav.calls[-1][1:] == (0.5, 1.5, 0.2)

    # Completing return-to-first transitions to IDLE
    nav.set_goal_state(3, STATE_SUCCEEDED)
    assert _wait_until(lambda: patrol.status()["state"] == PATROL_IDLE)


def test_second_stop_during_return_cancels_immediately():
    nav = _FakeNav2()
    patrol = MarkerPatrol(nav, poll_s=0.01)
    markers = [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}]

    patrol.start(markers, return_to_first=True)
    nav.set_goal_state(1, STATE_SUCCEEDED)
    assert _wait_until(lambda: len(nav.calls) == 2)

    patrol.stop()
    assert patrol.status()["state"] == PATROL_RETURNING

    ok, _ = patrol.stop()
    assert ok
    assert patrol.status()["state"] == PATROL_IDLE
    assert nav.cancels == 1


def test_abort_always_stops_in_place():
    nav = _FakeNav2()
    patrol = MarkerPatrol(nav, poll_s=0.01)
    markers = [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}]

    patrol.start(markers, return_to_first=True)
    patrol.abort()
    assert patrol.status()["state"] == PATROL_IDLE
    assert nav.cancels == 1
    assert len(nav.calls) == 1


def test_nav_failure_sets_failed_state():
    nav = _FakeNav2()
    patrol = MarkerPatrol(nav, poll_s=0.01)
    markers = [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}]

    patrol.start(markers, return_to_first=True)
    nav.set_goal_state(1, STATE_FAILED, fail_reason="No path")
    assert _wait_until(lambda: patrol.status()["state"] == PATROL_FAILED)
    assert patrol.status()["reason"] == "No path"


def test_loop_mode_three_markers():
    nav = _FakeNav2()
    patrol = MarkerPatrol(nav, poll_s=0.01)
    markers = [
        {"x": 0.0, "y": 0.0},
        {"x": 1.0, "y": 0.0},
        {"x": 2.0, "y": 0.0},
    ]

    ok, _ = patrol.start(markers, mode=MODE_LOOP)
    assert ok
    assert nav.calls[-1][1:3] == (0.0, 0.0)

    # 0 -> 1 -> 2 -> 0 -> 1
    expected_xy = [(1.0, 0.0), (2.0, 0.0), (0.0, 0.0), (1.0, 0.0)]
    for idx, xy in enumerate(expected_xy, start=1):
        nav.set_goal_state(idx, STATE_SUCCEEDED)
        assert _wait_until(lambda n=idx + 1: len(nav.calls) == n)
        assert nav.calls[-1][1:3] == xy

    patrol.abort()


def test_invalid_mode_rejected():
    nav = _FakeNav2()
    patrol = MarkerPatrol(nav, poll_s=0.01)
    ok, _ = patrol.start([{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}], mode="zigzag")
    assert not ok
    assert not patrol.set_mode("zigzag")
    assert patrol.set_mode(MODE_PING_PONG)
