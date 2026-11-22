"""
Reproduce the GUI's phantom 'FAILED' badge and prove the goal-identity guard fixes it.

Scenario taken verbatim from the jetson journal (2026-08-10 18:02:38-18:02:54):
  goal A sent + accepted, goal B sent + accepted, then A's ABORTED result lands,
  then B's SUCCEEDED result lands.
Before the fix the badge read FAILED between those last two events.
"""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import types

import nav2_client
from nav2_client import Nav2Client, STATE_CANCELLING, STATE_EXECUTING, STATE_FAILED, STATE_SUCCEEDED
from action_msgs.msg import GoalStatus


class FakeFuture:
    def __init__(self, result): self._r = result
    def result(self):
        if isinstance(self._r, Exception):
            raise self._r
        return self._r
    def add_done_callback(self, cb): self._cb = cb
    def fire(self): self._cb(self)


class FakeHandle:
    def __init__(self, name):
        self.accepted = True; self.goal_id = name
        self.result_future = None; self.cancelled = False
    def get_result_async(self):
        self.result_future = FakeFuture(None)
        return self.result_future
    def cancel_goal_async(self):
        self.cancelled = True
        self.cancel_future = FakeFuture(None)
        return self.cancel_future


class FakeActionClient:
    def __init__(self, *a, **k):
        self.sent = []
        self.feedback = []
        self.ready = True

    def server_is_ready(self): return self.ready
    def send_goal_async(self, goal, feedback_callback=None):
        fut = FakeFuture(FakeHandle(f"goal{len(self.sent)}"))
        self.sent.append(fut)
        self.feedback.append(feedback_callback)
        return fut


class FakeNode:
    def create_publisher(self, *a, **k): return types.SimpleNamespace(publish=lambda m: None)
    def create_subscription(self, *a, **k): return None
    def get_clock(self):
        from builtin_interfaces.msg import Time
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: Time()))


def make_result(status):
    return types.SimpleNamespace(status=status)


def run():
    nav2_client.ActionClient = FakeActionClient
    c = Nav2Client(FakeNode())
    ac = c._action_client

    # --- goal A ---
    a = c.navigate_to(-2.41, 0.87)
    send_a = ac.sent[0]
    send_a.fire()                       # A accepted
    assert c.get_status()["state"] == STATE_EXECUTING, "A should be executing"
    handle_a = send_a.result()

    # --- goal B preempts A ---
    b = c.navigate_to(0.51, -0.31)
    send_b = ac.sent[1]
    send_b.fire()                       # B accepted
    handle_b = send_b.result()
    assert c.get_status()["state"] == STATE_EXECUTING, "B should be executing"

    assert a != b
    assert c.is_cancelling()
    feedback = types.SimpleNamespace(
        distance_remaining=3.0,
        navigation_time=types.SimpleNamespace(sec=0, nanosec=0),
        estimated_time_remaining=types.SimpleNamespace(sec=0, nanosec=0),
        number_of_recoveries=0,
    )
    ac.feedback[0](types.SimpleNamespace(feedback=feedback))
    assert c.get_status(b)["dist"] is None, "superseded feedback must be ignored"
    ac.feedback[1](types.SimpleNamespace(feedback=feedback))
    assert c.get_status(b)["dist"] == 3.0

    # --- A's preemption result arrives LATE, as ABORTED (status 6) ---
    handle_a.result_future._r = make_result(GoalStatus.STATUS_ABORTED)
    handle_a.result_future.fire()
    handle_a.cancel_future._r = types.SimpleNamespace(goals_canceling=[], return_code=3)
    handle_a.cancel_future.fire()
    assert not c.is_cancelling()
    assert c.get_status(a)["outcome"] == "ABORTED"
    state = c.get_status()["state"]
    assert state != STATE_FAILED, (
        f"REGRESSION: stale ABORTED from preempted goal set badge to {state}")
    assert state == STATE_EXECUTING, f"expected still EXECUTING, got {state}"
    print("  ok: stale ABORTED from the preempted goal was ignored")

    # --- B genuinely succeeds ---
    handle_b.result_future._r = make_result(GoalStatus.STATUS_SUCCEEDED)
    handle_b.result_future.fire()
    assert c.get_status()["state"] == STATE_SUCCEEDED, "B's success must be reported"
    print("  ok: the live goal's SUCCEEDED still reported")

    # --- a genuine abort with no preemption must STILL surface as FAILED ---
    c2 = Nav2Client(FakeNode())
    c2.navigate_to(1.0, 1.0)
    s = c2._action_client.sent[0]
    s.fire()
    h = s.result()
    h.result_future._r = make_result(GoalStatus.STATUS_ABORTED)
    h.result_future.fire()
    assert c2.get_status()["state"] == STATE_FAILED, "real aborts must still report FAILED"
    print("  ok: a genuine abort still reports FAILED")

    # --- Cancel pressed before Nav2 accepts: the late-accepted goal would
    #     otherwise drive the robot with nobody holding its handle. ---
    c3 = Nav2Client(FakeNode())
    late = c3.navigate_to(2.0, 2.0)
    s3 = c3._action_client.sent[0]
    c3.cancel()                        # pressed while acceptance is still pending
    s3.fire()                          # Nav2 accepts only now
    assert s3.result().cancelled, "orphaned late-accepted goal must be cancelled"
    assert c3.get_status(late)["state"] == STATE_CANCELLING
    h3 = s3.result()
    h3.cancel_future._r = types.SimpleNamespace(
        goals_canceling=[types.SimpleNamespace(goal_id=h3.goal_id)], return_code=0)
    h3.cancel_future.fire()
    assert c3.is_busy(), "a cancellation acknowledgment does not confirm termination"
    assert c3.navigate_to(2.1, 2.0) is None
    h3.result_future._r = make_result(GoalStatus.STATUS_CANCELED)
    h3.result_future.fire()
    assert not c3.is_busy()
    assert c3.get_status(late)["outcome"] == "CANCELED"
    print("  ok: goal accepted after Cancel is actively cancelled, not abandoned")

    # --- set_initial_pose must stop an in-flight goal before relocalising ---
    c4 = Nav2Client(FakeNode())
    c4.navigate_to(3.0, 3.0)
    s4 = c4._action_client.sent[0]
    s4.fire()
    assert c4.is_driving()
    c4.set_initial_pose(0.0, 0.0, 1.57)
    assert s4.result().cancelled, "set_initial_pose must cancel the active goal"
    print("  ok: set_initial_pose cancels the active goal first")

    # --- Cancel before acceptance, followed by an explicit rejection ---
    c5 = Nav2Client(FakeNode())
    goal = c5.navigate_to(3.0, 4.0)
    assert goal is not None
    c5.cancel(goal)
    s5 = c5._action_client.sent[0]
    s5.result().accepted = False
    s5.fire()
    assert c5.get_status(goal)["outcome"] == "REJECTED"
    assert not c5.is_busy()

    # --- An action server that is not ready refuses submission immediately ---
    c5._action_client.ready = False
    assert c5.navigate_to(4.0, 5.0) is None
    assert len(c5._action_client.sent) == 1
    assert c5.get_status()["state"] == "UNAVAILABLE"

    # --- A rejected cancellation must keep the goal owned until terminal ---
    c6 = Nav2Client(FakeNode())
    goal = c6.navigate_to(2.0, 1.0)
    s6 = c6._action_client.sent[0]
    s6.fire()
    h6 = s6.result()
    c6.cancel(goal)
    rejected_cancel = h6.cancel_future
    rejected_cancel._r = types.SimpleNamespace(goals_canceling=[], return_code=1)
    rejected_cancel.fire()
    assert c6.is_busy()
    assert "did not accept" in c6.get_status(goal)["fail_reason"]
    c6.cancel(goal)
    assert h6.cancel_future is not rejected_cancel
    h6.result_future._r = make_result(GoalStatus.STATUS_CANCELED)
    h6.result_future.fire()
    assert not c6.is_busy()

    # --- A superseded pending request may be accepted after the new goal ---
    c7 = Nav2Client(FakeNode())
    older = c7.navigate_to(1.0, 2.0)
    newer = c7.navigate_to(3.0, 4.0)
    old_request, new_request = c7._action_client.sent
    new_request.fire()
    old_request.fire()
    assert old_request.result().cancelled
    assert c7.get_status(older)["state"] == STATE_CANCELLING
    assert c7.get_status(newer)["state"] == STATE_EXECUTING
    assert c7.get_status()["state"] == STATE_EXECUTING
    assert c7.is_cancelling()

    # --- A result transport failure leaves motion state uncertain ---
    c8 = Nav2Client(FakeNode())
    goal = c8.navigate_to(3.0, 4.0)
    s8 = c8._action_client.sent[0]
    s8.fire()
    h8 = s8.result()
    h8.result_future._r = RuntimeError("transport failed")
    h8.result_future.fire()
    assert h8.cancelled
    assert c8.get_status(goal)["outcome"] == "ERROR"
    assert c8.is_busy()
    assert c8.is_cancelling()


if __name__ == "__main__":
    run()
    print("\nALL PASS")
