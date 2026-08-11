"""
Reproduce the GUI's phantom 'FAILED' badge and prove the goal-identity guard fixes it.

Scenario taken verbatim from the jetson journal (2026-08-10 18:02:38-18:02:54):
  goal A sent + accepted, goal B sent + accepted, then A's ABORTED result lands,
  then B's SUCCEEDED result lands.
Before the fix the badge read FAILED between those last two events.
"""
import sys, os, types, threading
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import nav2_client
from nav2_client import Nav2Client, STATE_EXECUTING, STATE_FAILED, STATE_SUCCEEDED
from action_msgs.msg import GoalStatus


class FakeFuture:
    def __init__(self, result): self._r = result
    def result(self): return self._r
    def add_done_callback(self, cb): self._cb = cb
    def fire(self): self._cb(self)


class FakeHandle:
    def __init__(self, name):
        self.accepted = True; self.name = name
        self.result_future = None; self.cancelled = False
    def get_result_async(self):
        self.result_future = FakeFuture(None)
        return self.result_future
    def cancel_goal_async(self):
        self.cancelled = True
        return FakeFuture(None)


class FakeActionClient:
    def __init__(self, *a, **k): self.sent = []
    def wait_for_server(self, timeout_sec=0.0): return True
    def send_goal_async(self, goal, feedback_callback=None):
        fut = FakeFuture(FakeHandle(f"goal{len(self.sent)}"))
        self.sent.append(fut)
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
    c.navigate_to(-2.41, 0.87)
    send_a = ac.sent[0]
    send_a.fire()                       # A accepted
    assert c.get_status()["state"] == STATE_EXECUTING, "A should be executing"
    handle_a = send_a.result()

    # --- goal B preempts A ---
    c.navigate_to(0.51, -0.31)
    send_b = ac.sent[1]
    send_b.fire()                       # B accepted
    handle_b = send_b.result()
    assert c.get_status()["state"] == STATE_EXECUTING, "B should be executing"

    # --- A's preemption result arrives LATE, as ABORTED (status 6) ---
    handle_a.result_future._r = make_result(GoalStatus.STATUS_ABORTED)
    handle_a.result_future.fire()
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
    c3.navigate_to(2.0, 2.0)
    s3 = c3._action_client.sent[0]
    c3.cancel()                        # pressed while acceptance is still pending
    s3.fire()                          # Nav2 accepts only now
    assert s3.result().cancelled, "orphaned late-accepted goal must be cancelled"
    assert c3._goal_handle is None, "an orphaned goal must not become the live goal"
    print("  ok: goal accepted after Cancel is actively cancelled, not abandoned")

    # --- set_initial_pose must stop an in-flight goal before relocalising ---
    c4 = Nav2Client(FakeNode())
    c4.navigate_to(3.0, 3.0)
    s4 = c4._action_client.sent[0]
    s4.fire()
    assert c4._goal_handle is not None
    c4.set_initial_pose(0.0, 0.0, 1.57)
    assert s4.result().cancelled, "set_initial_pose must cancel the active goal"
    print("  ok: set_initial_pose cancels the active goal first")


if __name__ == "__main__":
    run()
    print("\nALL PASS")
