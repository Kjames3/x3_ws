import asyncio
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

import frontier_explorer as frontier


def snapshot(grid, yaw=0.0, sequence=1):
    height, width = grid.shape
    return {
        "data": grid.ravel(), "height": height, "width": width,
        "frame_id": "map", "sequence": sequence, "resolution": 0.1,
        "received_monotonic": time.monotonic(),
        "origin_x": 2.0, "origin_y": -3.0, "origin_yaw": yaw,
    }


def split_map(yaw=0.0):
    grid = np.full((55, 80), 100, dtype=np.int8)
    grid[5:50, 5:40] = 0
    grid[5:50, 41:75] = 0
    grid[40:50, 5:40] = -1
    grid[40:50, 41:75] = -1
    return snapshot(grid, yaw)


class Bridge:
    def __init__(self):
        self.map = split_map()
        self.pose = (4.55, -0.45, 0.0)

    def get_occupancy_grid(self):
        return self.map

    def get_exploration_pose(self, frame_id):
        assert frame_id == "map"
        return self.pose


class Nav:
    def __init__(self, statuses):
        self.statuses = iter(statuses)
        self.sent = []
        self.canceled = []

    def navigate_to(self, x, y, heading):
        self.sent.append((x, y, heading))
        return len(self.sent)

    def get_status(self, goal_id):
        assert goal_id == len(self.sent)
        return next(self.statuses)

    def is_busy(self):
        return False

    def cancel(self, goal_id):
        self.canceled.append(goal_id)


def test_rotated_map_filters_disconnected_room_and_respects_standoff():
    world = frontier._FrontierGrid(split_map())
    rotated = frontier._FrontierGrid(split_map(math.pi / 2))
    start = world._cell_to_world(25, 25)
    turned_start = rotated._cell_to_world(25, 25)
    goals = world.goals((*start, 0.0))
    turned = rotated.goals((*turned_start, 0.0))

    assert world.count == 2
    assert len(goals) == len(turned) == 1
    goal, turned_goal = goals[0], turned[0]
    row, col = world.world_to_cell(goal.x, goal.y)
    assert col < 40
    assert (40 - row - 0.5) * 0.1 >= frontier.GOAL_CLEARANCE_M
    assert goal.gain > 0.0
    assert goal.path_cost == pytest.approx((abs(row - 25) + abs(col - 25)) * 0.1)
    assert rotated.world_to_cell(turned_goal.x, turned_goal.y) == (row, col)
    assert (turned_goal.heading - goal.heading) % (2 * math.pi) == pytest.approx(math.pi / 2)


def test_goal_requires_translation_when_near_frontier():
    world = frontier._FrontierGrid(split_map())
    start = world._cell_to_world(34, 25)
    goal, = world.goals((*start, 0.0))
    assert math.dist(start, (goal.x, goal.y)) >= frontier.MIN_GOAL_TRAVEL_M


def test_goal_on_other_side_of_wall_is_not_visible():
    grid = np.full((48, 62), 100, dtype=np.int8)
    grid[5:43, 5:35] = 0
    grid[5:43, 36:41] = 0
    grid[5:43, 41:57] = -1
    geometry = frontier._FrontierGrid(snapshot(grid))
    assert geometry.count == 1
    assert geometry.goals((*geometry._cell_to_world(25, 25), 0.0)) == []


def test_selection_reuses_preprocessing_per_revision_and_runs_off_loop(monkeypatch):
    bridge = Bridge()
    explorer = frontier.FrontierExplorer(Nav([]), bridge)
    builds = []
    original = frontier._FrontierGrid

    def build(grid):
        builds.append((grid["sequence"], threading.get_ident()))
        return original(grid)

    monkeypatch.setattr(frontier, "_FrontierGrid", build)

    async def run():
        loop_thread = threading.get_ident()
        result = await explorer._select_goals(bridge.map, bridge.pose)
        assert result is await explorer._select_goals(bridge.map, bridge.pose)
        await explorer._select_goals(dict(bridge.map, sequence=2), bridge.pose)
        assert [sequence for sequence, _ in builds] == [1, 2]
        assert all(thread != loop_thread for _, thread in builds)

    asyncio.run(run())


def test_nav_abort_retries_and_does_not_count_as_visited(monkeypatch):
    bridge = Bridge()
    nav = Nav([{"outcome": "ABORTED"}] * 2)
    explorer = frontier.FrontierExplorer(nav, bridge)
    monkeypatch.setattr(frontier, "MAX_ATTEMPTS", 2)
    monkeypatch.setattr(frontier, "RETRY_COOLDOWN_S", 0.0)

    async def run():
        assert explorer.start()
        await asyncio.wait_for(explorer._task, 5.0)

    asyncio.run(run())
    status = explorer.status()
    assert len(nav.sent) == 2
    assert status["frontiers_visited"] == 0
    assert status["frontier_state"] == frontier.STATE_BLOCKED
    assert "navigation retries" in status["frontier_reason"]


def test_success_counts_reached_goal_and_leaves_disconnected_frontier_unresolved():
    bridge = Bridge()
    nav = Nav([{"outcome": "SUCCEEDED"}])
    explorer = frontier.FrontierExplorer(nav, bridge)

    async def run():
        assert explorer.start()
        await asyncio.wait_for(explorer._task, 2.0)

    asyncio.run(run())
    status = explorer.status()
    assert len(nav.sent) == 1
    assert status["frontiers_visited"] == 1
    assert status["frontier_state"] == frontier.STATE_BLOCKED
    assert "reachable approaches" in status["frontier_reason"]


def test_progress_allows_long_goal_but_stall_cancels(monkeypatch):
    bridge = Bridge()
    nav = Nav([{"state": "EXECUTING", "dist": distance} for distance in [8.0, 7.8, 7.6, 7.5, 7.5, 7.5]])
    explorer = frontier.FrontierExplorer(nav, bridge)
    explorer._goal_id = nav.navigate_to(0, 0, 0)
    bridge.map["received_monotonic"] = 0.0
    monkeypatch.setattr(frontier, "MAX_MAP_AGE_S", 100.0)
    times = iter([0.0, 1.0, 18.0, 35.0, 52.0, 70.0, 90.0])
    monkeypatch.setattr(frontier, "time", SimpleNamespace(monotonic=lambda: next(times)))
    monkeypatch.setattr(frontier, "POLL_S", 0.0)

    assert asyncio.run(explorer._wait_for_nav2(explorer._goal_id)) == "TIMEOUT"
    assert nav.canceled == [1]
    assert next(times, None) is None


def test_losing_map_pose_cancels_owned_goal():
    bridge = Bridge()
    bridge.pose = None
    nav = Nav([{"state": "EXECUTING", "dist": 2.0}])
    explorer = frontier.FrontierExplorer(nav, bridge)
    explorer._goal_id = nav.navigate_to(0, 0, 0)

    assert asyncio.run(explorer._wait_for_nav2(explorer._goal_id)) == "LOCALIZATION_LOST"
    assert nav.canceled == [1]


def test_stale_map_pauses_selection_despite_available_pose():
    bridge = Bridge()
    bridge.map["received_monotonic"] -= frontier.MAX_MAP_AGE_S + 1
    nav = Nav([])
    explorer = frontier.FrontierExplorer(nav, bridge)

    async def run():
        assert explorer.start()
        await asyncio.sleep(0)
        assert explorer.status()["frontier_state"] == frontier.STATE_SELECTING
        assert "fresh map-frame occupancy grid" in explorer.status()["frontier_reason"]
        explorer.stop()
        await explorer._task

    asyncio.run(run())
    assert nav.sent == []


def test_stale_map_cancels_owned_goal_even_with_fresh_pose():
    bridge = Bridge()
    bridge.map["received_monotonic"] -= frontier.MAX_MAP_AGE_S + 1
    nav = Nav([{"state": "EXECUTING", "dist": 2.0}])
    explorer = frontier.FrontierExplorer(nav, bridge)
    explorer._goal_id = nav.navigate_to(0, 0, 0)

    assert asyncio.run(explorer._wait_for_nav2(explorer._goal_id)) == "MAP_STALE"
    assert nav.canceled == [1]


def test_stop_status_waits_for_owned_action_terminal_result():
    nav = Nav([{"state": "CANCELLING"}, {"state": "IDLE"}])
    explorer = frontier.FrontierExplorer(nav, Bridge())
    explorer._goal_id = nav.navigate_to(1, 0, 0)
    explorer.stop()
    assert explorer.status()["frontier_state"] == "STOPPING"
    assert explorer.status()["frontier_state"] == "STOPPED"


def test_stuck_backs_out_opposite_command_and_marks_keepout_ahead(monkeypatch):
    """Commanded forward, map pose frozen -> STUCK, reverse drive, keep-out ahead."""
    monkeypatch.setattr(frontier, "STUCK_WINDOW_S", 0.6)
    monkeypatch.setattr(frontier, "POLL_S", 0.05)
    monkeypatch.setattr(frontier, "BACKOUT_S", 0.2)
    monkeypatch.setattr(frontier, "MAX_ATTEMPTS", 1)
    monkeypatch.setattr(frontier, "RETRY_COOLDOWN_S", 0.0)
    bridge = Bridge()
    nav = Nav(iter(lambda: {"state": "NAVIGATING", "dist": 2.0}, None))
    explorer = frontier.FrontierExplorer(nav, bridge)
    drives, keepouts = [], []
    explorer.drive_fn = lambda vx, vy, wz: drives.append((vx, vy, wz))
    explorer.get_nav_cmd = lambda: (0.2, 0.0, 0.0, 0.05)
    explorer.add_keepout = lambda x, y, r: keepouts.append((x, y, r))

    async def run():
        assert explorer.start()
        await asyncio.wait_for(explorer._task, 5.0)

    asyncio.run(run())
    assert explorer.stuck_count == 1
    assert nav.canceled == [1]
    assert drives and drives[0][0] < 0 and drives[0][1] == 0 and drives[-1] == (0.0, 0.0, 0.0)
    (kx, ky, r), = keepouts
    assert kx == pytest.approx(bridge.pose[0] + frontier.KEEPOUT_AHEAD_M)
    assert ky == pytest.approx(bridge.pose[1])
    assert explorer.status()["frontier_state"] == frontier.STATE_BLOCKED


def test_not_stuck_while_moving_or_without_command():
    explorer = frontier.FrontierExplorer(Nav([]), Bridge())
    explorer.get_nav_cmd = lambda: (0.0, 0.0, 0.0, 0.05)      # Nav2 holding still
    hist = []
    for i in range(60):
        assert not explorer._is_stuck(hist, i * 0.1, (0.0, 0.0, 0.0))
    explorer.get_nav_cmd = lambda: (0.2, 0.0, 0.0, 0.05)      # commanded and moving
    hist = []
    for i in range(60):
        assert not explorer._is_stuck(hist, i * 0.1, (i * 0.02, 0.0, 0.0))
