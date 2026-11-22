"""Explore SLAM frontiers using reachable, clearance-filtered Nav2 goals."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import logging
import math
import threading
import time

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

logger = logging.getLogger(__name__)

STATE_IDLE = "IDLE"
STATE_EXPLORING = "EXPLORING"
STATE_SELECTING = "SELECTING"
STATE_COMPLETE = "COMPLETE"
STATE_STOPPED = "STOPPED"
STATE_BLOCKED = "BLOCKED"

MIN_CLUSTER_SIZE = 10
# Current Nav2 footprint half extents are 0.32 m by 0.293 m. Use near-inscribed
# clearance for connectivity and circumscribed clearance for the turning goal.
PATH_CLEARANCE_M = 0.30
GOAL_CLEARANCE_M = 0.45
MAX_STANDOFF_M = 0.90
# Leave room for Nav2's 0.25 m position tolerance and within-cell pose quantization.
MIN_GOAL_TRAVEL_M = 0.35
GAIN_RADIUS_M = 1.0
FRONTIER_MATCH_M = 0.30
ACCEPT_TIMEOUT_S = 10.0
NO_PROGRESS_TIMEOUT_S = 30.0
PROGRESS_STEP_M = 0.10
POLL_S = 0.2
MAX_MAP_AGE_S = 5.0  # SLAM Toolbox is configured to publish every 0.5 s.
MAX_ATTEMPTS = 3
RETRY_COOLDOWN_S = 15.0
# Stuck = Nav2 keeps commanding motion but the scan-matched map pose does not move
# (wheels caught on a cord/base, or spinning on it -- wheel odometry would still
# "move", so the map pose is used).  Recovery backs out along the reverse of the
# commanded direction through the CBF, then marks a keep-out disc ahead of the
# stuck spot so the replanned route goes around it.
STUCK_WINDOW_S = 4.0
STUCK_MOVE_M = 0.04
STUCK_TURN_RAD = 0.15
STUCK_MIN_CMD_V = 0.05
STUCK_MIN_CMD_W = 0.25
BACKOUT_SPEED = 0.12
BACKOUT_S = 2.5          # ~0.3 m
KEEPOUT_AHEAD_M = 0.42   # footprint front 0.32 + 0.10
KEEPOUT_RADIUS_M = 0.15


@dataclass(frozen=True)
class FrontierGoal:
    x: float
    y: float
    heading: float
    anchor: tuple[float, float]
    gain: float
    path_cost: float

    @property
    def score(self):
        return self.path_cost / math.sqrt(self.gain)


class _FrontierGrid:
    """Map-revision preprocessing and path distances from the latest start cell."""

    def __init__(self, snapshot):
        self._resolution = snapshot["resolution"]
        self._origin = snapshot["origin_x"], snapshot["origin_y"]
        yaw = snapshot["origin_yaw"]
        self._cos, self._sin = math.cos(yaw), math.sin(yaw)
        grid = np.asarray(snapshot["data"], dtype=np.int8).reshape(
            snapshot["height"], snapshot["width"]
        )
        free, unknown = grid == 0, grid == -1
        cross = ndimage.generate_binary_structure(2, 1)
        labels, count = ndimage.label(free & ndimage.binary_dilation(unknown, cross))
        minimum = np.bincount(labels.ravel(), minlength=count + 1) >= MIN_CLUSTER_SIZE
        minimum[0] = False
        frontiers = minimum[labels]
        self.count = int(minimum.sum())
        self._start = None
        self._goals = []
        if not self.count:
            return

        self._centers = np.zeros((count + 1, 2))
        eligible = np.flatnonzero(minimum)
        self._centers[eligible] = ndimage.center_of_mass(frontiers, labels, eligible)

        # Pad with occupied cells so the distance transform includes map edges.
        # Subtract half a cell diagonal to account for the extent of blocked cells.
        clearance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]
        clearance = (clearance - math.sqrt(0.5)) * self._resolution
        passable = free & (clearance >= PATH_CLEARANCE_M)

        distance, nearest = ndimage.distance_transform_edt(~frontiers, return_indices=True)
        distance *= self._resolution
        owners = labels[tuple(nearest)]
        gain = unknown & (distance <= GAIN_RADIUS_M)
        self._gain = np.bincount(owners[gain], minlength=count + 1) * self._resolution**2

        candidates = (passable & (clearance >= GOAL_CLEARANCE_M)
                      & (distance <= MAX_STANDOFF_M))
        rows, cols = np.where(candidates)
        anchor_rows, anchor_cols = nearest[:, candidates]
        # Avoid associating an approach with a frontier on the far side of a wall.
        # Check the grid cells crossed on the line from the approach to its edge.
        visible = np.ones(rows.size, dtype=bool)
        steps = max(1, math.ceil(2 * MAX_STANDOFF_M / self._resolution))
        for fraction in np.linspace(0.0, 1.0, steps + 1)[1:-1]:
            line_rows = np.rint(rows + fraction * (anchor_rows - rows)).astype(int)
            line_cols = np.rint(cols + fraction * (anchor_cols - cols)).astype(int)
            visible &= free[line_rows, line_cols]
        self._candidate_rows, self._candidate_cols = rows[visible], cols[visible]
        self._candidate_owner = owners[candidates][visible]
        self._anchor_rows, self._anchor_cols = anchor_rows[visible], anchor_cols[visible]
        self._standoff = distance[candidates][visible]

        # Cardinal links between adjacent cells prevent diagonal corner cutting.
        # Construct a compact sparse graph over only the passable part of the map.
        self._ids = np.full(grid.shape, -1, dtype=np.int32)
        size = int(passable.sum())
        self._ids[passable] = np.arange(size, dtype=np.int32)
        horizontal = passable[:, :-1] & passable[:, 1:]
        vertical = passable[:-1, :] & passable[1:, :]
        rows = np.r_[self._ids[:, :-1][horizontal], self._ids[:-1, :][vertical]]
        cols = np.r_[self._ids[:, 1:][horizontal], self._ids[1:, :][vertical]]
        weights = np.full(rows.size, self._resolution)
        self._graph = coo_matrix((weights, (rows, cols)), shape=(size, size)).tocsr()

    def world_to_cell(self, x, y):
        dx, dy = x - self._origin[0], y - self._origin[1]
        col = math.floor((self._cos * dx + self._sin * dy) / self._resolution)
        row = math.floor((-self._sin * dx + self._cos * dy) / self._resolution)
        return row, col

    def _cell_to_world(self, row, col):
        x, y = (col + 0.5) * self._resolution, (row + 0.5) * self._resolution
        return (self._origin[0] + self._cos * x - self._sin * y,
                self._origin[1] + self._sin * x + self._cos * y)

    def goals(self, pose):
        """Return ranked approaches, or None when the start cell is unusable."""
        if not self.count:
            return []
        start = self.world_to_cell(*pose[:2])
        if start == self._start:
            return self._goals
        self._start = start
        row, col = start
        if not (0 <= row < self._ids.shape[0] and 0 <= col < self._ids.shape[1]):
            self._goals = None
            return None
        start_id = self._ids[row, col]
        if start_id < 0:
            self._goals = None
            return None

        distances = dijkstra(self._graph, directed=False, indices=start_id)
        path_cost = distances[self._ids[self._candidate_rows, self._candidate_cols]]
        travel = self._resolution * np.hypot(self._candidate_rows - row, self._candidate_cols - col)
        reachable = np.flatnonzero(
            np.isfinite(path_cost) & (self._gain[self._candidate_owner] > 0)
            & (travel >= MIN_GOAL_TRAVEL_M)
        )
        # Pick a reachable stand-off point for each frontier by path cost plus
        # residual distance to the boundary, then rank frontiers by gain and cost.
        standoff = self._standoff[reachable]
        ranked = reachable[np.lexsort((standoff, path_cost[reachable] + standoff))]
        _, first = np.unique(self._candidate_owner[ranked], return_index=True)
        goals = []
        for i in ranked[first]:
            x, y = self._cell_to_world(self._candidate_rows[i], self._candidate_cols[i])
            edge_x, edge_y = self._cell_to_world(self._anchor_rows[i], self._anchor_cols[i])
            heading = math.atan2(edge_y - y, edge_x - x)
            owner = self._candidate_owner[i]
            center = self._cell_to_world(*self._centers[owner])
            cost = float(path_cost[i])
            goals.append(FrontierGoal(x, y, heading, center, float(self._gain[owner]), cost))
        self._goals = sorted(goals, key=lambda goal: goal.score)
        return self._goals


class FrontierExplorer:
    """Async exploration supervisor with goal ownership and transient retries."""

    def __init__(self, nav2_client, bridge):
        self._nav2 = nav2_client
        self._bridge = bridge
        self._state = STATE_IDLE
        self._lock = threading.Lock()
        self._current_goal = None
        self._reason = None
        self._goal_id = None
        self._cancelled_goal_id = None
        self._frontiers_found = 0
        self._frontiers_visited = 0
        self._task = None
        self._worker = None
        self._grid = None
        self._grid_key = None
        # Optional hooks (server_x3 sets them): drive_fn(vx, vy, wz) feeds the
        # CBF-filtered motion queue; get_nav_cmd() -> (vx, vy, wz, age_s) of the
        # last /cmd_vel; add_keepout(x, y, radius) in the map frame.
        self.drive_fn = None
        self.get_nav_cmd = None
        self.add_keepout = None
        self.stuck_count = 0

    def start(self) -> bool:
        with self._lock:
            if self._state in (STATE_EXPLORING, STATE_SELECTING):
                return False
            self._state = STATE_SELECTING
            self._reason = None
            self._frontiers_found = 0
            self._frontiers_visited = 0

        self._task = asyncio.get_running_loop().create_task(self._explore_loop())
        logger.info("FrontierExplorer: started")
        return True

    def is_exploring(self) -> bool:
        with self._lock:
            return self._state in (STATE_EXPLORING, STATE_SELECTING)

    def stop(self) -> None:
        self._set_state(STATE_STOPPED)
        if self._task and not self._task.done():
            self._task.cancel()
        self._cancel_goal()
        logger.info("FrontierExplorer: stopped")

    def status(self) -> dict:
        with self._lock:
            return {
                "frontier_state": ("STOPPING" if self._state == STATE_STOPPED
                                   and self._cancelled_goal_id is not None
                                   and self._nav2.get_status(self._cancelled_goal_id)["state"] == "CANCELLING"
                                   else self._state),
                "frontiers_found": self._frontiers_found,
                "frontiers_visited": self._frontiers_visited,
                "frontier_reason": self._reason,
                "current_goal": list(self._current_goal) if self._current_goal else None,
                "stuck_count": self.stuck_count,
            }

    def _set_state(self, state, goal=None, reason=None):
        with self._lock:
            self._state = state
            self._reason = reason
            self._current_goal = (goal.x, goal.y) if goal else None

    def _cancel_goal(self):
        if self._goal_id is not None:
            self._cancelled_goal_id = self._goal_id
            self._nav2.cancel(self._goal_id)
            self._goal_id = None

    def _snapshot_and_pose(self, now=None):
        snapshot = self._bridge.get_occupancy_grid()
        # Nav2Client dispatches goals in map. Require the same frame for the grid.
        if snapshot is None or snapshot["frame_id"] != "map":
            return None, None
        if now is None:
            now = time.monotonic()
        if not 0.0 <= now - snapshot["received_monotonic"] <= MAX_MAP_AGE_S:
            return None, None
        return snapshot, self._bridge.get_exploration_pose(snapshot["frame_id"])

    @staticmethod
    def _snapshot_key(snapshot):
        return snapshot["frame_id"], snapshot["sequence"]

    async def _select_goals(self, snapshot, pose):
        # A cancellation may leave a running thread. Wait for it before sharing
        # cached grid state with another worker.
        if self._worker is not None:
            await asyncio.shield(self._worker)
        key = self._snapshot_key(snapshot)
        grid = self._grid if key == self._grid_key else None

        def select():
            current = grid if grid is not None else _FrontierGrid(snapshot)
            return current, current.goals(pose)

        self._worker = asyncio.create_task(asyncio.to_thread(select))
        self._grid, goals = await asyncio.shield(self._worker)
        self._worker = None
        self._grid_key = key
        return goals

    async def _explore_loop(self):
        visited = []
        failures = []
        try:
            while True:
                snapshot, pose = self._snapshot_and_pose()
                if pose is None:
                    self._set_state(STATE_SELECTING, reason=(
                        "Waiting for a fresh map-frame occupancy grid" if snapshot is None
                        else "Waiting for fresh map-frame localization"
                    ))
                    await asyncio.sleep(0.5)
                    continue

                goals = await self._select_goals(snapshot, pose)
                latest, fresh_pose = self._snapshot_and_pose()
                if (fresh_pose is None
                        or self._snapshot_key(latest) != self._grid_key
                        or self._grid.world_to_cell(*fresh_pose[:2]) != self._grid.world_to_cell(*pose[:2])):
                    await asyncio.sleep(POLL_S)
                    continue

                with self._lock:
                    self._frontiers_found = self._grid.count
                if goals is None:
                    self._set_state(STATE_SELECTING, reason="Robot pose is outside the traversable map")
                    await asyncio.sleep(0.5)
                    continue

                remaining = [
                    goal for goal in goals
                    if not any(_matches(goal, success) and goal.gain <= success.gain for success in visited)
                ]
                if not remaining:
                    if len(goals) < self._grid.count:
                        self._set_state(STATE_BLOCKED, reason="Frontiers remain without reachable approaches")
                    else:
                        self._set_state(STATE_COMPLETE)
                    return

                now = time.monotonic()
                eligible = []
                for goal in remaining:
                    failure = next((f for f in failures if _matches(goal, f[0])), None)
                    if failure is None or failure[1] < MAX_ATTEMPTS:
                        eligible.append((goal, failure))
                if not eligible:
                    self._set_state(STATE_BLOCKED, reason="Frontiers remain after navigation retries")
                    return

                selected = next(((goal, failure) for goal, failure in eligible
                                 if failure is None or failure[2] <= now), None)
                if selected is None or self._nav2.is_busy():
                    self._set_state(STATE_SELECTING, reason=(
                        "Waiting to retry frontier goals" if selected is None
                        else "Waiting for Nav2 to finish its active goal"
                    ))
                    await asyncio.sleep(0.5)
                    continue
                goal, failure = selected

                goal_id = self._nav2.navigate_to(goal.x, goal.y, goal.heading)
                if goal_id is None:
                    self._set_state(STATE_SELECTING, reason="Waiting for Nav2 to accept goals")
                    await asyncio.sleep(2.0)
                    continue
                self._goal_id = goal_id
                self._set_state(STATE_EXPLORING, goal)
                result = await self._wait_for_nav2(goal_id)
                self._goal_id = None
                pause_reason = {
                    "MAP_STALE": "Waiting for a fresh map-frame occupancy grid",
                    "LOCALIZATION_LOST": "Waiting for fresh map-frame localization",
                }.get(result)
                self._set_state(STATE_SELECTING, reason=pause_reason)

                if result == "SUCCEEDED":
                    visited.append(goal)
                    if failure is not None:
                        failures.remove(failure)
                    with self._lock:
                        self._frontiers_visited += 1
                elif result in ("ABORTED", "REJECTED", "ERROR", "ACCEPT_TIMEOUT", "TIMEOUT", "STUCK"):
                    if result == "STUCK":
                        await self._recover_from_stuck()
                    attempts = 1
                    if failure is not None:
                        failures.remove(failure)
                        attempts += failure[1]
                    failures.append((goal, attempts, time.monotonic() + RETRY_COOLDOWN_S))
                elif result == "CANCELED":
                    self._set_state(STATE_STOPPED)
                    return

                logger.info("FrontierExplorer: goal (%.2f, %.2f) ended with %s", goal.x, goal.y, result)
                await asyncio.sleep(0.5 if result == "SUCCEEDED" else 2.0)

        except asyncio.CancelledError:
            logger.debug("FrontierExplorer: task cancelled")
        except Exception:
            logger.exception("FrontierExplorer: unexpected error")
            self._cancel_goal()
            self._set_state(STATE_STOPPED)

    async def _wait_for_nav2(self, goal_id):
        start = last_progress = time.monotonic()
        best_distance = math.inf
        history = []          # (t, x, y, yaw, moving_cmd, (vx, vy))
        while True:
            status = self._nav2.get_status(goal_id)
            if status.get("outcome") is not None:
                return status["outcome"]
            state = status["state"]
            if state == "IDLE":
                return "CANCELED"
            if state == "UNAVAILABLE":
                return "UNAVAILABLE"
            if state == "FAILED":
                return "ERROR"

            now = time.monotonic()
            snapshot, pose = self._snapshot_and_pose(now)
            if pose is None:
                self._cancel_goal()
                return "MAP_STALE" if snapshot is None else "LOCALIZATION_LOST"

            distance = status.get("dist")
            if (distance is not None and math.isfinite(distance)
                    and (distance > 0.0 or distance == 0.0 and math.isfinite(best_distance))):
                if distance <= best_distance - PROGRESS_STEP_M:
                    best_distance = distance
                    last_progress = now
            if self._is_stuck(history, now, pose):
                self._cancel_goal()
                return "STUCK"
            if state == "PLANNING" and now - start >= ACCEPT_TIMEOUT_S:
                self._cancel_goal()
                return "ACCEPT_TIMEOUT"
            if now - last_progress >= NO_PROGRESS_TIMEOUT_S:
                self._cancel_goal()
                return "TIMEOUT"
            await asyncio.sleep(POLL_S)


    def _is_stuck(self, history, now, pose):
        if self.get_nav_cmd is None:
            return False
        cmd = self.get_nav_cmd()
        vx, vy, wz, age = cmd if cmd is not None else (0.0, 0.0, 0.0, math.inf)
        moving = age < 0.5 and (math.hypot(vx, vy) >= STUCK_MIN_CMD_V or abs(wz) >= STUCK_MIN_CMD_W)
        history.append((now, pose[0], pose[1], pose[2], moving, (vx, vy)))
        while history and now - history[0][0] > STUCK_WINDOW_S:
            history.pop(0)
        if not history or now - history[0][0] < STUCK_WINDOW_S - 0.5:
            return False
        if not all(h[4] for h in history):
            return False
        t0, x0, y0, a0 = history[0][:4]
        moved = math.hypot(pose[0] - x0, pose[1] - y0)
        turned = abs(math.atan2(math.sin(pose[2] - a0), math.cos(pose[2] - a0)))
        if moved >= STUCK_MOVE_M or turned >= STUCK_TURN_RAD:
            return False
        self._stuck_info = (pose, [h[5] for h in history])
        return True

    async def _recover_from_stuck(self):
        """Back out opposite the commanded direction, then mark a keep-out disc."""
        pose, cmds = self._stuck_info
        vx = float(np.mean([c[0] for c in cmds]))
        vy = float(np.mean([c[1] for c in cmds]))
        n = math.hypot(vx, vy)
        ux, uy = (vx / n, vy / n) if n > 1e-3 else (1.0, 0.0)   # body frame
        self.stuck_count += 1
        logger.warning("FrontierExplorer: STUCK at (%.2f, %.2f) cmd dir (%.2f, %.2f); backing out",
                       pose[0], pose[1], ux, uy)
        # Wait for the Nav2 cancel to settle so its zeros do not fight the back-out.
        for _ in range(15):
            if not self._nav2.is_busy():
                break
            await asyncio.sleep(0.1)
        if self.drive_fn is not None:
            end = time.monotonic() + BACKOUT_S
            while time.monotonic() < end:
                self.drive_fn(-ux * BACKOUT_SPEED, -uy * BACKOUT_SPEED, 0.0)
                await asyncio.sleep(0.05)
            self.drive_fn(0.0, 0.0, 0.0)
        if self.add_keepout is not None:
            c, s = math.cos(pose[2]), math.sin(pose[2])
            kx = pose[0] + KEEPOUT_AHEAD_M * (c * ux - s * uy)
            ky = pose[1] + KEEPOUT_AHEAD_M * (s * ux + c * uy)
            self.add_keepout(kx, ky, KEEPOUT_RADIUS_M)
            logger.info("FrontierExplorer: keep-out disc at (%.2f, %.2f) r=%.2f", kx, ky, KEEPOUT_RADIUS_M)


def _matches(goal, other):
    return math.dist(goal.anchor, other.anchor) <= FRONTIER_MATCH_M
