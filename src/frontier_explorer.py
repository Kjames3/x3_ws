"""
frontier_explorer.py
Autonomous frontier-based exploration for the Yahboom X3 robot.

Overview
--------
Frontier exploration repeatedly finds the boundary between known free space
and unknown space on the SLAM occupancy grid, picks the nearest frontier
cluster as the next goal, and sends it to Nav2 via Nav2Client.  The robot
autonomously covers the environment until no frontiers remain.

Integration requirements
------------------------
- ROS2 mode must be active (SLAM Toolbox publishing /map, Nav2 running).
- Nav2Client must be initialised and Nav2 must be launched before calling start().
- ROS2Bridge must expose get_occupancy_grid() and get_pose_m() (added alongside
  this file).

Usage (from server_x3.py)
--------------------------
    explorer = FrontierExplorer(nav2_client, ros2_bridge)
    explorer.start()   # begins background asyncio task
    explorer.stop()    # cancels current goal and halts exploration
    explorer.status()  # dict for telemetry broadcast

Algorithm
---------
1. Fetch latest OccupancyGrid from ROS2Bridge.
2. Detect frontier cells: free cells (0) with at least one unknown neighbour (-1).
3. Label connected frontier clusters with scipy.ndimage (or fallback BFS).
4. For each cluster compute centroid in map coordinates.
5. Select the nearest centroid to the robot's current pose.
6. Send it as a Nav2 goal; wait for SUCCEEDED or FAILED.
7. On SUCCEEDED: repeat from step 1.
   On FAILED:    skip this frontier, try the next nearest.
8. When no frontiers remain: transition to COMPLETE.
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Explorer states (mirrored in GUI telemetry)
# ---------------------------------------------------------------------------
STATE_IDLE       = "IDLE"
STATE_EXPLORING  = "EXPLORING"    # actively navigating to a frontier
STATE_SELECTING  = "SELECTING"    # computing next frontier goal
STATE_COMPLETE   = "COMPLETE"     # no frontiers remain
STATE_STOPPED    = "STOPPED"      # user-cancelled

# How long (s) to wait for Nav2 to accept / execute a goal before giving up
GOAL_TIMEOUT     = 30.0
# Minimum frontier cluster size (cells) — smaller clusters are noise
MIN_CLUSTER_SIZE = 10
# Distance (m) within which a frontier centroid is considered "reached" and skipped
REACHED_THRESHOLD = 0.3


class FrontierExplorer:
    """
    Autonomous frontier exploration controller.

    Parameters
    ----------
    nav2_client : Nav2Client
        The server's Nav2Client instance (must already be initialised).
    bridge : ROS2Bridge
        The server's ROS2Bridge (provides occupancy grid + robot pose).
    """

    def __init__(self, nav2_client, bridge):
        self._nav2   = nav2_client
        self._bridge = bridge

        self._state  = STATE_IDLE
        self._lock   = threading.Lock()

        # Exploration bookkeeping
        self._visited_frontiers: list[tuple[float, float]] = []
        self._current_goal: Optional[tuple[float, float]] = None
        self._goal_start_time: float = 0.0
        self._frontiers_found: int = 0
        self._frontiers_visited: int = 0

        # asyncio task handle (set by start())
        self._task: Optional[asyncio.Task] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Begin exploration.  Returns False if already running."""
        with self._lock:
            if self._state in (STATE_EXPLORING, STATE_SELECTING):
                logger.warning("FrontierExplorer: already running")
                return False
            self._state = STATE_SELECTING
            self._visited_frontiers.clear()
            self._frontiers_found   = 0
            self._frontiers_visited = 0

        self._task = asyncio.get_event_loop().create_task(self._explore_loop())
        logger.info("FrontierExplorer: started")
        return True

    def stop(self) -> None:
        """Stop exploration and cancel any active Nav2 goal."""
        with self._lock:
            self._state = STATE_STOPPED

        if self._task and not self._task.done():
            self._task.cancel()

        try:
            self._nav2.cancel()
        except Exception:
            pass

        logger.info("FrontierExplorer: stopped")

    def status(self) -> dict:
        """Return a dict suitable for the telemetry broadcast."""
        with self._lock:
            return {
                "frontier_state":     self._state,
                "frontiers_found":    self._frontiers_found,
                "frontiers_visited":  self._frontiers_visited,
                "current_goal":       list(self._current_goal) if self._current_goal else None,
            }

    # ------------------------------------------------------------------
    # Core exploration loop (runs as an asyncio Task)
    # ------------------------------------------------------------------

    async def _explore_loop(self) -> None:
        try:
            while True:
                with self._lock:
                    if self._state == STATE_STOPPED:
                        return

                # 1. Get latest grid and robot pose
                grid_info = self._bridge.get_occupancy_grid()
                pose_m    = self._bridge.get_pose_m()

                if grid_info is None:
                    logger.debug("FrontierExplorer: waiting for /map...")
                    await asyncio.sleep(1.0)
                    continue

                # 2. Detect frontiers
                frontiers = _find_frontier_centroids(
                    grid_info["data"],
                    grid_info["width"],
                    grid_info["height"],
                    grid_info["resolution"],
                    grid_info["origin_x"],
                    grid_info["origin_y"],
                )

                with self._lock:
                    self._frontiers_found = len(frontiers)

                if not frontiers:
                    logger.info("FrontierExplorer: no frontiers remain — exploration complete")
                    with self._lock:
                        self._state    = STATE_COMPLETE
                        self._current_goal = None
                    return

                # 3. Filter already-visited frontiers
                rx, ry = pose_m["x"], pose_m["y"]
                unvisited = [
                    f for f in frontiers
                    if not _near_any(f, self._visited_frontiers, REACHED_THRESHOLD)
                ]

                if not unvisited:
                    logger.info("FrontierExplorer: all reachable frontiers visited")
                    with self._lock:
                        self._state    = STATE_COMPLETE
                        self._current_goal = None
                    return

                # 4. Select nearest unvisited frontier centroid
                goal = min(unvisited, key=lambda f: math.hypot(f[0] - rx, f[1] - ry))

                with self._lock:
                    self._state        = STATE_EXPLORING
                    self._current_goal = goal

                logger.info(f"FrontierExplorer: navigating to frontier ({goal[0]:.2f}, {goal[1]:.2f})")
                self._goal_start_time = time.monotonic()

                # 5. Send Nav2 goal — face the frontier direction
                heading = math.atan2(goal[1] - ry, goal[0] - rx)
                self._nav2.navigate_to(goal[0], goal[1], heading)

                # 6. Wait for goal to finish
                result = await self._wait_for_nav2()

                # If Nav2 was unavailable the robot never moved — don't mark this
                # frontier as visited or we'll exhaust all frontiers without exploring.
                if result == "UNAVAILABLE":
                    logger.warning("FrontierExplorer: Nav2 unavailable, pausing 5 s then retrying")
                    with self._lock:
                        self._state = STATE_SELECTING
                        self._current_goal = None
                    await asyncio.sleep(5.0)
                    continue

                self._visited_frontiers.append(goal)
                with self._lock:
                    self._frontiers_visited += 1
                    self._state = STATE_SELECTING

                if result == "SUCCEEDED":
                    logger.info("FrontierExplorer: goal reached, selecting next frontier")
                elif result == "TIMEOUT":
                    logger.warning("FrontierExplorer: goal timed out, skipping frontier")
                else:
                    logger.warning(f"FrontierExplorer: goal ended with '{result}', skipping frontier")

                # Brief pause to let SLAM update the map before re-scanning
                await asyncio.sleep(0.5)

        except asyncio.CancelledError:
            logger.info("FrontierExplorer: task cancelled")
        except Exception as e:
            logger.error(f"FrontierExplorer: unexpected error: {e}")
            with self._lock:
                self._state = STATE_STOPPED

    async def _wait_for_nav2(self) -> str:
        """Poll Nav2Client state until the goal finishes or times out."""
        from nav2_client import STATE_SUCCEEDED, STATE_FAILED, STATE_IDLE, STATE_UNAVAILABLE
        while True:
            elapsed = time.monotonic() - self._goal_start_time
            if elapsed > GOAL_TIMEOUT:
                self._nav2.cancel()
                return "TIMEOUT"

            nav_state = self._nav2.get_status().get("state", STATE_IDLE)
            if nav_state == STATE_SUCCEEDED:
                return "SUCCEEDED"
            if nav_state in (STATE_FAILED, STATE_IDLE, STATE_UNAVAILABLE):
                return nav_state

            with self._lock:
                if self._state == STATE_STOPPED:
                    return "STOPPED"

            await asyncio.sleep(0.05)


# ---------------------------------------------------------------------------
# Frontier detection — pure numpy, no ROS dependency
# ---------------------------------------------------------------------------

def _find_frontier_centroids(
    data: list[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> list[tuple[float, float]]:
    """
    Detect frontier centroids in world coordinates (metres).

    Parameters
    ----------
    data       : flat OccupancyGrid data (-1=unknown, 0=free, 1-100=occupied)
    width, height : grid dimensions in cells
    resolution : metres per cell
    origin_x/y : world coords of the grid's bottom-left corner

    Returns
    -------
    List of (x, y) world-coordinate centroids, one per frontier cluster.
    Clusters smaller than MIN_CLUSTER_SIZE cells are discarded as noise.
    """
    grid = np.array(data, dtype=np.int8).reshape(height, width)

    # Frontier mask: free cells (0) that have at least one unknown neighbour (-1)
    free    = (grid == 0)
    unknown = (grid == -1)

    # Dilate unknown by 1 cell (4-connected), then intersect with free
    unknown_d = np.zeros_like(unknown)
    unknown_d[:-1, :] |= unknown[1:, :]   # shift up
    unknown_d[1:,  :] |= unknown[:-1, :]  # shift down
    unknown_d[:, :-1] |= unknown[:, 1:]   # shift left
    unknown_d[:, 1:]  |= unknown[:, :-1]  # shift right

    frontier_mask = free & unknown_d

    if not frontier_mask.any():
        return []

    # Label connected components
    try:
        from scipy.ndimage import label as scipy_label
        labeled, n_labels = scipy_label(frontier_mask)
    except ImportError:
        # Fallback: simple BFS labelling when scipy is unavailable
        labeled, n_labels = _bfs_label(frontier_mask)

    centroids: list[tuple[float, float]] = []
    for lbl in range(1, n_labels + 1):
        cells = np.argwhere(labeled == lbl)   # shape (N, 2): rows=y, cols=x
        if len(cells) < MIN_CLUSTER_SIZE:
            continue
        cy_cell = cells[:, 0].mean()   # row → y
        cx_cell = cells[:, 1].mean()   # col → x
        # Convert cell indices to world coordinates
        world_x = origin_x + (cx_cell + 0.5) * resolution
        world_y = origin_y + (cy_cell + 0.5) * resolution
        centroids.append((world_x, world_y))

    return centroids


def _bfs_label(mask: np.ndarray):
    """
    Simple BFS connected-component labelling — used only when scipy is absent.
    Returns (labeled_array, n_labels) matching scipy.ndimage.label output.
    """
    from collections import deque
    labeled  = np.zeros_like(mask, dtype=np.int32)
    n_labels = 0
    rows, cols = np.where(mask)

    for r0, c0 in zip(rows, cols):
        if labeled[r0, c0]:
            continue
        n_labels += 1
        queue = deque([(r0, c0)])
        labeled[r0, c0] = n_labels
        while queue:
            r, c = queue.popleft()
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < mask.shape[0] and 0 <= nc < mask.shape[1]:
                    if mask[nr, nc] and not labeled[nr, nc]:
                        labeled[nr, nc] = n_labels
                        queue.append((nr, nc))

    return labeled, n_labels


def _near_any(
    point: tuple[float, float],
    visited: list[tuple[float, float]],
    threshold: float,
) -> bool:
    """Return True if point is within threshold metres of any visited location."""
    px, py = point
    for vx, vy in visited:
        if math.hypot(px - vx, py - vy) < threshold:
            return True
    return False
