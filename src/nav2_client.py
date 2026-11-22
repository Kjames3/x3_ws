"""
nav2_client.py
Nav2 action client for the Yahboom X3 holonomic robot.

Provides:
  - navigate_to(x, y, theta)   – send a NavigateToPose goal
  - cancel()                   – cancel the active goal
  - set_initial_pose(x, y, theta) – publish AMCL initial pose
  - launch_nav2(...)           – spawn x3_nav2.launch.py subprocess
  - stop_nav2()                – kill that subprocess
  - load_map(path)             – hot-swap the AMCL/map_server map while running
  - get_status()               – snapshot dict for the broadcast loop

Designed to be injected into server_x3.py's ROS2Bridge node so both
share the same rclpy node (avoids spinning two nodes in one process).
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time
import logging
from dataclasses import dataclass, field
from functools import partial
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

logger = logging.getLogger(__name__)

# Nav2 state strings mirrored in the GUI badge
STATE_IDLE        = "IDLE"
STATE_PLANNING    = "PLANNING"
STATE_EXECUTING   = "EXECUTING"
STATE_SUCCEEDED   = "SUCCEEDED"
STATE_FAILED      = "FAILED"
STATE_CANCELLING  = "CANCELLING"
STATE_UNAVAILABLE = "UNAVAILABLE"

# Maximum path sample points sent to the GUI each broadcast cycle
_MAX_PATH_PTS = 50


def _yaw_to_quaternion(yaw: float):
    """Return (x, y, z, w) quaternion for a pure Z-axis rotation."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


# Humble's NavigateToPose result carries no error code, so a failed goal is just
# "ABORTED". These are the Nav2 log lines that say why, most specific first.
_NAV2_LOG_NODES = ("bt_navigator", "controller_server", "planner_server",
                   "behavior_server", "smoother_server", "amcl")
_FAILURE_PATTERNS = [
    (("failed to make progress",), "Robot stopped making progress (blocked, or wheels slipping)"),
    (("collision ahead",), "Recovery behaviour stopped: obstacle too close"),
    (("start occupied", "starting point in lethal space", "start coordinates", "robot is out of bounds"),
     "Robot is inside an obstacle on the costmap (or off the map)"),
    (("goal occupied", "goal coordinates", "goal is in lethal", "goal is out of bounds"),
     "Goal is inside an obstacle or outside the map"),
    (("failed to create plan", "no valid path", "failed to plan", "plugin failed to plan", "no path"),
     "Planner found no path to the goal"),
    (("patience exceeded", "resulting plan has 0 poses", "controller failed", "failed to compute"),
     "Controller could not follow the path"),
    (("extrapolation", "lookup would require", "transform", "tf "),
     "TF/localization problem (missing or stale transform)"),
]


def classify_nav_failure(lines):
    """Pick the most specific reason from Nav2 WARN/ERROR lines (oldest first)."""
    low = [ln.lower() for ln in lines]
    for needles, reason in _FAILURE_PATTERNS:
        for ln in reversed(low):
            if any(n in ln for n in needles):
                return reason
    if lines:
        return "Nav2: " + lines[-1][:160]
    return "Nav2 aborted the goal without logging a reason"


@dataclass
class _Goal:
    coordinates: dict
    started_at: float = field(default_factory=time.monotonic)
    state: str = STATE_PLANNING
    handle: object | None = None
    cancel_pending: bool = False
    outcome: str | None = None
    fail_reason: str | None = None
    dist: float | None = None
    elapsed_s: float | None = None
    eta_s: float | None = None
    recoveries: int = 0
    logs: list[str] = field(default_factory=list)


class Nav2Client:
    """
    Thin wrapper around the Nav2 NavigateToPose action client.

    Parameters
    ----------
    node : rclpy.node.Node
        An already-spinning rclpy node (shared with ROS2Bridge).
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._lock = threading.Lock()

        # ── State ───────────────────────────────────────────────────────────
        self._state: str = STATE_IDLE
        self._next_goal_id = 0
        self._current_goal_id: int | None = None
        self._goals: dict[int, _Goal] = {}
        self._active_goals: set[int] = set()
        self._stopping = False
        self._path: List[List[float]] = []
        self._nav2_proc: Optional[subprocess.Popen] = None
        # Map YAML currently served to AMCL (basename, GUI display only)
        self._current_map: Optional[str] = None
        self._external_slam = False

        # ── Action client ───────────────────────────────────────────────────
        self._action_client = ActionClient(
            node, NavigateToPose, 'navigate_to_pose')

        # ── Initial pose publisher (for AMCL) ───────────────────────────────
        self._initial_pose_pub = node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ── Global plan subscriber ──────────────────────────────────────────
        node.create_subscription(Path, '/plan', self._plan_cb, 10)
        node.create_subscription(
            Log, '/rosout', self._rosout_cb,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))

        # ── map_server hot-swap client (created lazily on first load_map) ───
        self._load_map_cli = None

        logger.info("[Nav2Client] Initialised (action: navigate_to_pose)")

    # ── Public API ──────────────────────────────────────────────────────────

    def navigate_to(self, x: float, y: float, theta: float = 0.0) -> int | None:
        """Submit a map-frame goal and return its ID, or None if refused."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(float(theta))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        with self._lock:
            if self._is_cancelling():
                return None
            if not self._action_client.server_is_ready():
                logger.warning("[Nav2Client] navigate_to_pose action server not available")
                if not self._active_goals:
                    self._state = STATE_UNAVAILABLE
                    self._current_goal_id = None
                    self._path = []
                return None

            self._next_goal_id += 1
            goal_id = self._next_goal_id
            try:
                send_future = self._action_client.send_goal_async(
                    goal_msg, feedback_callback=lambda msg: self._feedback_cb(goal_id, msg))
            except Exception:
                logger.exception("[Nav2Client] Could not send goal")
                if not self._active_goals:
                    self._state = STATE_UNAVAILABLE
                    self._current_goal_id = None
                    self._path = []
                return None

            previous_id = self._current_goal_id
            if previous_id in self._active_goals:
                self._goals[previous_id].state = STATE_CANCELLING
            self._current_goal_id = goal_id
            self._goals[goal_id] = _Goal({"x": x, "y": y, "theta": theta})
            self._active_goals.add(goal_id)
            self._path = []
            # Keep a bounded terminal history for consumers polling by ID.
            for old_id in list(self._goals):
                if old_id <= goal_id - 32 and old_id not in self._active_goals:
                    del self._goals[old_id]

        if previous_id is not None:
            self.cancel(previous_id)
        send_future.add_done_callback(partial(self._goal_response_cb, goal_id))
        logger.info(f"[Nav2Client] Goal sent → ({x:.2f}, {y:.2f}, θ={math.degrees(theta):.1f}°)")
        return goal_id

    def cancel(self, goal_id: int | None = None) -> None:
        """Request cancellation of one goal, or all outstanding goals."""
        with self._lock:
            ids = list(self._active_goals) if goal_id is None else [goal_id]
            requests = []
            for current_id in ids:
                if current_id not in self._active_goals:
                    continue
                goal = self._goals[current_id]
                goal.state = STATE_CANCELLING
                if current_id == self._current_goal_id:
                    self._path = []
                if goal.handle is not None and not goal.cancel_pending:
                    goal.cancel_pending = True
                    requests.append((current_id, goal.handle))

        for current_id, handle in requests:
            try:
                future = handle.cancel_goal_async()
                future.add_done_callback(partial(self._cancel_done_cb, current_id, handle))
            except Exception as exc:
                self._cancel_failed(current_id, f"Nav2 cancel request failed: {exc}")

    def set_initial_pose(self, x: float, y: float, theta: float = 0.0) -> None:
        """Publish an initial pose estimate to /initialpose (used by AMCL)."""
        self.cancel()
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(float(theta))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Covariance: loose x, y, yaw; everything else zero
        cov = [0.0] * 36
        cov[0]  = 0.25   # x
        cov[7]  = 0.25   # y
        cov[35] = 0.07   # yaw (~15°)
        msg.pose.covariance = cov
        self._initial_pose_pub.publish(msg)
        logger.info(f"[Nav2Client] Initial pose published ({x:.2f}, {y:.2f}, θ={math.degrees(theta):.1f}°)")

    def launch_nav2(
        self,
        use_sim_time: bool = False,
        map_path: Optional[str] = None,
        slam: bool = False,
        external_slam: bool = False,
    ) -> bool:
        """
        Spawn x3_nav2.launch.py as a subprocess.

        Returns True if the process was started, False if already running.
        """
        with self._lock:
            if self._stopping or (self._nav2_proc is not None and self._nav2_proc.poll() is None):
                logger.warning("[Nav2Client] Nav2 already running")
                return False

        # Locate the launch file via ament
        try:
            from ament_index_python.packages import get_package_share_directory
            nav_pkg = get_package_share_directory('yahboomcar_nav')
            launch_file = os.path.join(nav_pkg, 'launch', 'x3_nav2.launch.py')
        except Exception as exc:
            logger.error(f"[Nav2Client] Could not find yahboomcar_nav package: {exc}")
            return False

        cmd = [
            'ros2', 'launch', launch_file,
            f'use_sim_time:={"True" if use_sim_time else "False"}',
            f'slam:={"True" if slam else "False"}',
        ]
        if map_path:
            cmd.append(f'map:={map_path}')
        if external_slam:
            cmd.append('external_slam:=true')

        # Keep Nav2's console output: with DEVNULL a launch that loaded no
        # nodes left nothing to read (2026-11-22). Overwritten each launch.
        log_path = os.path.expanduser('~/.ros/log/x3_nav2_launch.log')
        logger.info(f"[Nav2Client] Launching Nav2: {' '.join(cmd)} (output: {log_path})")
        try:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            log_file = open(log_path, 'w')
        except OSError:
            log_file = subprocess.DEVNULL
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                # Own process group, so stop_nav2 can reach every Nav2 node
                # (separate processes since use_composition False).
                start_new_session=True,
            )
        except Exception as exc:
            logger.error(f"[Nav2Client] Failed to launch Nav2: {exc}")
            return False
        finally:
            if log_file is not subprocess.DEVNULL:
                log_file.close()     # the child keeps its own copy

        with self._lock:
            self._nav2_proc = proc
            self._state = STATE_IDLE
            self._external_slam = external_slam
            self._current_map = (None if slam or external_slam
                                 else os.path.basename(map_path) if map_path
                                 else 'yahboomcar.yaml')

        return True

    def is_busy(self) -> bool:
        """True while a goal is planning, executing, or being cancelled.

        The tilting lidar interlock uses this: a tilted scan plane means
        slam_toolbox and AMCL are being fed nothing (lidar_3d_processor_node
        gates /scan off), so Nav2 would be planning against a costmap that
        stops updating while the robot is still allowed to drive.
        """
        with self._lock:
            return bool(self._active_goals) or self._stopping

    def is_driving(self) -> bool:
        """True while the current goal is planning or executing."""
        with self._lock:
            goal = self._goals.get(self._current_goal_id)
            return goal is not None and goal.state in (STATE_PLANNING, STATE_EXECUTING)

    def is_cancelling(self) -> bool:
        """True while any goal is awaiting cancellation or Nav2 is stopping."""
        with self._lock:
            return self._is_cancelling()

    def _is_cancelling(self) -> bool:
        """Check the cancellation barrier with the lock already held."""
        return self._stopping or any(
            self._goals[g].state == STATE_CANCELLING for g in self._active_goals
        )

    def load_map(self, map_path: str, timeout: float = 10.0) -> tuple[bool, str]:
        """
        Hot-swap the map AMCL localises against, without restarting Nav2.

        Calls nav2's /map_server/load_map service. map_server republishes the
        new OccupancyGrid on /map (transient-local), and because `first_map_only`
        is not enabled in nav2_params_x3.yaml, AMCL rebuilds its likelihood
        field from it — so this works while the robot is driving.

        Note: the robot's current pose is kept. If the new map has a different
        origin/frame the estimate will be wrong; call set_initial_pose() (or the
        GUI's "Set Initial Pose") afterwards.

        Returns (success, message).
        """
        if not os.path.isfile(map_path):
            return False, f"Map file not found: {map_path}"

        with self._lock:
            running = (self._nav2_proc is not None
                       and self._nav2_proc.poll() is None)
        if self._load_map_cli is None:
            self._load_map_cli = self._node.create_client(
                LoadMap, '/map_server/load_map')

        if not self._load_map_cli.wait_for_service(timeout_sec=3.0):
            hint = ("Nav2 map_server not available — launch Nav2 first "
                    "(AMCL mode, SLAM Mode unchecked)")
            if running:
                hint = "Nav2 map_server not available (still starting up?)"
            return False, hint

        req = LoadMap.Request()
        req.map_url = map_path
        future = self._load_map_cli.call_async(req)

        # The node spins on its own thread, so poll the future instead of
        # calling rclpy.spin_until_future_complete (which would deadlock).
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            future.cancel()
            return False, "Timed out waiting for /map_server/load_map"

        try:
            result = future.result()
        except Exception as exc:                       # service raised
            return False, f"load_map call failed: {exc}"

        if result.result == LoadMap.Response.RESULT_SUCCESS:
            with self._lock:
                self._current_map = os.path.basename(map_path)
            logger.info(f"[Nav2Client] AMCL map switched to {map_path}")
            return True, f'Map "{os.path.basename(map_path)}" loaded'

        codes = {
            LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST: "map file does not exist",
            LoadMap.Response.RESULT_INVALID_MAP_DATA: "invalid map image data",
            LoadMap.Response.RESULT_INVALID_MAP_METADATA: "invalid map YAML metadata",
        }
        reason = codes.get(result.result, f"undefined failure (code {result.result})")
        logger.warning(f"[Nav2Client] load_map rejected: {reason}")
        return False, f"map_server rejected the map: {reason}"

    def stop_nav2(self) -> None:
        """Terminate the Nav2 subprocess if running."""
        with self._lock:
            if self._stopping:
                return
            proc = self._nav2_proc
            self._stopping = proc is not None

        self.cancel()
        if proc is None:
            return

        try:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    logger.warning("[Nav2Client] Nav2 did not stop in 10 s; killing its process group")
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    proc.wait(timeout=5.0)
                logger.info("[Nav2Client] Nav2 subprocess stopped")
        finally:
            with self._lock:
                self._stopping = False
                if proc.poll() is not None:
                    self._nav2_proc = None
                    for goal_id in list(self._active_goals):
                        self._finish_goal(goal_id, STATE_IDLE, "CANCELED")
                    self._state = STATE_IDLE
                    self._current_goal_id = None
                    self._path = []
                    self._current_map = None
                    self._external_slam = False

    def get_status(self, goal_id: int | None = None) -> dict:
        """Return a JSON-safe status, optionally for a specific goal ID."""
        with self._lock:
            current = goal_id is None or goal_id == self._current_goal_id
            goal = self._goals.get(self._current_goal_id if goal_id is None else goal_id)
            path = self._path if current else []
            if len(path) > _MAX_PATH_PTS:
                step = math.ceil(len(path) / _MAX_PATH_PTS)
                path = path[::step]
            elapsed_s = goal.elapsed_s if goal else None
            if elapsed_s is None and goal and goal.state in (STATE_PLANNING, STATE_EXECUTING):
                elapsed_s = time.monotonic() - goal.started_at
            return {
                "state": goal.state if goal else (self._state if current else STATE_IDLE),
                "goal": goal.coordinates if goal and goal.state != STATE_IDLE else None,
                "path": path,
                "dist": goal.dist if goal else None,
                "elapsed_s": elapsed_s,
                "eta_s": goal.eta_s if goal else None,
                "recoveries": goal.recoveries if goal else 0,
                "fail_reason": goal.fail_reason if goal else None,
                "outcome": goal.outcome if goal else None,
                "map": self._current_map,
                "external_slam": self._external_slam,
                "nav2_running": self._nav2_proc is not None and self._nav2_proc.poll() is None,
            }

    def _goal_response_cb(self, goal_id, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._mark_uncertain(goal_id, f"Nav2 goal acceptance unknown: {exc}")
            return

        with self._lock:
            if not handle.accepted:
                if goal_id in self._active_goals:
                    self._finish_goal(
                        goal_id, STATE_FAILED, "REJECTED",
                        "Nav2 rejected the goal (is it inside the map? is Nav2 active?)",
                    )
                return

            active = goal_id in self._active_goals
            if active:
                goal = self._goals[goal_id]
                goal.handle = handle
                cancel = goal.state == STATE_CANCELLING or goal_id != self._current_goal_id
                if not cancel:
                    goal.state = STATE_EXECUTING

        if not active:
            # An owned Nav2 process may have stopped before its late response.
            try:
                handle.cancel_goal_async()
            except Exception:
                logger.exception("[Nav2Client] Could not cancel late Nav2 acceptance")
            return

        try:
            result_future = handle.get_result_async()
            result_future.add_done_callback(partial(self._result_cb, goal_id))
        except Exception as exc:
            self._mark_uncertain(goal_id, f"Nav2 goal result unavailable: {exc}")
            return
        if cancel:
            self.cancel(goal_id)

    def _feedback_cb(self, goal_id, feedback_msg) -> None:
        fb = feedback_msg.feedback
        with self._lock:
            if goal_id != self._current_goal_id or goal_id not in self._active_goals:
                return
            goal = self._goals[goal_id]
            if goal.state == STATE_CANCELLING:
                return
            goal.dist = fb.distance_remaining
            goal.elapsed_s = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
            eta = fb.estimated_time_remaining.sec + fb.estimated_time_remaining.nanosec * 1e-9
            goal.eta_s = eta if eta > 0 else None
            goal.recoveries = int(fb.number_of_recoveries)

    def _rosout_cb(self, msg) -> None:
        try:
            self._rosout_handle(msg)
        except Exception:
            logger.debug('rosout callback failed', exc_info=True)

    def _rosout_handle(self, msg) -> None:
        if msg.level < 30 or not any(n in msg.name for n in _NAV2_LOG_NODES):
            return
        if "control loop missed" in msg.msg.lower():
            return
        with self._lock:
            if self._current_goal_id in self._active_goals:
                goal = self._goals[self._current_goal_id]
                goal.logs.append(f"{msg.name}: {msg.msg}")
                del goal.logs[:-30]

    def _finish_goal(self, goal_id, state, outcome, reason=None) -> None:
        """Record a terminal result with the lock held."""
        self._active_goals.remove(goal_id)
        goal = self._goals[goal_id]
        goal.state = state
        goal.outcome = outcome
        goal.fail_reason = reason
        goal.handle = None
        goal.dist = None
        logger.info("[Nav2Client] Goal %s %s%s", goal_id, outcome, f": {reason}" if reason else "")

    def _mark_uncertain(self, goal_id, reason) -> None:
        with self._lock:
            if goal_id not in self._active_goals:
                return
            goal = self._goals[goal_id]
            goal.outcome = "ERROR"
            goal.fail_reason = reason
        logger.warning("[Nav2Client] Goal %s: %s", goal_id, reason)
        # Keep ownership until the action or its owned process is terminal.
        self.cancel(goal_id)

    def _result_cb(self, goal_id, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:
            self._mark_uncertain(goal_id, f"Nav2 goal result unknown: {exc}")
            return
        if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                          GoalStatus.STATUS_ABORTED):
            self._mark_uncertain(goal_id, f"Nav2 returned a nonterminal goal status ({status})")
            return

        with self._lock:
            if goal_id not in self._active_goals:
                return
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._finish_goal(goal_id, STATE_SUCCEEDED, "SUCCEEDED")
            elif status == GoalStatus.STATUS_CANCELED:
                self._finish_goal(goal_id, STATE_IDLE, "CANCELED")
            else:
                reason = classify_nav_failure(self._goals[goal_id].logs)
                self._finish_goal(goal_id, STATE_FAILED, "ABORTED", reason)

    def _cancel_done_cb(self, goal_id, handle, future) -> None:
        try:
            response = future.result()
            if any(info.goal_id == handle.goal_id for info in response.goals_canceling):
                return  # The action's result confirms terminal state.
            reason = f"Nav2 did not accept cancellation (code {response.return_code})"
        except Exception as exc:
            reason = f"Nav2 cancel response failed: {exc}"
        self._cancel_failed(goal_id, reason)

    def _cancel_failed(self, goal_id, reason) -> None:
        with self._lock:
            if goal_id not in self._active_goals:
                return
            goal = self._goals[goal_id]
            goal.cancel_pending = False
            goal.fail_reason = reason
        logger.warning("[Nav2Client] Goal %s: %s", goal_id, reason)

    def _plan_cb(self, msg: Path) -> None:
        """Store the global plan as a list of [x, y] pairs."""
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        with self._lock:
            goal = self._goals.get(self._current_goal_id)
            if goal and goal.state in (STATE_PLANNING, STATE_EXECUTING):
                self._path = pts
