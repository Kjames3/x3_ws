"""
nav2_client.py
Nav2 action client for the Yahboom X3 holonomic robot.

Provides:
  - navigate_to(x, y, theta)   – send a NavigateToPose goal
  - cancel()                   – cancel the active goal
  - set_initial_pose(x, y, theta) – publish AMCL initial pose
  - launch_nav2(...)           – spawn x3_nav2.launch.py subprocess
  - stop_nav2()                – kill that subprocess
  - get_status()               – snapshot dict for the broadcast loop

Designed to be injected into server_x3.py's ROS2Bridge node so both
share the same rclpy node (avoids spinning two nodes in one process).
"""

from __future__ import annotations

import math
import os
import subprocess
import threading
import time
import logging
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path

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
        self._goal_handle = None
        self._goal: Optional[dict] = None           # {x, y, theta}
        self._path: List[List[float]] = []           # [[x, y], ...]
        self._dist_remaining: Optional[float] = None
        self._nav2_proc: Optional[subprocess.Popen] = None

        # ── Action client ───────────────────────────────────────────────────
        self._action_client = ActionClient(
            node, NavigateToPose, 'navigate_to_pose')

        # ── Initial pose publisher (for AMCL) ───────────────────────────────
        self._initial_pose_pub = node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ── Global plan subscriber ──────────────────────────────────────────
        node.create_subscription(Path, '/plan', self._plan_cb, 10)

        logger.info("[Nav2Client] Initialised (action: navigate_to_pose)")

    # ── Public API ──────────────────────────────────────────────────────────

    def navigate_to(self, x: float, y: float, theta: float = 0.0) -> None:
        """Send a NavigateToPose goal (x, y in metres, theta in radians)."""
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            logger.warning("[Nav2Client] navigate_to_pose action server not available")
            with self._lock:
                self._state = STATE_UNAVAILABLE
            return

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
            self._state = STATE_PLANNING
            self._goal = {"x": x, "y": y, "theta": theta}
            self._path = []
            self._dist_remaining = None

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_response_cb)
        logger.info(f"[Nav2Client] Goal sent → ({x:.2f}, {y:.2f}, θ={math.degrees(theta):.1f}°)")

    def cancel(self) -> None:
        """Cancel the currently active navigation goal."""
        with self._lock:
            handle = self._goal_handle
            if handle is None:
                self._state = STATE_IDLE
                return
            self._state = STATE_CANCELLING

        cancel_future = handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_cb)
        logger.info("[Nav2Client] Cancel requested")

    def set_initial_pose(self, x: float, y: float, theta: float = 0.0) -> None:
        """Publish an initial pose estimate to /initialpose (used by AMCL)."""
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
    ) -> bool:
        """
        Spawn x3_nav2.launch.py as a subprocess.

        Returns True if the process was started, False if already running.
        """
        with self._lock:
            if self._nav2_proc is not None and self._nav2_proc.poll() is None:
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

        logger.info(f"[Nav2Client] Launching Nav2: {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as exc:
            logger.error(f"[Nav2Client] Failed to launch Nav2: {exc}")
            return False

        with self._lock:
            self._nav2_proc = proc
            self._state = STATE_IDLE

        return True

    def stop_nav2(self) -> None:
        """Terminate the Nav2 subprocess if running."""
        with self._lock:
            proc = self._nav2_proc
            self._nav2_proc = None

        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            logger.info("[Nav2Client] Nav2 subprocess stopped")

        with self._lock:
            self._state = STATE_IDLE
            self._goal = None
            self._path = []
            self._dist_remaining = None
            self._goal_handle = None

    def get_status(self) -> dict:
        """Return a snapshot dict safe to serialise to JSON."""
        with self._lock:
            path = self._path
            # Subsample to ≤ _MAX_PATH_PTS points for the broadcast
            if len(path) > _MAX_PATH_PTS:
                step = len(path) // _MAX_PATH_PTS
                path = path[::step]
            return {
                "state":     self._state,
                "goal":      self._goal,
                "path":      path,
                "dist":      self._dist_remaining,
                "nav2_running": (
                    self._nav2_proc is not None
                    and self._nav2_proc.poll() is None
                ),
            }

    # ── Internal callbacks ──────────────────────────────────────────────────

    def _goal_response_cb(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            logger.warning("[Nav2Client] Goal rejected by Nav2")
            with self._lock:
                self._state = STATE_FAILED
                self._goal_handle = None
            return

        with self._lock:
            self._goal_handle = handle
            self._state = STATE_EXECUTING

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)
        logger.info("[Nav2Client] Goal accepted, executing")

    def _feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        with self._lock:
            self._dist_remaining = fb.distance_remaining

    def _result_cb(self, future) -> None:
        result = future.result()
        status = result.status
        with self._lock:
            self._goal_handle = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._state = STATE_SUCCEEDED
                logger.info("[Nav2Client] Goal SUCCEEDED")
            elif status == GoalStatus.STATUS_CANCELED:
                self._state = STATE_IDLE
                self._goal = None
                logger.info("[Nav2Client] Goal CANCELLED")
            else:
                self._state = STATE_FAILED
                logger.warning(f"[Nav2Client] Goal FAILED (status={status})")
            self._dist_remaining = None

    def _cancel_done_cb(self, future) -> None:
        with self._lock:
            self._state = STATE_IDLE
            self._goal = None
            self._goal_handle = None
        logger.info("[Nav2Client] Cancel confirmed")

    def _plan_cb(self, msg: Path) -> None:
        """Store the global plan as a list of [x, y] pairs."""
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        with self._lock:
            self._path = pts
