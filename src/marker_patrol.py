"""Back-and-forth multi-marker Nav2 patrol controller."""

from __future__ import annotations

import logging
import math
import threading
import time

try:
    from nav2_client import (
        STATE_FAILED,
        STATE_IDLE,
        STATE_SUCCEEDED,
        STATE_UNAVAILABLE,
    )
except ImportError:
    STATE_IDLE = "IDLE"
    STATE_SUCCEEDED = "SUCCEEDED"
    STATE_FAILED = "FAILED"
    STATE_UNAVAILABLE = "UNAVAILABLE"

logger = logging.getLogger(__name__)

PATROL_IDLE = "IDLE"
PATROL_ACTIVE = "PATROLLING"
PATROL_RETURNING = "RETURNING_FIRST"
PATROL_FAILED = "FAILED"

MODE_PING_PONG = "ping_pong"
MODE_LOOP = "loop"
VALID_MODES = (MODE_PING_PONG, MODE_LOOP)


def _parse_markers(markers) -> list[dict]:
    """Validate and normalize marker dicts to finite {x, y, theta} floats."""
    if not isinstance(markers, (list, tuple)) or len(markers) < 2:
        raise ValueError("At least 2 markers are required for patrol")
    parsed = []
    for i, m in enumerate(markers):
        if not isinstance(m, dict):
            raise ValueError(f"Marker {i + 1} is not an object")
        try:
            x = float(m["x"])
            y = float(m["y"])
            theta = float(m.get("theta", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Marker {i + 1} has invalid coordinates") from exc
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(theta)):
            raise ValueError(f"Marker {i + 1} has non-finite coordinates")
        parsed.append({"x": x, "y": y, "theta": theta})
    return parsed


class MarkerPatrol:
    """Drives the robot back and forth between map markers via Nav2Client."""

    def __init__(self, nav2_client, poll_s: float = 0.2) -> None:
        self._nav2 = nav2_client
        self._poll_s = float(poll_s)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._generation = 0

        self._state = PATROL_IDLE
        self._markers: list[dict] = []
        self._active_index: int | None = None
        self._direction = 1
        self._leg_count = 0
        self._current_goal_id: int | None = None
        self._return_to_first = False
        self._mode = MODE_PING_PONG
        self._reason: str | None = None

    def is_active(self) -> bool:
        with self._lock:
            return self._state in (PATROL_ACTIVE, PATROL_RETURNING)

    def set_return_to_first(self, enabled: bool) -> None:
        with self._lock:
            self._return_to_first = bool(enabled)

    def set_mode(self, mode: str) -> bool:
        if mode not in VALID_MODES:
            return False
        with self._lock:
            self._mode = mode
            if mode == MODE_LOOP:
                self._direction = 1
        return True

    def start(
        self,
        markers,
        return_to_first: bool = False,
        mode: str = MODE_PING_PONG,
    ) -> tuple[bool, str]:
        """Begin patrol across the given markers in ping_pong or loop mode."""
        if mode not in VALID_MODES:
            return False, f"Invalid patrol mode: {mode}"
        try:
            parsed = _parse_markers(markers)
        except ValueError as exc:
            return False, str(exc)

        with self._lock:
            if self._state in (PATROL_ACTIVE, PATROL_RETURNING):
                return False, "Patrol is already running"

            first = parsed[0]
            goal_id = self._nav2.navigate_to(first["x"], first["y"], first["theta"])
            if goal_id is None:
                return False, "Nav2 is unavailable or a goal is still being cancelled"

            self._generation += 1
            gen = self._generation
            self._stop_event = threading.Event()
            self._markers = parsed
            self._active_index = 0
            self._direction = 1
            self._leg_count = 1
            self._current_goal_id = goal_id
            self._return_to_first = bool(return_to_first)
            self._mode = mode
            self._state = PATROL_ACTIVE
            self._reason = None

            self._thread = threading.Thread(
                target=self._run_loop,
                args=(gen, self._stop_event),
                name="marker-patrol",
                daemon=True,
            )
            self._thread.start()

        logger.info(
            "[MarkerPatrol] Started with %d markers (mode=%s, return_to_first=%s)",
            len(parsed), mode, bool(return_to_first),
        )
        return True, f"Patrol started ({len(parsed)} markers, {mode})"

    def stop(self, return_to_first: bool | None = None) -> tuple[bool, str]:
        """
        Stop the patrol.

        If return_to_first is enabled and the patrol is currently active,
        navigate back to the first marker before idling. A second stop()
        while returning cancels immediately and stops in place.
        """
        cancel_nav = False
        msg = "Patrol stopped in place"
        with self._lock:
            if return_to_first is not None:
                self._return_to_first = bool(return_to_first)

            if self._state not in (PATROL_ACTIVE, PATROL_RETURNING):
                return False, "Patrol is not running"

            if self._state == PATROL_RETURNING or not self._return_to_first:
                self._generation += 1
                self._stop_event.set()
                self._state = PATROL_IDLE
                self._active_index = None
                self._current_goal_id = None
                self._reason = "Stopped in place"
                cancel_nav = True
            else:
                first = self._markers[0]
                goal_id = self._nav2.navigate_to(first["x"], first["y"], first["theta"])
                if goal_id is None:
                    self._generation += 1
                    self._stop_event.set()
                    self._state = PATROL_FAILED
                    self._active_index = None
                    self._current_goal_id = None
                    self._reason = "Failed to send return-to-first goal"
                    cancel_nav = True
                    msg = self._reason
                else:
                    self._state = PATROL_RETURNING
                    self._active_index = 0
                    self._current_goal_id = goal_id
                    self._reason = "Returning to marker #1"
                    msg = "Returning to marker #1"

        if cancel_nav:
            self._nav2.cancel()
        logger.info("[MarkerPatrol] %s", msg)
        return True, msg

    def abort(self) -> None:
        """Immediately stop in place (used by Motion Lock / E-stop / Nav2 shutdown)."""
        with self._lock:
            was_active = self._state in (PATROL_ACTIVE, PATROL_RETURNING)
            self._generation += 1
            self._stop_event.set()
            if was_active:
                self._state = PATROL_IDLE
                self._active_index = None
                self._current_goal_id = None
                self._reason = "Aborted"
        if was_active:
            self._nav2.cancel()
            logger.info("[MarkerPatrol] Aborted")

    def status(self) -> dict:
        with self._lock:
            return {
                "state": self._state,
                "active_index": self._active_index,
                "marker_count": len(self._markers),
                "leg_count": self._leg_count,
                "return_to_first": self._return_to_first,
                "mode": self._mode,
                "reason": self._reason,
            }

    def _next_step_locked(self) -> tuple[int, int]:
        """Compute (next_index, next_direction) for loop or ping-pong traversal."""
        n = len(self._markers)
        if self._mode == MODE_LOOP:
            return ((self._active_index or 0) + 1) % n, 1
        idx = (self._active_index or 0) + self._direction
        direction = self._direction
        if idx >= n:
            direction = -1
            idx = n - 2
        elif idx < 0:
            direction = 1
            idx = 1
        return idx, direction

    def _run_loop(self, gen: int, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            with self._lock:
                if gen != self._generation or self._state not in (PATROL_ACTIVE, PATROL_RETURNING):
                    return
                goal_id = self._current_goal_id
                state = self._state

            nav_status = self._nav2.get_status(goal_id)
            nav_state = nav_status.get("state")

            if nav_state == STATE_SUCCEEDED:
                with self._lock:
                    if gen != self._generation:
                        return
                    if state == PATROL_RETURNING:
                        self._state = PATROL_IDLE
                        self._active_index = None
                        self._current_goal_id = None
                        self._reason = "Returned to marker #1"
                        logger.info("[MarkerPatrol] Returned to marker #1")
                        return
                    if state == PATROL_ACTIVE:
                        next_idx, next_dir = self._next_step_locked()
                        target = self._markers[next_idx]
                        new_goal_id = self._nav2.navigate_to(
                            target["x"], target["y"], target["theta"]
                        )
                        if new_goal_id is None:
                            self._state = PATROL_FAILED
                            self._active_index = None
                            self._current_goal_id = None
                            self._reason = "Nav2 unavailable for next marker"
                            logger.warning("[MarkerPatrol] %s", self._reason)
                            return
                        self._active_index = next_idx
                        self._direction = next_dir
                        self._leg_count += 1
                        self._current_goal_id = new_goal_id
            elif nav_state in (STATE_FAILED, STATE_UNAVAILABLE, STATE_IDLE):
                # STATE_IDLE can occur if the goal was externally cancelled
                # (e.g. a manual Nav2 cancel or preemption).
                with self._lock:
                    if gen != self._generation:
                        return
                    if nav_state == STATE_IDLE:
                        self._state = PATROL_IDLE
                        self._reason = "Cancelled"
                    else:
                        self._state = PATROL_FAILED
                        self._reason = nav_status.get("fail_reason") or "Navigation failed"
                    self._active_index = None
                    self._current_goal_id = None
                    logger.info("[MarkerPatrol] Ended (%s): %s", self._state, self._reason)
                    return

            stop_event.wait(self._poll_s)
