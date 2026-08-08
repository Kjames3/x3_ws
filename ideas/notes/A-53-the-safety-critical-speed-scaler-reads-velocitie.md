---
id: A-53
title: "The Safety-Critical Speed Scaler Reads Velocities Through a WebSocket JSON Round-Trip, Not the ROS Topic Published Beside It"
status: Logged
domain: Performance
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-53"
session: "3. Performance & Execution Efficiency"
tags: [idea]
---

# A-53 — The Safety-Critical Speed Scaler Reads Velocities Through a WebSocket JSON Round-Trip, Not the ROS Topic Published Beside It

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (One `create_subscription` in a node that is already an rclpy node; removes ~55 ms mean transport delay and an unbounded stale-snapshot latch from the braking path)
- **Problem:** `_get_speed_scaling` — the function that decides whether the robot brakes for a pedestrian — reads `self._latest_estimates` under a lock (`src/ab_comparison_test.py:784–785`) at the $20\text{ Hz}$ control rate (`self._timer = self.create_timer(0.05, self._control_loop)`, line 225). The **only** writer of that field is inside a WebSocket client running in a daemon thread with its own asyncio loop (`_set_estimator_mode`, lines 720–747):
  ```python
  if data.get("type") == "readout":
      with self._estimates_lock:
          self._latest_estimates = data.get("velocity_estimates", [])
  ```
  So the estimates make a full round trip out of the process and back: estimator thread $\to$ asyncio broadcast coroutine $\to$ orjson $\to$ TCP loopback $\to$ `json.loads` $\to$ lock $\to$ control timer. Meanwhile `ROS2Bridge.publish_pedestrians` is building and publishing the same data as a typed `PedestrianArray` on `/pedestrian_states` (`src/server_x3.py:589–591, 611–626, 675–677`) — and a repository-wide grep for `pedestrian_states`, `pedestrian_poses` and the marker topic returns **no subscriber anywhere in the workspace**, not even an `.rviz` config. The one consumer with a safety function reads a JSON copy; the typed topic built for it is published to nobody.

  **Latency.** Three delays stack on top of the estimate's own age:
  1. Producer $\to$ broadcast poll. The estimator publishes at $10\text{ Hz}$ (`velocity_estimator.py:604–605`); the coroutine polls `get_estimates()` at $20\text{ Hz}$ (`server_x3.py:2011–2014`, `await asyncio.sleep(0.05)` at line 2086). Uniform $[0, 50)\text{ ms}$, mean $25\text{ ms}$.
  2. Serialise, transport, parse. The estimates ride inside the *whole* readout payload — `detections`, battery, power, motor telemetry, ~40 fields (line 2046) — on the same coroutine tick that also ships the JPEG camera frame (line 2022). Both ends pay for the full dict: $\approx 2\text{–}8\text{ ms}$, and it is contended rather than fixed.
  3. Client thread $\to$ control timer. Another uniform $[0, 50)\text{ ms}$, mean $25\text{ ms}$.

  Mean added delay $\approx 55\text{ ms}$, worst case $\approx 108\text{ ms}$ — **on top of** the frame age Idea A-43 shows is already up to $100\text{ ms}$. The position and velocity the brake acts on are therefore typically $\approx 150\text{ ms}$ and up to $\approx 250\text{ ms}$ old. For a pedestrian at $1.4\text{ m/s}$ that is $0.21\text{ m}$ of stale position typically and $0.35\text{ m}$ worst case — the latter equal to the **entire** $\texttt{LATERAL\_THRESHOLD} = 0.35\text{ m}$ corridor half-width (line 795) that decides whether the person is in the path at all. As pure delay it also shows up directly as brake lag: $4.5\text{–}12.5\text{ cm}$ of extra robot travel at the $0.3\text{–}0.5\text{ m/s}$ drive speed before the response starts.

  **Liveness, which is worse.** There is no timestamp and no staleness check on this path. On WebSocket loss the handler at lines 741–743 logs a warning, sleeps $1\text{ s}$ and reconnects — but `self._latest_estimates` is **never cleared**. `_get_speed_scaling` keeps evaluating a frozen snapshot indefinitely, in whichever of two bad directions the snapshot happens to hold: a frozen non-empty set with a closing velocity brakes the robot forever on empty floor, and a frozen empty list makes it blind. Note this is a **different** latch from the one Idea A-49 repairs. A-49 makes `/pedestrian_states` publish the empty state — which does nothing here, because the controller does not read that topic, and the readout JSON already carries `"velocity_estimates": []` correctly and unconditionally at line 2046. The content is fine; the *transport liveness* is not.
- **Proposed Solution:** Consume the typed topic that is already being published, stamp it with capture time, and gate on staleness.
  1. **Subscribe.** The controller is already an rclpy node with timers and publishers, so this is one call:
     ```python
     from x3_msgs.msg import PedestrianArray
     self.create_subscription(PedestrianArray, '/pedestrian_states', self._ped_cb, 10)
     ```
     Intra-host DDS delivers in $\approx 1\text{ ms}$, and the callback fires **on production** rather than on a $20\text{ Hz}$ poll, which removes both $25\text{ ms}$ poll waits, not just the serialisation. Keep the WebSocket connection for the `set_velocity_estimation` toggle it was written for (lines 723–728) — that is a control message, not a data stream.
  2. **Stamp it with the capture time, not the publish time.** `PedestrianArray.msg` already declares a `std_msgs/Header`, and `publish_pedestrians` already fills it — but with `self._node.get_clock().now()` at the moment of *publication* (`server_x3.py:584, 590`), which is the broadcast tick, not the frame. Carry Idea A-43's per-frame device timestamp through to `header.stamp` so the age is measurable end to end. Then gate in `_get_speed_scaling`: if `now - stamp > 0.3 s`, treat the estimate set as **unavailable** and fall back to the LiDAR corridor check the file already implements (lines ~690–712), rather than acting on a snapshot of unknown age.
  3. **Lands with Idea A-49, not after it.** A-49 step 2 deletes the `and velocity_estimates` guard so the empty array is actually published; without that, a subscriber inherits exactly the latch it just escaped from — the topic would simply stop updating when the last person leaves. A-49 makes the topic's *content* correct; this makes it the controller's *source*. Neither is complete alone.
  4. **Second-order saving.** With the controller off the readout, `velocity_estimates` in the $20\text{ Hz}$ JSON becomes GUI-only and can move to the existing $2\text{ Hz}$ slow lane (lines 2059–2085), removing per-tick serialisation of up to five nested dicts from the asyncio loop. Combined with A-49 step 4 (publishing from the estimator's own thread), no pedestrian data is constructed or serialised on the coroutine that owns the WebSocket clients at all.
- **Expected Benefit:** Removes $\approx 55\text{ ms}$ mean and $\approx 108\text{ ms}$ worst-case of pure transport delay from the braking decision — $0.08\text{–}0.15\text{ m}$ of pedestrian position error at walking speed, against a corridor whose half-width is $0.35\text{ m}$ — and converts a poll-on-a-poll into event-driven delivery at the producer's own $10\text{ Hz}$. Eliminates an unbounded stale-snapshot latch that today has no timeout of any kind on the one path that can stop the robot, replacing it with an explicit LiDAR-only degradation. And it makes the three ROS topics that are already being built and published on every tick actually load-bearing, rather than the second copy of data the controller reads over TCP.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-53`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
