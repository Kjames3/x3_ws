---
id: A-49
title: "Pedestrian Topics Republished at 2× the Producer Rate and Never Published Empty — Unbounded Stale Markers and a `/pedestrian_states` Array That Never Clears"
status: Logged
domain: Performance
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-49"
session: "3. Performance & Execution Efficiency"
tags: [idea]
---

# A-49 — Pedestrian Topics Republished at 2× the Producer Rate and Never Published Empty — Unbounded Stale Markers and a `/pedestrian_states` Array That Never Clears

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (One sequence counter, one deleted `and`, and one DELETEALL marker; halves the DDS work and removes a hazard state that never expires)
- **Problem:** The broadcast loop pulls and republishes the estimator's output on every tick (`src/server_x3.py:2011–2018`):
  ```python
  velocity_estimates = velocity_estimator.get_estimates()
  if ROS2_MODE and ros_bridge is not None and velocity_estimates:
      ros_bridge.publish_pedestrians(velocity_estimates)
  ```
  That loop runs at **20 Hz** (`await asyncio.sleep(0.05)  # 20 FPS cap`, line 2086) while the producer runs at **10 Hz** (`INFER_HZ = 10`, `src/velocity_estimator.py:34`, paced at lines 666–668). There is no freshness check, so every estimate set is serialised and published to three topics twice. Three distinct defects follow.

  1. **Half the DDS work is a duplicate, and it runs on the asyncio event loop.** `publish_pedestrians` (`src/server_x3.py:577–677`) builds, per track, one `Pose`, one `PedestrianState`, an ARROW `Marker` and a TEXT_VIEW_FACING `Marker` — each marker carrying a nested `Header`, `Pose`, `Vector3` and `ColorRGBA`. At the `MAX_OBSTACLES = 5` ceiling that is 20 top-level messages and roughly 120 Python object constructions per call, then three `publish()` calls with their CDR serialisation (lines 675–677). At 20 Hz that is $\approx 2400$ object constructions per second, **half of them re-encoding bytes that were already sent $50\text{ ms}$ earlier**, on the same coroutine that must also ship the JPEG frame (line 2022) and the readout JSON (line 2056) inside the same $50\text{ ms}$ budget. Each call also takes `VelocityEstimator._lock` through `get_estimates()` (lines 685–688), contending with the inference thread that holds it at line 604.

  2. **The "no pedestrians" state is never published at all.** The `and velocity_estimates` guard makes the publish conditional on the list being non-empty, so when the last person leaves the frame — or the empty-frame fast path clears `self._estimates` at lines 435–436 — *nothing is sent*. The last non-empty `PedestrianArray` therefore remains the newest message on `/pedestrian_states` indefinitely. Any consumer that treats the latest message as current state reads "there is still a pedestrian here, moving at $v$" forever. That is not hypothetical: Idea J-15's forward-projected costmap layer and Idea J-13's CBF obstacle-drift term are both specified to consume exactly this stream, and both would inherit a phantom hazard that no timeout clears. A stale *hazard* is the one kind of stale state that cannot be argued as fail-safe, because it makes the robot avoid empty floor.

  3. **RViz markers accumulate without bound.** `marker.id = int(tid)` (line 632) and `text_marker.id = int(tid) + 1000` (line 661) come from `ObstacleTracker.next_id`, which increments on every new track and is **never reset** (`src/velocity_estimator.py:106–107`). No marker carries a `lifetime`, and no `Marker.DELETEALL` is ever published. Every ID that ever existed leaves a permanent arrow and label frozen at its last reported position. Under the track churn Idea A-27 documents — blobs merging and splitting in clutter destroy and recreate tracks continuously — even a modest 2 new IDs/s is $7{,}200$ permanent markers after an hour, all of which RViz keeps rendering and all of which are now in the wrong place (Idea A-47).
- **Proposed Solution:** Publish once per produced frame, publish the empty state, and let markers expire.
  1. **Sequence the producer.** Add `self._seq = 0` in `VelocityEstimator.__init__` and `self._seq += 1` inside the lock beside `self._estimates = estimates` (line 605); return it from `get_estimates()` (or add `get_seq()`). In the broadcast loop, republish only when `seq` changes. This halves the publish rate to the producer's $10\text{ Hz}$ with **zero information loss** — the second copy is bit-identical by construction. The same counter is what Idea A-43 step 2 needs to detect a duplicate depth frame, so one field serves both.
  2. **Delete the `and velocity_estimates` guard** at line 2015. An empty `PoseArray` / `PedestrianArray` is the correct, unambiguous "no pedestrians" message, and it is nearly free to serialise. Combine with (1) so the empty state is published exactly once per producer frame rather than at $20\text{ Hz}$.
  3. **Prepend a clear marker.** Make `markers[0]` a single `Marker` with `action = Marker.DELETEALL` on every `MarkerArray` — the standard RViz idiom — so the display is rebuilt from scratch each publish and no dead ID can survive. Belt-and-braces alternative, or addition: set `marker.lifetime = Duration(sec=0, nanosec=200_000_000)` (two producer periods) so a marker self-expires if a publish is ever missed. DELETEALL is the primary fix here because the ID space is unbounded, which is exactly the case `lifetime` alone handles poorly.
  4. **Get it off the event loop.** With (1) in place, the natural home for `publish_pedestrians` is the estimator's own thread — call it at the end of `_inference_loop` right after line 605, where the data is produced and the cadence is already correct — or an `rclpy` timer on `self._node`. The broadcast coroutine then only puts `velocity_estimates` into the readout JSON at line 2046, and no ROS message construction or CDR serialisation happens on the thread that owns the WebSocket clients at all.
  5. Composes with Idea A-47, which is what makes the published position correct in the first place, and with Idea A-36/J-08's `conf` export, which should ride in the same payload change.
- **Expected Benefit:** Halves pedestrian-topic publishing work — $\approx 1200$ Python object constructions per second and three CDR serialisations per tick — and removes all of it from the asyncio event loop that also carries the $20\text{ Hz}$ JPEG and readout broadcasts, plus one lock acquisition per tick that today contends with the inference thread. More importantly it makes the *cleared* state representable: `/pedestrian_states` stops being a latch that holds the last pedestrian forever, which is a prerequisite for Ideas J-13 and J-15 consuming it safely, and RViz stops accumulating thousands of permanently wrong markers over a session. Cost is one counter, one deleted conjunction, and one extra marker per array.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-49`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
