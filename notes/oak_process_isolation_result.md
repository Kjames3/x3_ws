# OAK-D process isolation — measured, no improvement, reverted

**Date:** 2026-08-17 · **Robot:** jetson (Orin Nano, 6 cores) · **Branch:** `worktree-oak-process-isolation`

## Question

`oakd_driver._run` is the top CPU consumer in `x3_server` and was believed to hold the
GIL while doing the mm→m depth conversion and host-side YOLO decode, starving the
asyncio event loop. Would moving it to its own process fix that?

**Answer: no.** Measured, reverted on the robot the same session.

## Metric

`broadcast_loop` is capped at 20 Hz (`await asyncio.sleep(0.05)`). Its *actual*
inter-arrival period, observed from a WebSocket client, is a direct readout of
event-loop stalls. Probe run **on the robot over localhost** so campus-WiFi jitter
could not contaminate it (`scripts/probe_event_loop.py`). 3 × 60 s per arm.

## Result

| metric (3-run mean) | baseline (in-process) | process-isolated | post-revert |
|---|---|---|---|
| readout rate | 12.06 Hz | 11.70 Hz | 11.81 Hz |
| period p50 | 69 ms | 72 ms | 71 ms |
| period p95 | ~223 ms | ~207 ms | 225 ms |
| period p99 | ~293 ms | ~294 ms | 274 ms |
| stalls >200 ms/min | 42 | 38 | 48 |
| depth fps | 27.0 | 26.7 | 26.0 |

Every delta is smaller than the baseline's own run-to-run spread. The A-B-A
(post-revert) column lands back in the baseline band, so the null is real.

CPU while isolated: parent ~85%, child ~44% (~129% total, up from one process) —
the isolation *added* aggregate cost via the extra memcpy + pickling.

## Why the hypothesis was wrong

The transport was verified working, so this is a true null and not a broken path:
payload check showed `oak_imu` populated on 242/242 readouts with live accel/gyro,
depth 30 fps, detections list present, intrinsics fx=615.6 correct.

The likely explanation is that **numpy already releases the GIL** for the large
elementwise operations that dominate `_run` (the mm→m conversion, the decode's
sigmoid/NMS). "50% CPU" is not the same as "50% of the GIL" — the driver was
burning CPU on another core without actually blocking the event loop. The
remaining ~12 Hz shortfall against the 20 Hz target therefore comes from work
*inside* the loop body (velocity-estimator MLP, WebRTC, ROS, CBF), not from
contention with the OAK thread.

## Where to look instead

The bimodal period distribution (p90 ~94 ms → p95 ~223 ms) points at something
periodically blocking for ~150 ms, roughly 0.7×/s. That periodicity — not the OAK
driver — is the thing worth chasing. Profile the `broadcast_loop` body directly
rather than the driver.

## Status

- Robot fully reverted: drop-in removed, `server_x3.py` byte-identical to its
  pre-patch backup, new modules deleted, no stray processes or `/dev/shm` segments.
- Code kept on this branch, off by default behind `--oak-process`, in case the
  loop-body work is ever reduced enough for GIL contention to become the binder.

## Unrelated pre-existing bug found

`VelocityEstimator: depth source exposes no get_raw_depth_frame()` fires at startup
when the OAK device connect (~4 s) loses the race against estimator init. It
reproduces on the **unmodified** build, so it is not caused by this change. It is
self-healing (the depth source is re-fetched each cycle, and `_legacy_path_warned`
only suppresses the repeat log), but it means obstacles are not reported for the
first few seconds after boot.
