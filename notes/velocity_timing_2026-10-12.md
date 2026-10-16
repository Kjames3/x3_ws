# Timestamped MLP measurements — 2026-10-12

Deployed to the robot on 2026-10-12; deployment checks are recorded below.
Not drive-tested by this change. Scope is OAK depth timing and
MLP sample cadence. No new weights, attention, person measurement method, C3
synchronization changes, or camera configuration changes.

## Changes

- `OakDCamera.get_depth_observation()` returns depth, matching intrinsics, capture
  time, receipt time, session and frame identity in one locked snapshot. Capture
  uses the existing SDK-to-host-monotonic conversion used by recording metadata.
- The live estimator processes each selected frame once. Stale/future/invalid
  timestamps clear estimates; unavailable capture-time pose suppresses updates.
  The loop still polls at 10 Hz; it does not process every 30 Hz depth frame.
- A separate pose history receives fused `/odom` only. Message acquisition time
  is mapped from the ROS clock to host monotonic time. Position and shortest-arc
  yaw are interpolated only between supported samples (maximum bracket 150 ms).
  Existing general-purpose latest-pose behavior is unchanged.
- Matched track positions carry capture timestamps. The MLP gets ten positions
  on a 100 ms grid ending at the latest measurement. There is no extrapolation
  or startup padding. A complete window needs 0.9 seconds of support. Gaps over
  250 ms reset the relevant history; unmatched tracks produce no new velocity.
- The acceleration clamp uses actual capture intervals. Timed outputs and feature
  logs include `capture_t`. Output coordinates/velocities use the robot frame
  at capture, not a prediction of where the object is at delivery time.
- Astra/untimestamped ROS camera paths and `_step(..., capture_t=None)` retain
  the legacy contract. This work does not timestamp those camera sources.

Training cadence was checked read-only on REAL-1 in
`~/x3_velsweep3/thor_magni_augment.py`: native 10 Hz CSVs, with strided windows
scaled back to the training cadence. Excerpt saved in
`artifacts/velocity-timing-2026-10-12/training-cadence-reference.txt`.

## Validation

68 targeted tests pass, including 10 new timing tests, existing feature/scaler
replay checks, input validity, intrinsics, person tags and OAK array handling.
Compilation and `git diff --check` pass. One existing test was excluded after
confirming its assertion fails on HEAD too:
`test_cbf_rejects_far_obstacles_before_the_speed_test` searches for an older CBF
code layout that has since been refactored. Its underlying CBF behavior was not
changed here.

Four recorded datasets were replayed for their first 30 seconds each using
`src/replay_velocity_timing.py`. Both arms use identical extracted centroids and
weights. Legacy uses 10 Hz polling with tick-time pose; timed deduplicates frames,
uses capture-time pose, and resamples the history. The replay uses recorded
acquisition timestamps, not an emulation of the full original delivery schedule.
Synthetic/live-loop tests separately exercise duplicate delivery and pose lookup.

| Recording | Legacy ticks with output / 301 | Timed ticks with output / 301 | Duplicate frames skipped | Long-gap resets |
|---|---:|---:|---:|---:|
| stand-2.4m-r2 | 299 | 289 | 2 | 0 |
| cross-2.4m-r2 | 299 | 291 | 0 | 0 |
| startstop-r2 | 299 | 270 | 3 | 1 |
| apt_loop03 | 299 | 275 | 0 | 0 |

These are ticks with **any** estimate, not person recall. Raw outputs and summaries
are in `artifacts/velocity-timing-2026-10-12/<recording>/`. No detector/person tags
were used, and there is no independent velocity truth in this comparison.

Standing-run output speed p95 changed from 0.238 to 0.203 m/s, and apartment-run
p95 from 1.173 to 1.158 m/s. Crossing/start-stop speeds increased. These cannot
establish an accuracy win: output populations differ, and a correct moving
object may legitimately have a larger speed. The reduction in outputs is partly
the intentional removal of unsupported short/stale histories.

## Practical limit and next check

The timing contract is tested, but this is not evidence of deployment readiness
or improved walking-speed accuracy. In particular, 0.9 s startup support can
reduce useful short-track coverage. Live capture-time pose availability is checked below. Person recall/onset on a
common reference population and a controlled moving-robot/static-scene check
remain to be validated.
The current broad depth blobs and association remain unchanged and can still
produce false motion. The offline implementation phase did not restart services or move hardware.

Reproduce one run:

```bash
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=1 python3 src/replay_velocity_timing.py \
  artifacts/c3-captures/stand-2.4m-r2 \
  --output artifacts/velocity-timing-2026-10-12/stand-2.4m-r2 --seconds 30
```


## Robot deployment

The user explicitly requested deployment. Robot and laptop were both based on
`dc33ce84bba477b876ca134671a4d2d991bec46f`; the three existing destination files
were clean before copying. Backups are on the robot in
`/home/jetson/x3_ws/backups/velocity-timing-2026-10-12/`.

Deployed `server_x3.py`, `oakd_driver.py`, `velocity_estimator.py` and the new
`velocity_timing.py`, checked matching SHA-256 hashes and compiled them on the
Jetson. Added `velocity_timing` counters to WebSocket readouts to verify actual
frame acceptance, pose availability, duplicate rejection and gap resets.
The ten timing tests were rerun successfully before deployment.

The first restart initially produced timestamped estimates, then ROS delivery
stalled: `/scan_raw` stopped reaching the lidar processor, IMU updates stopped
reaching the base driver, and fused odometry stopped supporting the estimator.
The estimator withheld velocities rather than substitute current/stale pose.
Evidence: `artifacts/velocity-timing-2026-10-12/deployment-ros-stall.json`.
This symptom also appeared in the pre-deployment server log. Recovered using
the previously documented order: restart `x3_server`, then `foxglove_bridge`.
No changes to DDS configuration were made.

Passive WebSocket checks after recovery are saved as
`artifacts/velocity-timing-2026-10-12/deployment-recovery-*.json`.
No driving commands were sent. Startup retains the server's normal tilt homing.

The remaining three items can be developed offline. The robot's newer
`artifacts/c3-captures-segpose` corpus contains 30 pair indexes, 25 reference
files, 50 mask archives and 50 cached person-measurement files. Prefer copying
those recordings to the laptop for comparisons. C3 synchronization has packet,
pose and depth timestamps in the older drive logs, but atomicity/queue behavior
still needs a live check. Person-position inputs and uncertainty calibration
can both be compared using existing replays; shared-camera references do not
establish independent absolute accuracy.

Recovery check result: three consecutive 20-second intervals accepted **592**
frames; **0** missing-pose failures, **0** invalid timestamps, **0** duplicate
frames and **0** gap resets. All **915/915** readouts contained estimates.
Per-interval median capture-to-readout age was 148–153 ms and p95 was 203–211 ms.
`x3_server` remained active (PID 22984), and lidar continued delivering about
35 scans per 5 seconds beyond the earlier stall point. These are operational
checks, not accuracy or driving validation.
