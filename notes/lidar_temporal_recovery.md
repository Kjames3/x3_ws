# Temporal consistency and local recovery — 2026-09-07

User-approved approach is saved in shared memory as
`~/.agent-memory/project-lidar-localization-recovery-plan.md`.

Implemented `lidar_tracking.py`, integrated it into the diagnostic ROS node,
and installed it on the Jetson. This changes diagnostic output only: no TF or
velocity publisher, no AMCL handover, no interlock changes, no hardware motion
or normal-service restart. The diagnostic node is not left running.

## Implemented policy (provisional)

Ordinary tracking compares map-to-odom corrections at the current odometry pose
so both real motion and map-origin lever arms are accounted for. It requires
4 supporting matches in a window of 5, within 6 cm and 2 degrees of a medoid,
over at least 0.35 s and 10 degrees of tilt variation. The newest match must
belong to that cluster. Evidence older than 1 s is discarded. The published
correction is an actual medoid observation, not an average of opposing poses.
The prior 30 cm / 15 degree match limits remain unchanged for tracking.

States: UNINITIALIZED, SUSPECT, TRACKING, LOST, SEARCHING, VERIFYING, PAUSED.
Initialization supplies a prior, not immediate evidence of a trusted pose.
Repeated disagreement, time since trusted evidence, and missing input cause
loss handling. No withheld/stale result updates trusted correction. No old pose
is re-published with a new stamp. A late computation is rolled back before any
trusted update can escape. Pose outputs retain acquisition timestamps.

Recovery freezes five odometry-aligned clouds, downsamples to at most 2,000
points, and searches 27 nearby seeds (x/y offsets +/-0.6 m; heading +/-20 deg).
It tries only ONE seed per subsequent input callback; it does not block a
callback on the whole search. Recovery matching may propose larger corrections
(up to 1.2 m / 40 deg from a seed), but those do not update trust.

Distinct candidates are clustered using position and heading; up to three are
retained. More than three distinct admitted clusters causes rejection rather
than hiding alternatives. No candidate after a full search also remains lost.
There is no global search fallback: uniqueness is LOCAL to this bounded search.

Candidates are evaluated at fixed map-to-odom corrections on strictly LATER
clouds, never the proposal window. A candidate needs five subsequent successful
geometry checks with angular/time diversity, and a mean-score advantage of
0.08 over the next candidate (score = overlap - NN_RMSE/0.15 m). Verification
has a 12-frame limit. Failure/ambiguity produces no pose and returns to LOST.
A lone candidate must still pass those later checks; absence of a rival in the
bounded search is not proof of global uniqueness.

## Pause API and ROS behavior

`/lidar_localizer/status` retains its event-oriented `state` and adds an explicit
uppercase `localization_state`. Inspect the latter to distinguish lost,
searching, verifying and trusted tracking. Withheld events include reasons.

`/lidar_localizer/paused` (std_msgs/Bool) explicitly declares an intentional
cloud-source pause. True clears provisional evidence and prevents publication;
false resumes SUSPECT and requires new evidence. Publish this around intentional
sweep/capture pauses. The node does NOT infer intention merely from silence.
The normal no-cloud watchdog reports LOST on unexpected silence.

Use the existing diagnostic launch and explicit map-aligned initialization
instructions in lidar_localization_diagnostic.md. `max_age_s` may be reduced,
but cannot exceed 0.5 s. The new policy is currently defined in the immutable
Policy dataclass, with exact values recorded in each evaluation JSON. Do not
change live parameters and assume these policy values changed.

## Evaluation on existing development recording

This is the same frozen map and the same offline initial alignment from the
first 35 clouds of station 1 as the previous baseline. The other 574 clouds are
processed sequentially, without per-station pose reinitialization. The evaluator
uses known station boundaries to explicitly pause/resume across intentional
no-cloud driving intervals. Those boundaries do not provide a map pose.

The initial policy was evaluated without a threshold search. Nevertheless,
this run informed the design and is now DEVELOPMENT DATA, not a fresh independent
validation set. It has no position ground truth. Published-pose spread measures
stationary consistency only; withholding more samples also changes the evaluated
subset. A stable, wrong location is still possible.

| Station | Old outputs | New outputs | Old p95 xy radius | New p95 xy radius |
|---|---:|---:|---:|---:|
| 1 | 35 | 25 | 18.5 cm | 2.9 cm |
| 2 | 75 | 60 | 7.7 cm | 3.8 cm |
| 3 | 75 | 63 | 4.6 cm | 4.3 cm |
| 4 | 76 | 61 | 7.7 cm | 3.8 cm |
| 5 | 0 | 0 | — | — |
| 6 | 78 | 37 | 3.7 cm | 2.3 cm |
| 7 | 16 | 24 | 23.8 cm | 2.5 cm |
| 8 | 0 | 0 | — | — |

Total: 270 published poses / 574 evaluated inputs (baseline 355/574). Both
laptop and Jetson made the same final publication decisions and recovered at
stations 6 and 7. Those recoveries are algorithmic confirmations, not independent
proof of correct map positions. First-output delay is ~0.42-0.56 s at stations
1-4, 5.02 s at station 6 and 5.72 s at station 7. The long recovery latency is a
real tradeoff. Stations 5 and 8 remain withheld rather than forcing a result.

Jetson full Tracker.process runtime: median 37.3 ms, p95 122.5 ms, max 128.0 ms.
The ICP 120 ms budget is checked between numerical operations, not a hard
real-time preemption guarantee. 43 match attempts hit their runtime budget;
they were rejected, not used as evidence. Runtime includes recovery work but
excludes ROS transport/TF. The robot service was active; simultaneous sweeping
and OctoMap insertion was NOT part of this replay workload. Headroom relative
to the ~139 ms acquisition period is limited.

46 relevant tests passed on laptop and Jetson. Tests cover motion-compensated
agreement, angular diversity, isolated outliers, stale/duplicate input, explicit
pause vs loss, fixed search geometry, distinct later-frame verification,
ambiguous hypotheses, and rollback of a late trusted result.

Installed-node ROS replay (isolated domain 43) passed on both machines: initial
three valid matches withheld pending evidence, then 9 diagnostic poses from
12 input clouds; stale input rejected, silence -> LOST, explicit pause ->
PAUSED, resume -> SUSPECT. Publisher inspection confirms no /tf, /tf_static or
/cmd_vel. Test data was restamped for replay, so it is not a latency capture.
The existing Jetson SciPy/NumPy version warning remains; no system dependency
upgrade was performed.

## Reproduction and remaining work

```bash
OPENBLAS_NUM_THREADS=1 python3 src/evaluate_lidar_tracking.py \
  --capture artifacts/localization-validation/localization_validation_01.npz \
  --map artifacts/localization-2026-09-06/map.npz \
  --bootstrap artifacts/localization-validation/frozen_map_result.json \
  --out artifacts/localization-recovery-2026-09-07/recheck.json
```

Results, exact policy, tests/build logs and ROS replay reports are under
artifacts/localization-recovery-2026-09-07 on both machines. Remote originals
were backed up there before the scoped deployment. The map is unchanged.

Remaining qualification: new independent stop-and-go observations with a
known initial pose, ambiguous/repeated geometry and dynamic-scene cases, and
full sweep/OctoMap load with input ages/queue drops measured. Stations 5 and 8
still need diagnosis; a bounded local search cannot handle arbitrary odometry
error or prove global uniqueness. Do not expand the map using this evaluation
capture and call the resulting self-fit a successful held-out test.
