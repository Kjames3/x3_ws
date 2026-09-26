# TRIZ diagnostic and priority matrix

2026-09-20 planning artifact; status reviewed 2026-09-23 against the ToF implementation and recorded test results. C1 stage 1 is active on the Jetson and its physical ToF capture/replay passed; synchronization qualification remains pending.

**Recommended next step after ToF bench testing: establish synchronized, geometrically correct observations and separate obstacle existence from estimated motion. Then evaluate the person tracker and an offline STL monitor together.** Predictive control follows those results; RGB-D mapping is a separate branch.

Here STL means **Signal Temporal Logic**, as in the September 20 planning session. A TRIZ *contradiction matrix* describes competing engineering objectives; a statistical *confusion matrix* counts prediction errors. This document supplies a project-specific contradiction matrix plus a diagnostic error table. The TRIZ strategies below are engineering interpretations, not entries claimed from the classical 39-parameter lookup table. Priorities reflect dependencies and diagnostic value, not measured risk scores.

## Starting point: what is actually established

| Item | Evidence/status | Consequence |
|---|---|---|
| Dual ToF | Both sensors integrated into server/GUI/ROS/costmap/CBF; recorded 15 fps operation and low-obstacle drive tests (ceb32f2). | Operational ToF prerequisite passed; matched single/dual comparison deferred to apartment. |
| ToF mount | Dual 15°/40° arrays installed; recorded floor/obstacle tests support the transforms and grazing-zone correction. | Retain calibrated geometry; refresh floor baseline when mount/floor changes. |
| ToF timing | C1 stage 1 preserves read/receipt/publication times, sequences, clock epochs and raw validity in `/tof/observations`; physical capture/replay passes, synchronization still unqualified. | MCU acquisition/clock bounds and camera pairing remain open; C1 is not complete. |
| OAK timing | Current publisher assigns ROS `now()` during publication; the experiment plan identifies missing paired RGB acquisition data. | Repair the recording contract before interpreting small velocity differences. |
| Velocity baseline | v3 range-gate explanation superseded the missing-training-tail hypothesis. Old zero-phantom and later wide-range results use different populations. | Compare identical recordings and range bins; do not restart model training from the old explanation. |
| Lidar | Continuous 45°/s sweep was validated while parked; temporal recovery exists in diagnostic code. | Faster sweeping and reimplementing recovery are not the default next tasks. Moving validation and ambiguity remain separate questions. |
| Tracker / STL / robust MPC | Staged proposals, not qualified replacement behavior. | Begin with replay and shadow outputs. |

## 1. TRIZ contradiction matrix

Read each row as: **improving this objective can worsen that objective; separate the responsibilities and test whether the conflict actually improves.**

| ID / order | Improve | What can worsen | TRIZ strategy applied to this robot | Smallest discriminating experiment | Evidence needed to advance |
|---|---|---|---|---|---|
| C1 · first | Fast, dense multi-sensor observations | Timing error, transport load, stale data presented as fresh | Intermediary + separation: Teensy handles ranging; host keeps distinct measurement/read/receipt times and sensor identity | Short synchronized payload capture; quantify per-sensor rate, age, gaps, resets and pairing | Replay preserves identity, units, clock uncertainty and missing intervals; no cached evidence silently refreshed |
| C2 · first | Close-range obstacle coverage | Floor/self returns become obstacles; aggressive floor removal hides low obstacles | Local quality: separate upper/lower geometry and zone validity, using measured transforms | With installed mount, compare empty floor, wall and a known low obstacle in each sensor, alone and together | Correct floor/wall placement, retained low obstacle, explicit invalid/unknown zones; rate and interaction effects reported |
| C3 · next | Rapid detection of person motion | Static jitter becomes false velocity; smoothing delays real onset | Separation + feedback: distinguish measurement quality, association, existence and motion state | Replay v3, Kalman-on-blobs and Kalman-on-person-depth on identical observations | Lower static false-motion rate without unacceptable recall/onset regression; use the existing experiment-plan gates |
| C4 · next | Person-specific prediction quality | Person filtering removes furniture and other collision hazards | Segmentation: person tracker and general obstacle occupancy have separate outputs | Empty furnished scene, standing/walking person, non-person obstacle | Person output improves while general occupancy continues to cover non-person obstacles |
| C5 · next | Conservative handling of uncertain tracks | Phantom predictions make doorways permanently untraversable | Separation in time: current occupancy, track existence and future occupancy remain distinct; predictions are transient | Replay occlusion, static object with false velocity, false detection and genuinely blocked doorway | False blockage duration falls without deleting real hazards; expired tracks do not imply observed free space |
| C6 · shadow STL | Anticipatory pedestrian clearance | Large uncertainty regions prevent progress; narrow regions miss people | Preliminary action + feedback: time-indexed trajectory regions and calibrated uncertainty | Score 0.5 s / 1 s predictions on crossing, stopping and held-out occlusion runs; monitor candidate trajectories | Empirical coverage, missed detections and association failures reported alongside clearance and unnecessary stops |
| C7 · before live control | Stronger predictive constraints | Solver latency or infeasibility defeats timely actuation | Segmentation + prior action: bounded computation, explicit assumption monitor and validated backup policy | Offline/shadow deadline, stale-input and infeasibility cases; characterize braking and command latency separately | Deadline/failure behavior, braking envelope and every command path audited; monitoring alone does not qualify enforcement |
| C8 · mapping branch | Continuous volumetric mapping | Tilting removes horizontal scan coverage; mapping adds compute contention | Separation in space/function: evaluate OAK RGB-D mapping with horizontal lidar | Same-route replay: EKF-placed clouds versus RGB-D SLAM with the same EKF input | Mapping gain plus fresh horizontal sensing and acceptable live resource cost; only one production map→odom owner |
| C9 · localization branch | Recovery from larger pose errors | A plausible wrong room match is accepted; confirmation delays recovery | Separation in time + feedback: retain competing hypotheses and verify on later diverse observations | Reuse diagnostic recovery; test mapped-room coverage, revisits and independent reference | Correct recovery and explicit ambiguity/loss, not merely a stable low-residual pose |

### Current C1/C2 status (2026-09-23)

- **C1 recording/replay trial passes; full qualification remains open:** trial05
  captured 359 scored RGB/depth pairs in 30s (~12Hz), all with pose/TF support,
  max skew 16.585ms, max gap 101.315ms, and estimated age p95 322.621ms. Both ToF
  streams ~15.1Hz; one lower sequence omission is preserved. Repeated audit
  matched exactly and all 359 pairs passed pixel-hash verification. Absolute
  SDK clock error, ToF exposure/boot uncertainty and quantitative registration
  remain unqualified. See [stage-2 results](../artifacts/c1-stage2-2026-09-23/RESULTS.md)
  and [recording contract](c1_recording_contract.md). 51 tests pass.
  Alignment update: omitted IMX2141080P crop in RGB intrinsics is fixed and
  hardware-verified. Raised-target top mismatch improved~40→3px median; earlier
  poses improve to6px at1m and1px at1.5m in corrected replay. Full registration
  gate remains unmet (residual errors/invalid boundaries/support occlusion).
  See [alignment results](../artifacts/c1-alignment-2026-09-23/RESULTS.md).
- **C2 operationally validated:** ceb32f2 records 60 s clear-floor testing with
  zero obstacle frames, 136/136 detections of a 3 cm block, and 10/10 successful
  stop approaches. This is recorded prior evidence, not a newly repeated trial.
  Matched single-versus-dual tests and the formal interaction report are deferred
  by the user until the apartment; they do not block starting C1 instrumentation.

## 2. Diagnostic “confusion” table

These are hypotheses to distinguish, not newly proven root causes. Do not label every unexpected stop a phantom.

| Observed symptom | Competing explanations | First evidence to inspect | Next row |
|---|---|---|---|
| Furniture appears to move while robot is parked | Depth/segmentation jitter; association switch; timing error | Raw depth support, track identity, acquisition age; compare three tracking arms | C1, C3 |
| Static scene moves only while robot moves | Pose/time mismatch; extrinsic error; odometry error | Transform observations at acquisition time; compare robot-relative and odom-frame motion | C1, C2, C9 |
| Empty floor is blocked after mount change | Wrong sensor pose; floor model; self-return; invalid zone converted to obstacle | Per-sensor unfiltered points, measured mount pose, validity and floor residual | C2 |
| Only dual-ToF operation degrades | Scheduling/USB load; optical interaction; power behavior | Single-versus-dual matched captures, read duration, validity, drops and loaded supply evidence | C1, C2 |
| Person disappears when filtering improves static scores | Over-aggressive rejection; failed association; occlusion | Recall, missing-estimate duration and raw observations, including near range | C3, C4 |
| Corridor stays blocked after person leaves | Stale observation; old predictions retained; real obstacle still present | Observation age and layer provenance; fresh evidence of free space | C1, C5 |
| STL reports safe but observed clearance is poor | Wrong timestamps/frames; underestimated uncertainty; missing person; sampled-time gap | Time-aligned independent reference and monitor assumptions | C1, C6, C7 |
| Planner never progresses near a doorway | Real blockage; uncertainty too wide; ghost occupancy; infeasible dynamics | Separate current from predicted occupancy, track coverage and infeasibility reason | C5, C6, C7 |
| Map doubles walls or jumps rooms | Timing/extrinsic drift; wrong registration; false loop closure; competing TF owners | TF ownership, mapped-area coverage, surveyed checkpoints and pre/post-loop maps | C1, C8, C9 |

For an actual statistical confusion matrix, annotate two separate tasks:

| Reference truth | Reported occupied | Reported free |
|---|---|---|
| Occupied | True positive | False negative: missed obstacle |
| Free | False positive: phantom occupancy | True negative |

Record **unknown/invalid** outputs separately; never force them into “free.” Then score static-versus-moving classification only for matched real objects, with missed/unmatched objects reported separately. A real chair assigned velocity is a motion error, not nonexistent occupancy. Use a declared spatial region, time tolerance and independent reference; no numerical counts are available from this planning review.

## 3. Queue after ToF testing

| Priority | Concrete deliverable | Completion gate / dependency |
|---|---|---|
| P0 · close the ToF task | Single/dual benchmark report, sensor identities, installed transforms, validity rules, transport/drop behavior | Bench evidence plus physical mount checks. If already completed elsewhere, link results instead of repeating captures. |
| P1 · make evidence trustworthy | Versioned synchronized RGB/depth/ToF/pose recording and deterministic replay contract | Short payload trial passes; observation age, clock conversion/reset behavior and RGB-depth pairing are explicit. Preserve the single camera owner. |
| P2 · resolve phantom mechanisms | Three-arm tracker comparison and occupancy provenance report | Separate false existence, false motion and stale prediction; evaluate recall, onset delay, range bins and held-out runs. |
| P3 · prepare STL alongside P2 | Offline/shadow monitor consuming the same replay, with reason-coded violations and assumption failures | Timestamped tracks, uncertainty regions, independent occupancy and candidate robot trajectories exist. Imperfect perception is acceptable for development when its failures remain visible. |
| P4 · evaluate predictive control | Robust MPC versus existing prediction-plus-CBF report | P2/P3 evidence supports the model; evaluate clearance, false blockage, progress, deadlines and failure behavior before live integration. |
| P5 · mapping/localization branch | RGB-D replay comparison and targeted recovery validation | Share P1 data infrastructure. Promote earlier only if task completion is demonstrably limited by map/localization quality; it does not fix person-track uncertainty. |

Do not default to another MLP architecture, higher tilt speed, global person-only collision filtering, or tuning away conservative stops. First identify which error class dominates. Keep the settled ToF boresight-depth model and lidar tilt sign; remeasure changed mount geometry without reopening unrelated settled constants.

## 4. STL preparation contract

The monitor should receive a common time base; candidate robot footprint trajectory; person identity, existence evidence, last observation time and prediction status; time-indexed uncertainty regions; independent general occupancy; and sensor/localization validity. Log data provenance with each violation so a failed predicate can be traced to C1–C7.

Initial specification sketches, with thresholds to be measured/frozen before evaluation:

- **Clearance:** over horizon H, robot footprint and each modeled person region remain separated by at least d_min at matching future times. Include footprint geometry and a justified allowance for between-sample motion.
- **Freshness:** required observation age stays below tau_age; stale/unknown inputs produce an explicit assumption failure, not a reassuring clearance score.
- **Response:** stale sensing, solver failure or infeasibility triggers the defined fallback within tau_response. Define and validate that policy before claiming enforcement.
- **Progress:** measure completion time and blockage duration; do not require eventual passage when a person can block the route indefinitely.

Report robustness/violations alongside reference clearance, intervention delay, false stops, completion, solver deadlines and uncertainty coverage. Distinguish nominal, bounded-uncertainty and probabilistic claims. An STL monitor is not a controller; stopping is not an unconditional guarantee against an approaching person, and finite-horizon coverage is not mission-wide safety probability.

## Evidence links

- [Existing experiment design and proposed numerical gates](triz_perception_mapping_experiment_plan.md)
- [Dual mount status and transforms](dual_tof_mount.md)
- [Dual ToF bench protocol and transport limitations](../scripts/teensy_tof_test/README.md)
- [USB receiver](../src/teensy_tof_serial.py), [OAK publisher](../src/oakd_ros_publisher.py)
- [Implemented diagnostic recovery](lidar_temporal_recovery.md), [continuous sweep validation](continuous_sweep_validation.md)
- Retained sessions: `~/.agent-memory/project-triz-experiment-direction.md` and `~/.agent-memory/project-stl-predictive-safety-plan.md`.

Update each row with a linked run/report, result, and next decision as evidence arrives. “Implemented,” “bench passed,” “shadow passed,” and “live qualified” are separate statuses.

C1 subpixel update (2026-09-23): [controlled board comparison](../artifacts/c1-subpixel-2026-09-23/RESULTS.md)
retains clean recording/replay but does not pass spatial qualification. Top median
edge improves4→1px; p95 bound remains6.5px; side coverage remains insufficient.
Interior depth error against1.5m reference increases+35→+54mm. Recommend returning
to off for repeatability check before fitting calibration or promoting subpixel.

A-B-A return completed: off/on/off interior medians1535/1554/1537mm, clean
120-pair return audit. Top median4/1/1px invalidates attribution of the apparent
edge improvement to subpixel alone. Subpixel OFF restored; no restart pending.
C1 spatial and clock qualification remain open.

C3 started (2026-09-23): [first-slice results](../artifacts/c3-2026-09-23/RESULTS.md).
Deterministic A/B/C replay harness on C1 datasets; v3 per-frame math split into
`VelocityEstimator._step` (behaviour-preserving, not yet deployed). Parked
no-person data only: a Kalman filter on v3 blobs (B) cuts 3–4 m false motion on
the board scenes (5.6–18.6 % -> 0–1.7 % >0.15 m/s) but only ~30 % in the
cluttered furniture scene. It also adds 1.8–3 m false motion and track churn:
blob segmentation, not filtering, looks dominant. Recall, onset and arm C are
unmeasured until person captures exist; v3 live-output reproduction is still unmet.

C3 evaluation (2026-09-25): [held-out results](../artifacts/c3-eval-2026-09-25/RESULTS.md).
27 parked person captures (r1 tuning, r2+r3 evaluation, one room). Arm C
(Kalman on person-box depth) beats v3 on held-out runs. Recall: crossing
69-89 % vs 3-46 %. Standing 3-4 m static >0.15 m/s: 3 % vs 51 %. Empty-room
false motion: 0 vs 165/min. Onset: 0.27 s vs 0.45 s. It fails only the
0.5-1.8 m static gate (1.7 % vs 0.5 %). v3's 1.5-4 m depth band merges people
with background, which filtering (B) cannot fix. New issues: C coasting
predictions after exit (C5) and live v3 blind 226 s after start during captures
(logging suspected). Decision: revise, not adopt. A held-out room is required.
