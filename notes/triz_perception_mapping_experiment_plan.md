# TRIZ perception and mapping experiments

Date: 2026-09-18. Status: planned; no implementation, installation, capture,
service change, or controller change performed by this planning task.

## Objective and reasoning

Test two separations of responsibility before further MLP training or faster
lidar sweeping:

1. A probabilistic person tracker separate from general collision occupancy.
2. RGB-D SLAM using the OAK while the lidar remains horizontal.

The first separates identity, measurement uncertainty, and motion estimation.
The second separates volumetric mapping from continuous horizontal obstacle
sensing. Neither is yet proven better on this robot.

The MLP receives centroid histories, which cannot uniquely distinguish object
motion from segmentation changes, association errors, and ego-pose/timing error.
Fast response versus noise rejection is the central tracking contradiction.
The existing person gate is deliberately disabled because its centroids also
serve collision avoidance: enabling it globally would discard other obstacles.

Continuous lidar sweeping already demonstrated 45 degrees/s and 7.18 clouds/s
while parked. Faster tilt cannot increase the lidar's measurement budget. The
remaining moving-mapping requirements include fresh obstacle sensing, chassis
motion compensation, observable registration, and loop closure. Complete sweeps
need not be the localization update unit. Preserve the existing sweep interlocks.

Do not compare the old zero-phantom result at a restricted range with later
4 m reporting. A recorded wide-range static result was mean/p95/max
0.043/0.103/0.180 m/s, all outside the 1.8 m predictive-control gate. These are
historical results, not a new measurement. The earlier missing-training-tail
explanation for walking under-read was superseded by the range-gate experiment.

RTAB-Map was previously proposed in project_lidar_3d_audit.md, B4. This plan
operationalizes an unevaluated alternative; it does not claim a new invention.

## Shared prerequisite: reproducible, synchronized evidence

- Inspect current robot source/configuration against the checkout before any
  later deployment. Preserve unrelated changes and existing service arguments.
- Record source revision plus dirty diff, model/scaler hashes, intrinsics,
  extrinsics, ground-plane settings, range gates, software versions, and rates.
- Extend the existing camera owner/ROS publisher; do not open the OAK from a
  second process. Current src/oakd_ros_publisher.py has no RGB image publisher.
  It stamps cached depth, mono, IMU, and detections with publication time.
- Expose paired RGB and aligned depth with acquisition timestamps, frame sequence
  identifiers, correct CameraInfo, and optical frames. Document device-to-ROS
  clock conversion, clock resets, image-depth pairing, and dropped frames.
  Publish receipt time separately for latency diagnostics. Never relabel cached
  detections as fresh evidence. Confirm actual deployment behavior first.
- Record RGB, depth, CameraInfo, /odom, /scan, /tf, /tf_static, timestamped
  detections, estimator outputs, and original v3 feature logs when available.
  Existing record_bag.sh needs extension; its default topics are insufficient.
  Historical /oak/detections bags are not ground truth.
- Measure RGB-depth skew and sensor-to-pose alignment. Initial target: p95
  RGB-depth skew <=20 ms, with justified bounds on clock error. Reject unpaired
  frames; do not mask a synchronization failure with a wider approximate-sync
  queue. Revisit the numerical target before scoring if sensor behavior requires it.
- Run a 20–30 s payload-level trial before long recordings. Inspect distinct
  frame IDs/content, depth units, intrinsics resolution, stamps, TF availability,
  observed rates, missing intervals, and bag replay. Require no unexplained
  >0.5 s gap in a scored interval; report coverage rather than silently deleting gaps.
- User starts all captures requiring walking or driving from their own SSH
  terminal. At implementation time provide one exact verified command per run,
  its duration, and start/stop indicators. No speculative runnable commands here.
- Keep tuning and evaluation recordings separate. Freeze configuration before
  evaluating; use three independent repetitions and one held-out room/route.
  Use runs, not correlated individual frames, as the unit of repeatability.

## Experiment 1: uncertainty-aware person tracking versus v3

### Hypothesis

Stable person measurements plus explicit state uncertainty reduce static false
motion without increasing motion-onset delay or losing people. Separating person
tracks from occupancy preserves non-person collision coverage.

### Implementation sequence

1. Build deterministic offline replay from raw synchronized observations; the
   existing score_velocity_models.py only replays MLP feature windows and cannot
   evaluate a changed detector/tracker by itself. Reproduce recorded v3 outputs
   through the full gating/scaling/clamping chain before trusting comparisons.
2. Produce timestamped person boxes or masks using the same frozen detector in
   all candidate arms. Start with robust torso-region depth and outlier rejection;
   record valid-depth fraction and spread. Boxes alone can include background.
3. Estimate planar state [x, y, vx, vy] using a variable-dt Kalman filter in the
   continuous odom frame. Interpolate robot pose at acquisition time. Include
   range/depth quality and ego-pose uncertainty in measurement covariance.
4. Associate using predicted state, Mahalanobis gating and global assignment.
   Define track birth, confirmation, missed-observation handling and expiry.
   Emit identity, position, velocity, covariance, observation age and measured
   versus predicted status. Handle timestamp resets and stale pose explicitly.
5. Keep output diagnostic-only. General obstacle occupancy remains independent;
   do not connect candidate velocities to the CBF during this experiment.
6. Only after the constant-velocity baseline is scored, consider stopped/walking/
   turning IMM models or a learned trajectory head. Lidar fusion is a later arm,
   not an additional uncontrolled change in this experiment.

### Comparison arms

A. Existing v3 pipeline, unchanged, with its actual configuration documented.
B. Kalman tracker on the existing depth-blob measurements: isolates filtering
   and association improvements from improved person measurements.
C. Kalman tracker on person-supported depth: measures the full proposed system.

Replay identical raw frames and poses. Associate baseline outputs to annotated
people for person metrics, but also report every unmatched moving output as a
false-motion candidate. Do not improve a score by dropping inconvenient tracks.
Evaluate 0.5–1.8 m, 1.8–3 m and 3–4 m separately; include missing estimates.

### Capture matrix and reference

- Parked robot, empty furnished scene: 3 x 120 s.
- Parked robot, standing person at near/mid/far range: 3 x 60 s per range.
- Marked straight walks: approach/recede, cross view, start/stop and turn;
  3 x 60–90 s per scenario, keeping the person visible.
- Brief occlusion and two-person crossing: 3 x 60 s each.
- After parked validation: slow manual robot motion past stationary furniture
  and a standing person, followed by a walking person; 3 x 60 s each.

Use independently recorded, synchronized overhead/side video with surveyed
floor markers to annotate person ground position and crossing times. Document
occlusion and reference uncertainty. Tape-distance/time gives segment-average
speed only, not per-frame velocity or trajectory truth. If only that reference
is available, restrict claims accordingly. For moving-robot cases, obtain an
independent robot trajectory or score relative quantities; EKF odometry is not
independent world-frame ground truth.

### Metrics and proposed decision gates

These are initial engineering targets, not measured capabilities. Freeze them
before held-out evaluation; change only with a documented reason.

- Static speed p50/p95/max and fraction >0.15 and >0.30 m/s, per range bin.
- Person position error, segment-speed error, recall, ID switches, fragmentation,
  false confirmed person tracks/minute, and time without an estimate.
- Real motion onset/stop delay, measurement age, processing p95, CPU/RAM.
- Short-horizon predictions at 0.5 and 1 s: displacement error and empirical
  coverage of predicted uncertainty regions; do not treat intention as known.
- Candidate C target: >=50% fewer static >0.15 m/s outputs where A has enough
  events to compare; otherwise extend the static test and report an absolute
  false-event rate. No more than 5 percentage points loss in person recall,
  no >100 ms increase in onset delay, and no material position/speed regression
  (initial margin 10%, interpreted against reference uncertainty).
- Require the tradeoff to hold across repeated runs and the held-out room.
  Report failures individually; a lower global mean cannot hide loss at range.

Deliver: replay command, raw-data manifest, per-arm configurations, track CSVs,
plots, annotated failures, resource profile and adopt/revise/reject report.
Passing permits a later separately tested controller integration, not automatic
replacement of the existing collision behavior.

## Experiment 2: RGB-D SLAM with horizontal lidar

### Hypothesis

OAK RGB-D observations can support a consistent 3D map and revisits while the
horizontal lidar continues delivering fresh navigation/obstacle measurements.
This avoids requiring tilt speed to solve both sensing roles simultaneously.

### Implementation sequence

1. Check available ROS Humble/JetPack-compatible RTAB-Map packages and interfaces
   against official documentation at implementation time. Start on laptop replay;
   do not assume compatibility or Jetson real-time performance.
2. Use the synchronized RGB/depth/CameraInfo contract above and audited camera
   extrinsics. Keep tilt at its calibrated horizontal position throughout.
3. Establish baseline A: RGB-D clouds placed using unchanged wheel/IMU EKF poses,
   with no loop closure. This isolates gains from SLAM versus simply adding depth.
4. Candidate B: RTAB-Map RGB-D mapping with the SAME external EKF odometry, loop
   closure enabled. Test RGB-D visual odometry as a separate candidate C only
   after B works, avoiding simultaneous changes to pose input and mapping.
5. Run in an isolated replay/domain or namespace with TF remapping. Only one
   system may own production map->odom. Neither RTAB-Map nor replay may publish
   commands or compete with AMCL/slam_toolbox. Audit TF ownership explicitly.
6. Keep resolution, depth range, frame selection and input data matched across
   arms. If a horizontal lidar constraint is added to RTAB-Map later, label it
   as a separate fusion ablation rather than attributing the gain to RGB-D alone.
7. After offline scoring, run a live diagnostic Jetson session to measure latency
   and contention. Preserve /scan freshness and existing controller configuration.

### Capture matrix and reference

- Empty/static room, closed route returning to a surveyed start pose: three runs.
- Through a doorway into a second room and back: three runs, including different
  view directions on the return.
- Low-texture or dim section: three runs to expose tracking failures.
- Repeat one route with a person crossing: three runs to quantify dynamic ghosts.
- Hold out a second route/room from parameter tuning.

Record stops at independently surveyed robot poses and measure representative
wall separations, doorway widths and heights/overhangs. Use external tracking
where available. Sparse surveyed checkpoints support checkpoint error, not full
trajectory ATE. Reserve any AprilTags used for evaluation from the estimator's
inputs; alternatively document their use and obtain separate evaluation truth.

### Metrics and proposed decision gates

- Checkpoint translation/yaw error before and after loop closure; full ATE/RPE
  only when an independent continuous reference exists.
- Surveyed dimension error, duplicated-wall separation, geometric completeness,
  false loop closures, tracking loss/recovery time and dynamic ghost persistence.
- Record maps before/after revisit to verify that historical geometry benefits
  from corrected poses. A visually pleasing final view alone is insufficient.
- Resource cost, sensor-to-pose latency, backlog, /scan delivery and controller
  timing compared with a matched baseline. Host receipt timing and acquisition
  timing must be reported separately.
- Initial target: >=30% lower checkpoint error or duplicated-wall separation
  than A where baseline error exceeds reference uncertainty; no material
  regression in measured dimensions/completeness; zero observed false loop
  closures on this small suite (not a universal reliability claim).
- Live target: sustain >=5 pose updates/s, p95 sensor-to-pose latency <=250 ms,
  no growing backlog over 10 minutes, and no new >0.5 s /scan delivery gaps.
  Observe controller deadlines and watchdog behavior; no candidate may cause a
  stop or timing regression. Compare against matched idle and moving baselines.
- If replay quality passes but Jetson timing fails, report offline feasibility
  only and profile before reducing quality or claiming deployment readiness.

Deliver: reproducible isolated launch/replay configuration, maps and trajectories
for each arm, reference measurements, failures, resource profile, and a decision
on live adoption. Any eventual navigation integration is a subsequent task.

## Order and stop conditions

Shared recording contract -> short trial -> Experiment 1 parked replay ->
Experiment 1 held-out/moving tests -> Experiment 2 replay -> Jetson diagnostic.
Existing suitable bags can shorten this order only if payload checks pass.

Stop and repair evidence collection if timestamps, image alignment, TF or
reference quality are inadequate. A failed hypothesis is a useful result:
retain the data and explain whether sensing, association, pose, geometry or
compute dominated. Do not tune indefinitely on the held-out set.

## References

- src/velocity_estimator.py; src/score_velocity_models.py
- src/oakd_ros_publisher.py; record_bag.sh
- notes/continuous_sweep_validation.md; notes/registration_feasibility.md
- Shared memory: project_velocity_model_comparison.md,
  project_lidar_3d_audit.md, feedback_user_runs_hardware_captures.md
- Tracking baseline: https://arxiv.org/abs/2008.08063
- Uncertainty-aware association: https://arxiv.org/abs/2001.05673
- RTAB-Map ROS integration: https://github.com/introlab/rtabmap_ros
- Continuous-time lidar alternative: https://arxiv.org/abs/2205.12595
