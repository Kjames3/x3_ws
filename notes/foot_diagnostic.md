# Foot avoidance diagnostic — 2026-10-01

This is a **shadow-only experiment**, not collision protection. It adds no motor
commands and does not modify production CBF obstacles, Stop, or motion lock.

The existing OAK interface supplies atomic detections/metadata, timestamp-matched
aligned depth (within 50 ms), and intrinsics. The diagnostic worker samples fresh
odometry, interpolates it to capture time, locates compact above-floor depth
components near confident ankles, and tracks each side in odom. Outputs are
transformed into the current robot frame. It uses calibrated lens height/pitch
from `config/camera_ground_plane.json`, not the legacy driver height constant.
No new DepthAI nodes or SDK migration is involved.

Missing depth, capture pose, ankle confidence, clipped crops, or old observations
produce no usable estimate. Data older than 650 ms is suppressed at both server
and GUI. A camera outage must not freeze a suggested motion. The diagnostic has
its own thread; the telemetry loop only reads a small cached snapshot.

## What the GUI shows

The Surroundings **Foot diagnostic — motors unchanged** checkbox controls display.
Magenta rings show provisional ankle-adjacent occupied regions plus uncertainty.
Because COCO has ankles but no toes, extent includes a **minimum assumed 0.16 m
shoe radius**; it is not a measured shoe outline or a guaranteed enclosure.
The magenta robot-origin arrow is what a separate moving-obstacle CBF *would*
suggest from a stationary nominal command. Its length is 2 s of suggested travel
with a 0.35 m visual minimum. The status gives the suggested speed in m/s.

The shadow constraint is `-2 o·u + 2 o·v_foot + (|o|² - R²) >= 0`, with
`R = 0.30 m robot radius + provisional foot radius + 2 sigma` and a 0.15 m/s
Euclidean speed cap. Foot velocity needs three associated observations before
suggestions appear. Infeasibility is explicit, not reported as a safe stop.

**This calculation considers only feet.** It does not check walls, ToF/lidar,
actuator deadband, acceleration, rotation, or feasible retreat corridors. Its
uncertainty growth is a heuristic, not a statistically calibrated bound.
Association is side-specific nearest assignment, not guaranteed person identity;
close/crossing people and occluding furniture can confuse it. Height/floor tests
need calibration validation. It deliberately clears unobserved feet rather than
representing unknown occupied space; a control implementation needs a separate
policy for that loss. The 650 ms diagnostic age cap is not a validated control
latency budget. No accuracy or safe-avoidance claim is made.

## Run a short user-timed test

After deployment, restart `x3_server`, hard-refresh the GUI, and keep the robot
stationary with motion disabled. Start in view with ankle pixels away from image
edges. First check rings align with the appropriate feet. Then slowly extend and
withdraw one foot; compare its motion with the “would move” arrow.

From the laptop workspace:

```bash
python3 scripts/foot_diagnostic_capture.py --host x3 --seconds 20 --label first-foot-extension
```

The tool prints a three-second countdown, START, then STOP and packet counts.
It sends only read-only clock probes, never movement commands. Evidence is NDJSON plus a summary under
`evidence/foot-diagnostic/`. No regions in the summary means inspect rejection
reasons before attempting a longer capture. This logs outputs for review; it does
not save RGB/depth or provide ground-truth foot positions.

Disable the worker with `--no-foot-diagnostic` if comparing server performance.

## Validation and next boundary

`python3 -m pytest -q tests/test_foot_diagnostic.py` covers synthetic floor/foot
separation, invalid/clipped data, independent limb velocity, ego-motion,
capture-pose interpolation, expiry, moving-foot CBF direction, infeasibility,
threaded timestamped processing, and absence of control-path integration.
A browser-injected synthetic packet checks the GUI separately.

Live foot localization remains pending until the restarted server supplies actual
foot/depth pairs. Existing cached pose replay outputs located for this task have
boxes but no keypoints and cannot prove this measurement path. Compare this
first diagnostic against the physical scene before any motor-enabled work.

## Stage timing (instrumentation pass)

Each diagnostic packet now includes `timing`, `session_id`, and `settings`.
No detection thresholds, association gates, stale cutoff, or CBF limits changed.
The current completed decode is measured before the foot worker consumes it.

- `camera_delivery_ms`: estimated capture to host packet receipt. This combines
  on-device inference, camera/device queues, USB transfer and host queue polling;
  it does **not** isolate pure neural-network inference time.
- `host_predecode_ms`, `host_decode_wall_ms` / `host_decode_cpu_ms`: receipt to
  decode start, then host decoding and localization. SDK device-to-host clock
  conversion has no calibrated hard error bound.
- `diagnostic_wait_ms`: completed decode to diagnostic work start (poll/scheduling).
- `pose_lookup`, `depth_lookup`, `extraction`, `tracking`, `shadow_cbf`, and
  `result_build`, each with `_wall_ms` and `_cpu_ms`. Wall includes scheduling
  and GIL waits; CPU is only the calling thread, not BLAS/OpenCV helper threads.
  A large wall/CPU difference is a lead to investigate, not proof of a GIL issue.
- `work_wall_ms`, `work_cpu_ms`, `capture_to_ready_ms`: aggregate work and age.
- `ready_to_snapshot_ms`, `capture_to_snapshot_ms`: completed result waiting for
  broadcast-loop consumption, and age at that point. Repeated snapshots are kept.

Readouts also carry `telemetry_timing`: frame sequence, a monotonic timestamp
before encoding, and encoding/broadcast-call durations for the **previous** frame
(explicitly identified by its own sequence). This avoids serializing twice.
The broadcast duration measures enqueue work, not network delivery completion.

The capture script sends read-only `diagnostic_clock_ping` probes before/after a
run. Four timestamps bound the server-minus-laptop monotonic clock offset; the
lowest-roundtrip-width sample supplies an estimate and ± bound. The output logs
`prepare_to_client_estimate_ms` (encoding + server/network/client queues + transport)
and `capture_to_client_estimate_ms`. These exclude browser rendering. The bound
covers the host clock probe assuming nonnegative path delays, not device timestamp
error or arbitrary clock drift; before/after consistency is included in `clock.json`.
An older server without the probe still records data but has no delivery estimate.
Negative estimates are retained rather than concealing synchronization uncertainty.

`summary.json` reports per-stage median/p95/max, packet and telemetry intervals,
all-readout stale gaps, and camera settings (requested NN/depth rate, observed depth
rate, input size, model blob, RGB-D recording/subpixel, USB and threading environment).
Distinct packets are keyed by session + sequence. Repeated readouts do not overweight
processing times, but they do contribute snapshot ages and stale-gap reporting.

Run the same 20-second command above after restarting the updated server. The
script still prints countdown/START/STOP and never sends movement commands.
Timing instrumentation and a real local WebSocket exchange with synthetic data
are covered by `tests/test_foot_timing.py`; real sensor timing needs the next capture.

### Direct shadow solver (2026-10-02)

The shadow CBF now enumerates the minimum-norm candidate velocities for its
2-D half-plane constraints: zero, perpendicular boundary projections, and
pairwise boundary intersections. It accepts the shortest feasible candidate
within the existing 0.15 m/s speed disk. If the unconstrained polygon's closest
point lies outside the disk, the bounded problem is infeasible. No iterative
optimizer is needed. Candidate checks scale cubically with the number of ready
feet; this is not a hard real-time scheduling guarantee.

The barrier, uncertainty inflation, nominal stationary velocity, diagnostic-only
integration, and explicit infeasible result remain unchanged. Normalized
constraint and speed tolerances are 1e-10 m/s. Tests cover contradictory and
parallel constraints, degenerate normals, speed boundaries, 200 randomized
comparisons with SLSQP, and the recorded foot-timing capture (65 suggestions and
18 infeasible results reproduced). Live timing after restart remains to be
measured; this change does not address camera delivery latency.

### Camera host polling timing

The camera delivery measurement ends when the driver dequeues the pose packet,
not when USB first delivers it. The driver polls pose after depth, recording RGB,
and auxiliary processing. Additional `camera_*_ms` fields measure depth blocking,
depth work, RGB work, auxiliary work, NN dequeue, and the preceding pose poll
interval. They use host monotonic time and include scheduling delays. These
stages describe the loop that consumed the pose, not necessarily the loop that
received it. Do not subtract their sum from delivery and label the remainder
inference time: device preprocessing, inference, transport, and queue residence
are not individually timestamped. Samples cover loops that consumed a pose only.
Use `--label foot-camera-polling` after restarting to establish this baseline
before changing model or inference settings.

### 2026-10-02 latency experiments

The retained live configuration is yolo26n-pose-512 (384x512 portrait), four
SHAVEs, two inference threads; latest-only NN output remains enabled. Full-size
four-SHAVE / two-thread baseline (140618) versus smaller working model (143953):
camera delivery median 268.8 -> 142.8 ms, p95 312.6 -> 165.5 ms; capture to
ready median 313.8 -> 186.5 ms. Both had feet in every distinct packet and no
recorded gaps. Laptop age comparisons are approximate: baseline clock probes
were inconsistent; smaller model's initial bound was +/-10.2 ms.

The 143550 smaller-model run is INVALID for tracking: the host decoder assumed
6300 anchors and rejected the 4032-anchor output. The decoder now reads the
output count from config, with regression tests for both sizes. The 134223
polling run predates the server restart and lacks the new stage fields.

Four SHAVEs roughly doubled observed update throughput versus eight, but did
not reduce median delivery by itself (compiler versions also differ). Reducing
the output queue produced no material latency change. One inference thread
reduced delivery slightly but lost updates and did not improve overall freshness.
Smaller input also reduces aligned depth size; foot localization accuracy and
occlusion recovery are still unvalidated. No experiment enables foot actuation.

### Shared-depth ambiguity gate

Before tracking, opposite ankles of the same detection are withheld when their
selected component pixels overlap by at least 60% of BOTH components and their
horizontal centers are within 0.06 m. Comparison uses actual selected depth
pixels, not rectangle overlap. Both measurements count toward
`rejected.ambiguous_shared_depth`; neither receives an independent marker or
shadow constraint. No side is guessed from keypoint confidence. Single feet,
separate components in overlapping crops, and different person detections are
not rejected by this gate. Tracking resumes through normal warm-up when evidence
separates. This does not retain an unlabeled obstacle during ambiguity, and is
still diagnostic-only. Thresholds need live sideways/close-feet validation;
saved telemetry has no raw depth support with which to replay this gate.

### Foot smoothing experiment

Tracker smoothing is in odom/world coordinates at capture timestamps. Velocity
uses an exponential filter with 0.18 s time constant on consecutive RAW position
differences. Position uses a 0.10 s time constant reduced by
1 + |filtered velocity| / 0.15, so sustained movement receives less smoothing.
Coefficients use actual dt. Identity jumps and missing/ambiguous observations
reset the filter through normal track creation/loss. The distance between raw
and filtered position is added to sigma and exposed as smoothing_lag_m; this
is a lag allowance, not a statistically calibrated uncertainty bound.

Synthetic tests cover stationary alternating 1 cm noise, steady 0.2 m/s motion,
stop settling at 5/10/20 Hz, and dropout resets. These are not measured live
performance. First repeat foot-still-smoothed, then a separately labelled
slow-extension test before retaining or tuning coefficients. Smoothing does
not add persistence and does not feed the production CBF.

### Stability findings, 2026-10-02

Sideways right-foot-nearer capture 145538 reported both feet only 1.1 cm apart
(median); overlapping depth support made stable IDs misleading. Ambiguity gate
run 151238 rejected both in 187/190 packets; front-facing run 151439 retained
both in 172/172 without rejection. No independent foot truth was recorded.
Smoothed stationary run 191303 retained both in 193/193, p95 position dispersion
0.81/0.84 cm left/right versus 1.39/1.14 cm previously, but stance distance
changed by ~20 cm. Movement runs 191745 and 191819 kept both in 184/186 and
204/204 respectively, while IDs changed (four per side fast; one left and three
right slow). User alternated lifting feet, holding ~4–5 s in the slower run;
do not assume prescribed stationary time windows.

Two fast-run left-ID changes follow no_compact_component rejections. Both
slow-run right-ID changes show ~27 cm displacement over 83 ms, consistent with
the 3 m/s raw-speed rejection. Physical motion vs depth-surface switching is
unresolved without raw depth. Some intermediate packets are absent from the
capture; do not infer every reset reason exactly. Next: explicit raw-position
and association diagnostics, plus short identity-only retention, with fresh
velocity warm-up and no stale obstacle publication. Do not loosen gates yet.

### Identity-only retention and reset events

Unmatched identities survive privately for at most 0.35 s since their last
measurement, using the existing 0.30 m association and 3 m/s raw-speed gates.
Only currently measured tracks appear in local feet or shadow CBF inputs.
Reacquisition resets velocity to zero and requires three fresh observations
before velocity_ready. Unsynchronized input clears active and retained state.
No extension of physical obstacle persistence is implied.

Each packet now contains tracking_events with raw_world_xy in odom metres,
matched/reacquired/new/rejected/expired decisions, prior/new IDs and, where
applicable, gap_s, predicted residual_m and raw_speed_mps. Missing identities
are explicitly reported as retained. Capture scripts preserve these fields
without changes. Exact depth-surface attribution still requires raw depth.

### Retention correction after capture 193226

Association now runs active tracks first, then retained tracks for remaining
observations. Unmatched old identities are retired when their side is observed
again, with superseded_identity_retired events. This prevents an old identity
remaining available after a new same-side replacement has appeared. No distance,
speed, smoothing, warm-up or retention-duration thresholds were loosened.

Without persistent person IDs, this retirement rule is intentionally conservative
across people too: an occluded foot may lose its retained identity if another
person's same-side foot remains visible. Prefer loss of identity continuity to
an unjustified revival. Fresh observations are still processed normally.
Tests cover active-first matching, old-new-old replacement, and independent
opposite-side disappearance/recovery. Live validation is pending.

### Multi-layer ankle extraction experiment

Replaced the single 20th-percentile seed with seeds at 1/3/5/10/20/40/60/80
percentiles. Choose the nearest supported component; within 2 cm depth prefer
its centroid nearer the ankle. Components need at least 16 pixels and 1% of
valid crop support, with centroid within one crop radius of the ankle. The
radius allowance was expanded from 0.85 to 1 to include a raised shoe near the
crop edge. Existing height, spread and shared-support ambiguity gates remain.
No temporal depth hold or relaxed tracker limits were added.

Targeted replay of ten paired depth frames around five resets in foot-depth-r1,
using reconstructed integer ankle-crop centers and calibrated depth, reduced
forward-coordinate pair changes from roughly 0.306/0.278/0.298/0.256/0.320 m
to 0.009/0.029/0.112/0.038/0.026 m. This is an approximate paired-frame replay,
not an exact live depth/NN reconstruction or accuracy ground truth. Third pair
still moves significantly and needs live review. Original RGB/depth bag stays
on robot artifacts/c3-captures-segpose/foot-depth-r1. Additional component passes
increase extraction work; live timing must be checked. Synthetic tests cover
foreground occupancy changes, speckles, remote components, and distinct feet
at equal depth; all 30 diagnostic tests pass.

Live multi-layer capture 195849 retained the right foot's ID throughout and
both feet in 163/163 packets, with zero raw-speed rejections. One left-foot
association rejection remained (0.376 m). Extraction median increased from
17.8 to 43.1 ms; capture-to-ready median 187 to 215 ms, p95 237 to 299 ms.
The movements were not identical; optimize extraction without changing its
selection behavior before attributing every improvement to the algorithm.
