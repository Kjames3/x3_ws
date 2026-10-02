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
