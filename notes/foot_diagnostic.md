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
It sends no commands. Evidence is NDJSON plus a summary under
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
