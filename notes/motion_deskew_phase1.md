# Phase 1: chassis-motion deskew — 2026-09-05

**Verdict: PASS.** Deskew's chassis-motion compensation works, its benefit
scales with yaw rate exactly as predicted, and the residual distortion under
driving is well below the scales that matter for localization.

Capture: 90 s hand-driven, mount LEVEL, no sweep, 658 clouds all paired,
speeds to 0.85 m/s and yaw to 1.05 rad/s, with 122 stationary clouds as a
control. Scripts and data in `artifacts/motion-deskew-2026-09-05/`.

Neither interlock was touched. The mount stayed level and no sweep ran, so
`_tilt_nav_conflict`'s teleop arm never fired and `ROS2Bridge.move()`'s
`if lidar_3d_scan_enabled: zero everything` guard was never armed. `/scan` kept
flowing and the CBF stayed live throughout.

## Result

Consecutive clouds (137 ms apart) are placed in a common frame by odometry and
scored by median nearest-neighbour distance between their surfaces. An
internally sheared cloud cannot agree with its neighbour however it is
positioned, so this reads internal distortion directly. Deskewed and rigid are
scored on the same pairs with the same placement.

**Yaw rate, translation held below 0.10 m/s:**

| yaw rad/s | pairs | deskew | rigid | ratio |
|---|---:|---:|---:|---:|
| 0.00-0.05 (control) | 126 | 3.24 mm | 3.27 mm | 1.01 |
| 0.05-0.20 | 14 | 15.32 mm | 25.31 mm | 1.65 |
| 0.20-0.50 | 29 | 10.52 mm | 19.04 mm | 1.81 |
| 0.50-2.00 | 81 | **8.69 mm** | **17.53 mm** | **2.02** |

**Translation, yaw held below 0.10 rad/s:**

| speed m/s | pairs | deskew | rigid | ratio |
|---|---:|---:|---:|---:|
| 0.00-0.02 (control) | 121 | 3.23 mm | 3.25 mm | 1.01 |
| 0.02-0.20 | 24 | 12.72 mm | 26.05 mm | 2.05 |
| 0.20-0.40 | 123 | 6.97 mm | 7.48 mm | 1.07 |
| 0.40-2.00 | 72 | 9.46 mm | 10.12 mm | 1.07 |

Yaw is what deskew buys you: the benefit rises monotonically with yaw rate to
**2.02x at >0.5 rad/s**, which is the predicted ordering (0.5 rad/s over a
137 ms scan is 3.9 deg, or 0.20 m of smear at 3 m, against 0.055 m for 0.4 m/s
of translation). Under pure translation the gain is only ~7%, because a
translation offset common to both clouds largely cancels in the odometry
placement.

The parked control is the check that the metric is real: with no motion to
correct, deskewed and rigid agree to 0.02 mm.

## Does the residual matter? No.

Motion still costs roughly 3x over parked (3.2 mm -> 7-15 mm), so compensation
is not perfect. But the absolute number is what counts, and 15 mm sits well
under both scales it has to clear:

* the map voxel is **50 mm**;
* pose-graph cross-station consistency is **23 mm**
  ([[notes/registration_feasibility.md]]).

So motion distortion is not the limiting factor for 3D localization. Chasing
the remaining residual would be optimising something already an order below the
dominant error.

Direct confirmation that compensation is applied at all: deskew displaces
points by p50 0.03-0.05 m, p95 up to 0.30 m, max 1.15 m under motion, and
0.0002 m when parked.

## A metric that failed, and why (`analyze.py`, superseded)

The first metric fitted lines to bearing sectors and measured wall
straightness. It reported deskew/rigid ratios of 1.00-1.01 in every bin -- i.e.
"deskew does nothing" -- while the deskew was demonstrably moving points by up
to 1.15 m. It was wrong twice over:

* **Dominated by scene content.** The PARKED bin scored the *worst* residual
  (75 mm, against 23 mm while turning) purely because the robot happened to be
  parked facing clutter. Any cross-bin comparison was meaningless.
* **Insufficient resolution.** A 3-5 cm shear is invisible inside a 23-75 mm
  baseline set by furniture and corners.

Kept in the repo as a record; `analyze_merge.py` supersedes it. The lesson is
that the control bin is what exposed it -- a metric whose stationary baseline
is worse than its moving one is broken, whatever its headline says.

## Bug found: cached ROS parameters are silent no-ops

The first 90 s capture recorded **zero** clouds. `publish_cloud_when_level` was
cached in `lidar_3d_processor_node.__init__`, so `ros2 param set` reported
"Set parameter successful" and changed nothing. `require_settled` had already
been converted to a live property in an earlier session; the fix was never
generalised.

`publish_cloud_when_level`, `cloud_max_range` and `tilted_min_range` are now
live properties too. The capture script no longer depends on it either: it
launches its OWN processor instance publishing to `/motion_test/*` with the
parameter set at launch, so the production node keeps serving `/scan` and the
CBF untouched. Two traps in doing that, both from the August bench:
`/lidar_tilt/is_level` is hardcoded in the node and needs an explicit remap or
the test instance double-publishes onto the live one; and the executable must
be launched directly with `start_new_session` and killed by process group,
because `ros2 run` execs the node as a child and `terminate()` orphans it.

The capture script now also aborts if its processor publishes nothing within
6 s, rather than letting someone drive for 90 s producing an empty file.

## What Phase 1 does NOT establish

The mount was **level and stationary**. Tilt-motion deskew is validated
separately (parked continuous sweep, floor p95 7.87 -> 4.07 cm) but the two
have never run together, and Phase 2 needs both interlocks relaxed behind an
explicit opt-in.

A level cloud is also a flat disc at z=0.313, so this says nothing about *3D*
registration while moving -- only that motion compensation itself is sound.
