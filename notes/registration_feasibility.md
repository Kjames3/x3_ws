# 3D registration feasibility — offline study, 2026-09-05

Does a deskewed sweep cloud carry enough structure to localize against a map?
Offline only; no hardware moved. Scripts and results in
`artifacts/registration-2026-09-05/` (`icp.py`, `study.py`, `viewpoint.py`,
`budget.py`, `basin.py` + matching `.json`). Source data is the parked 30 s /
45 deg/s continuous capture, deskewed, in `base_footprint`.

Point-to-plane ICP was written by hand (neither open3d nor small_gicp is
installed) mainly so the 6x6 information matrix is available: the question is
which DOF the sweep constrains, and the library APIs do not expose that.

## Verdict

**Feasible, and cheaper than expected — but the 3D-over-2D case is weaker than
the premise assumed.**

1. **A single cloud is enough.** One 0.142 s cloud (2970 points) registers to
   **~1.5 mm / 0.02 deg** against a 5 cm map. Accumulating a full sweep pass
   does not improve accuracy at all.
2. **Do not accumulate — you cannot afford to anyway.** 2970 points cost
   29 ms per registration on the laptop, 21% of the 139 ms cloud period.
   A full pass (42.7k points) costs 317 ms, i.e. **228% of the period**, and
   buys nothing. Registration should run per-cloud at 7.18 Hz.
3. **3D's advantage is convergence basin, not accuracy.** Whenever the 2D
   slice converges it is just as accurate (mm-level). What 3D buys is
   tolerance of a bad initial guess.

## Method, and how the first two attempts were wrong

Three bugs were found and fixed on the way; recording them because each one
produced a confident, wrong answer.

1. **Shared viewpoint.** Registering a cloud against a map built at the same
   parked pose converged from 1.5 m / 30 deg to sub-millimetre, with identical
   error at every perturbation. That measures self-consistency, not
   localization: with no viewpoint change nothing is occluded and a perfect
   global optimum exists. Fixed by synthesizing the query from a DISPLACED
   viewpoint with hidden-surface removal (nearest return per 0.35 deg angular
   bin), drawn from a disjoint half of the returns so query points are
   different physical measurements than map points.
2. **The 2D baseline was a straw man.** Estimating normals by 3D PCA over a
   10 cm slab returns the slab's own VERTICAL normal for every point, so
   point-to-plane measured only vertical displacement, which x/y/yaw cannot
   move. The baseline "failed" at 0.1 m. Fixed with 2D PCA for in-plane wall
   normals plus a `planar=True` solve over (yaw, tx, ty) — the 3 DOF a 2D
   matcher actually has; the 6x6 solve is singular for a planar problem.
3. **The 2D query slab was at the wrong height.** `visible_from` returns points
   in the sensor frame whose origin is the viewpoint at FLOOR level, so
   `|z| < slab` selected floor returns and matched them against a map slab
   0.31 m higher. Corrected to `|z - 0.313| < slab`.

Sanity checks that the harness can fail at all: random-garbage input fails
(1.08 m, 34 deg), and a 2.0 m / 40 deg perturbation at a 0.25 m correspondence
radius fails. Unperturbed RMSE is 6.8 mm, a believable sensor noise floor.

## Convergence basin (query = one cloud, 1.0 m correspondence radius)

| displacement | 3D | 2D slice |
|---|---|---|
| 0.2 m / 5 deg | 1.7 mm | 1.1 mm |
| 0.5 m / 10 deg | 1.2 mm | 3.7 mm |
| 0.8 m / 15 deg | 1.5 mm | 3.9 mm |
| 1.0 m / 20 deg | 1.6 mm | 2.7 mm |
| 1.2 m / 25 deg | 1.2 mm | **fails** (2.39 m) |
| 1.4 m / 30 deg | 1.5 mm | **fails** |
| 2.0 m / 40 deg | **fails** | fails |

3D tolerates roughly **1.4 m / 30 deg** against 2D's **1.0 m / 20 deg** — about
40% wider, not a different regime. The 2.0 m result flipped between runs, so
treat the 3D edge as "somewhat wider basin", not a hard number.

Condition number of the 3D information matrix stays between 8 and 17 across all
displacements, so no DOF is close to unobservable in this scene.

## Cost

| query | time | share of the 139 ms cloud period |
|---|---:|---:|
| one cloud (2970) | 29.4 ms | 21% |
| quarter pass (10.7k) | 89.6 ms | 64% |
| full pass (42.7k) | 316.9 ms | 228% |

Laptop figures. The Orin Nano will be roughly 3x slower on single-threaded
numpy/scipy, putting one cloud near **60-90 ms, or 45-65% of the period** —
tight but workable, and a C++ GICP would be far cheaper. Only the one-cloud
budget is viable, which happily is also the one that loses no accuracy.

## What this does NOT establish

Every capture on disk is **parked**, so there is no real displacement, no
odometry drift to fight, no motion distortion during the query, and no dynamic
objects. The displaced viewpoint is synthesized by occluding the same capture,
so the map can never contain geometry the original pose did not see, and holes
grow with displacement — which caps credible displacement around 1.5 m and
biases toward optimism.

It is also **one furnished apartment room**, which is a favourable scene: the
2D slice is feature-rich there. A synthetic "corridor" (a 2.2 m lane cut from
the same room) still let 2D work to 0.6 m, but that is not a real degeneracy
test — cutting a lane leaves the end walls and furniture in. The classic 2D
failure, a long featureless corridor, has not been tested and cannot be from
this data.

So: a failure here would have been conclusive, and there was none. Success is
necessary but not sufficient.

## Recommended next steps

1. **Capture while driving.** This is the single missing input. A slow manual
   drive with the sweep running, logging `/odom` and per-cloud stamps, would
   supply real displacement, real deskew residual under motion, and a
   ground-truth trajectory to score against. Everything else is blocked on it.
2. **Record per-cloud boundaries and stamps this time.** Both existing npz
   files concatenate points with no cloud index, which forced the tilt of every
   return to be reconstructed geometrically here and made "obstacle age"
   unanswerable in the virtual-scan study. One index array fixes both.
3. **Do not build this on accumulated sweeps.** Register per cloud.
4. **Revisit the motivation.** The measured 3D advantage over a 2D slice is a
   ~40% wider convergence basin at equal accuracy. That is real but modest, and
   on its own it is a thin case for replacing AMCL. The stronger arguments —
   corridors where the 2D slice is degenerate, overhangs, z/pitch observability
   — are exactly the ones this data cannot test.

---

# Real multi-pose data — 2026-09-05 drive capture

Five stations driven by hand around the apartment (`drive_capture.npz`, 951k
points, 345 clouds, ~68 per station), headings spanning ~300 deg. This is the
viewpoint change the parked study could not supply.

## Corrections to the parked study, and to my own first read of this data

1. **The parked/synthetic basin of 1.4 m / 30 deg was optimistic.** It was
   measured against a map built from the same pose, where nothing is occluded
   and a perfect optimum exists. Flagged as a risk at the time; now confirmed.
2. **Whole-cloud RMSE is the wrong metric across viewpoints.** It counts
   non-overlapping points matching to whatever surface is nearest. It read
   0.098 m and looked alarming; **inlier RMSE over the corresponding subset is
   0.032-0.052 m.** Use the inlier figure.
3. **My first overlap table was contaminated.** It placed stations by
   odometry and reported 6-20% overlap for stations 3 and 4. Odometry is off
   by up to 0.88 m here, so that understated overlap badly. Measured AFTER
   registration, real overlap is **71-90%** for most pairs and 34% for 3->4.
   Drift, not lack of overlap, was the larger effect.
4. **`real_study.py`'s leave-one-out results are contaminated** for the same
   reason: its map is four stations placed by drifted odometry, i.e. a smeared
   double image. Its "basin fails at 0.2 m" is a property of that blurred map,
   not of registration. The pairwise numbers are the trustworthy ones.

## What holds up

| measurement | value |
|---|---|
| intra-station self-consistency (1 cloud vs the station's other 67) | **0.008-0.036 m** |
| cross-station inlier RMSE, overlap >= 71% | **0.032-0.052 m** |
| cross-station inlier RMSE, overlap 34% | 0.082 m |
| registration vs odometry disagreement | **0.147-0.875 m**, growing with drive order |

**The sweep geometry is sound.** Every station is upright (floor z~0, ceiling
2.5-2.9 m), which independently re-confirms `tilt_direction=+1` on live driving
data, and a single sweep is self-consistent to 1-3 cm.

**Registration is correcting real odometry drift.** ICP moves the pose
0.15-0.88 m away from odometry and the surfaces then align to 3-5 cm. If
registration were the thing in error, the alignment would be worse, not better.
The disagreement grows monotonically with drive order, which is the signature
of accumulated drift. This is the positive result for the localization goal:
it is exactly the job a localizer exists to do.

**A 480-start search found nothing better.** For every well-overlapped pair the
best of 120 restarts is the SAME solution the odometry-initialised run found
(4->0: 0.1046 vs 0.1047 m, 0.000 m apart). So the odometry prior is adequate
and the solver is not the limit -- there is no better alignment being missed.

## The one genuine failure, and it is a nasty one

Pair 3->4, the lowest-overlap pair (34%), does not merely register poorly: the
multi-start found a solution **5.27 m and 154.7 deg away from the truth with a
LOWER cost** (0.163 vs 0.220 m). At low overlap the cost surface has a false
global minimum that fits better than the correct pose.

That is worse than a convergence failure. A better initialiser does not help; a
global optimiser would confidently choose the wrong answer. Any deployed
localizer needs an overlap or residual gate that REJECTS a match rather than
trusting the lowest cost, and this pair is the regression fixture for it.

## Where this leaves the goal

Registration against a 3D sweep map is viable and corrects drift the wheels and
IMU cannot. The blockers are no longer "is there enough structure":

1. **The map must not be built from raw odometry.** 0.15-0.88 m of drift over
   one room smears it. It needs a pose graph -- register stations to each
   other, optimise, then build. That is the actual next step and it is
   offline work on data already captured.
2. **Low-overlap matches must be rejected, not trusted.** See 3->4.
3. Still untested: registration while MOVING (all data is stop-and-go), and a
   geometrically degenerate space such as a corridor.
