# Map-prior dynamic detection — plan (2026-10-14)

Status: **Phase 0 half done (2026-10-16)**, site auto-select built, not yet run on the robot.

## Progress 2026-10-16

- `src/map_prior.py`: `MapPrior.label()` (static/new/unknown by wall distance),
  `ChangeDetector` (per-cell EMA, tau 3 s: NEW that stays put becomes LEARNED),
  `SiteIdentifier` (global match against every map in `config/site_maps.json`,
  accept only if the residual margin between sites is >= 1.30).
- `src/auto_initial_pose.py` brought over from the unmerged
  `worktree-auto-initial-pose` branch (matcher core only is used).
- `src/map_diff_replay.py`: Phase 0 replay. Results on the C3 lab captures
  (`artifacts/map-prior-2026-10-16/phase0_summary.json`):
  - Every bag recognised as **lab**, margin 2.1-2.2 (apartment scores ~0.6 vs 0.8).
  - Walls line up: per-scan global pose spread 1.7 cm / 0.35 deg. Raw NEW was
    still **~22%** in the empty room. That was furniture moved since RealLab2
    was mapped, not localization (`empty-r2_lab.png`).
  - With the learned layer, empty room NEW is **0.5% median, 1.0-1.5% p95**
    (d_tol 0.10) -> passes the < 2% bar. Walking runs: 1.1-2.5% median
    (person present), 2-5x the empty-room count.
  - Caveat: `pose path` in the summary is refine jitter, not motion.
- Still open for go/no-go: the phantom-track half (needs per-track supporting
  points from the tracker logs).
- Site auto-select: `server_x3.py --auto-nav2-map auto` identifies the site
  from the first scans, launches Nav2+AMCL on that map for the session and
  seeds AMCL with the matched pose. Refuses on an unclear match. Prerequisite: trustworthy localization health
(see the lidar localization recovery plan). Do not build this on top of
localization you can't trust: a pose error turns every wall into a "new" object.

## Idea

Given a prior occupancy map and a localized pose, a lidar return that lands in
mapped **free** space is new. Only new returns may become dynamic candidates.
Returns on mapped walls are static by definition, and that kills the wall-slice
phantom movers. Lidar also covers what the camera misses (e.g. the Roomba).

Role: a **gate and a candidate source** for the existing tracker. It does not
replace the camera/pose person path, which stays the source of identity,
heading and person radius for the CBF.

## Phase 0 — Offline feasibility (no robot changes)

Inputs: existing walking/crossing bags that have `/scan`, `/tf` and a matching
saved map (newApartment1, RealLab1, apartment3_test), plus the corresponding
velocity estimator logs.

1. Write `src/map_diff_replay.py`: read the bag (reuse the `bag_viewer.py`
   sqlite reader), transform `/scan` into the map frame with the localized pose,
   and label each point as follows:
   - `static`: within `d_tol` of an occupied cell (start at 0.10 m, run a distance transform once per map)
   - `new`: in a known-free cell beyond `d_tol`
   - `unknown`: in an unknown cell, or outside the map
   - Apply the de-skew from the RTAB-Map replay traps: the beam index runs
     against time.
2. Metrics per bag:
   - Fraction of points labelled `new` on an **empty-room** bag. This is the false-change floor, and it effectively measures localization quality.
   - For each tracker track: the share of its supporting points that are `new`. Do the known phantom tracks fall to around 0?
   - Do real people and the Roomba stay mostly `new`?
3. Sweep `d_tol` from 0.05 to 0.20 m and the yaw error tolerance. Record the curve.

**Go/no-go:** the empty room gives less than ~2% `new` points, and at least 80% of
phantom tracks would be removed without losing any real walker. If the empty
room is noisy, the fix is localization, not this filter.

## Phase 1 — Live gate (diagnostic only)

- Add a `MapPrior` helper in the server: load the map PGM/YAML once,
  precompute the distance transform, and label points in a vectorized numpy
  pass. That's ~3k beams at 7 Hz, which is trivial.
- **Health switch:** label only when localization is healthy (AMCL covariance
  or the scan-to-map residual, using the same signal as the auto-initial-pose
  residual ratio). Otherwise pass everything through as today.
- Log the label counts and draw `new` points in the GUI in a distinct colour.
  **The tracker and the CBF are not affected yet.**

## Phase 2 — Use it

- In the tracker, lidar clusters made of `static` points may not create or
  feed a moving track.
- `new` clusters with no camera match become "dynamic, unknown class" tracks
  (that covers the Roomba case) and get the dynamic-category CBF treatment.
- Camera person tracks are unchanged. The map label only adds supporting
  evidence.
- Stale map handling: a `new` cluster that stays still for N seconds is
  demoted to static, so moved furniture doesn't become a mover.

## Phase 3 — Evaluate (capstone figure)

Run A/B on the same replayed bags with the map prior off and on. Report:
phantom swerve rate, true-mover recall, and the go-delay for crossing walkers.
Then do one live walking run with each arm.

## Risks

- Localization error or the 4.85° lidar yaw offset appearing as false change. Check that the URDF yaw is applied in replay.
- Maps with no posegraph (GUI Save Map) may need re-saving after the furniture moves.
- Only works in mapped rooms; elsewhere it falls back to current behaviour.
- With a 3D lidar (e.g. Mid-360) the same idea works on a voxel map. Keep the
  `MapPrior` interface 2D/3D-agnostic: `label(points_map_frame) -> labels`.
