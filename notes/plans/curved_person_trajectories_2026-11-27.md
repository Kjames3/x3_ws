# Curved person trajectories in the Reality view (2026-11-27, overnight)

Branch `overnight/curved-person-trajectories-2026-11-27`. Display only: nothing
here feeds the CBF or Nav2. Nothing was run on the robot.

## Result

- **Shipped behind two flags, both off by default.** The server flag is
  `X3_PERSON_PATH=cv|ctrv|gru|off` and the GUI flag is `?curvedIntent=1`.
  With either one off, the Reality view draws exactly what it draws today.
- **Recommended live setting: `X3_PERSON_PATH=gru` with `?curvedIntent=1`.**
  This uses a new GRU, `config/traj_gru_person_paths.npz`, trained tonight on
  Livox and C3 tracks from the runs before 2026-11-10. On the held-out runs it
  beats a straight line on every source and subset. On the source the GUI
  actually shows (`fused`, lidar plus camera-only tracks) it passes all three
  rules declared below before any numbers were looked at. On lidar alone the
  run-to-run clause of rule 2 is borderline: the upper quartile of per-run
  change is +0.0 %.
- **The honest catch: nothing here can predict that a person is about to
  turn.** The GRU's gain is almost all along-track: it predicts slowing and
  stopping. On held-out turning windows outside circling runs, its bends go
  the right way 54 % of the time, and CTRV's 55 %. A history-based curve
  reliably points the right way only when someone is already in a sustained
  turn (93 % on yard-circle). The fan of alternative turns is the honest
  picture for everything else: on turning windows, the better fan arm cuts
  the 2 s endpoint error from 0.84 m to 0.65 m.
- **Map awareness (option c) was checked for feasibility, then dropped.** The
  straight-line path runs into a static obstacle in 30 % of turning windows,
  against 23 % of straight ones. That is too small a difference to bend paths
  on.

## Selection rule (written before any numbers were looked at)

The candidates are `cv` (today's straight line), `ctrv` (constant turn rate and
velocity, with the turn rate damped) and `gru`. A candidate becomes the
recommended `X3_PERSON_PATH` value only if all three of these hold on the
held-out REPORT runs:

1. On MOVING windows it is no worse than `cv`: FDE at 2 s within +3 % of cv,
   and ADE at 1 s within +3 %.
2. On TURNING windows it has the lowest FDE at 2 s, and the gain over cv is
   larger than the run-to-run spread. Circling runs are reported both
   included and excluded.
3. It is stable enough to show. The median frame-to-frame change of the 2 s
   endpoint on moving tracks is at most 1.5x cv's. A wagging ribbon is worse
   to look at than a straight one.

If nothing passes, the default stays `off` and these notes say why. Turn
damping, clamps and gates are tuned on the TUNE runs only.

Definitions:
- MOVING: ground-truth mean speed over the horizon > 0.3 m/s (the
  trajectory_baselines definition).
- TURNING: MOVING, and the ground-truth heading change over the horizon is
  more than 30 deg. Heading at t0 is the direction of the last 0.3 s of
  history displacement and heading at t0+h is the direction of the last 0.3 s
  of future displacement. Both segments must be faster than 0.3 m/s, so
  standing jitter cannot count as a turn.
- TURNING_XT (added later as a sensitivity check, see below): TURNING, and
  the 2 s endpoint is also more than 0.4 m off the t0 heading line.

Two things were added after the first REPORT look, and both were tuned on
TUNE only. One is path-shape smoothing (GRU time constant 0.15 s), because the
unsmoothed lidar-trained GRU failed rule 3 (wag 0.147 m against cv's 0.091,
1.62x). The other is a cubic-in-time fit of the GRU path, because the raw GRU
path wiggles step to step. Each choice and its TUNE cost is stated where it
is made.

## Data and protocol

- Tracks come from all 62 `lidar_persons.ndjson` recordings under
  `~/x3_ws/artifacts/person-lidar-2026-1*` (2026-10-30 to 2026-11-24). They
  are loaded the `trajectory_baselines.tracks_from_ndjson` way: `lidar` is the
  Livox tracker with coasting rows dropped, `c3` is measured camera tracks,
  and `fused` is lidar plus camera tracks with no lidar track within 0.5 m.
- The split is by date. TUNE is the 33 runs before 2026-11-10 (Oct 30 to
  Nov 9: circle, cross, walkup, goal-on, busy, group, apt-*). REPORT is the 29
  runs from Nov 20 to Nov 24 (yard-*, crowd-r1..4, seeded*-walk, left-check,
  and others). REPORT was never used for tuning or training.
- Scoring replays every track through the runtime class
  (`person_paths.PersonPaths`) at the tracker's own stamps, one instance per
  mode, and scores each prediction against the track's own future
  (interpolated, with no gap > 0.35 s in between). Only predictions where
  every mode had a full 1 s history are scored, so all modes see the same
  windows. Windows are stride-1 and overlap heavily, so the effective n is
  the track and run counts given with each table.
- The deployed OAK GRU (`config/traj_gru_deploy.npz`, labelled `gru` below)
  has never seen Livox or C3 tracks, so all of these runs are out-of-sample
  for it.
- Tool: `src/tools/perception/curved_path_eval.py`. Example:
  `python3 src/tools/perception/curved_path_eval.py ~/x3_ws/artifacts/person-lidar-2026-1*/* --source lidar`

## Numbers (REPORT runs, final configuration)

FDE and ADE are in metres. The percentage is the FDE change against cv. "Per
run" is the 25th, 50th and 75th percentile of the per-run FDE change, over
runs with at least 10 windows.

### lidar (Livox tracker): 29 runs, 18 526 scored predictions

| subset | n (tracks, runs) | cv ADE / FDE | ctrv | gru (OAK) | **gru_pp (new)** |
|---|---|---|---|---|---|
| moving @1 s | 2671 (40, 13) | 0.165 / 0.326 | 0.162 / 0.318 (-2.5 %) | 0.182 / 0.356 (+9.3 %) | **0.152 / 0.305 (-6.3 %)** |
| turning @1 s | 401 (24, 9) | 0.186 / 0.388 | 0.184 / 0.386 (-0.6 %) | 0.198 / 0.401 (+3.5 %) | **0.170 / 0.358 (-7.8 %)** |
| moving @2 s | 2655 (35, 12) | 0.377 / 0.844 | 0.366 / 0.817 (-3.2 %) | 0.407 / 0.894 (+5.9 %) | **0.353 / 0.802 (-5.0 %)**, per run -2.2 / -0.1 / +14.2 % |
| turning @2 s | 876 (23, 9) | 0.404 / 0.933 | 0.390 / 0.900 (-3.5 %), per run -3.3 / -0.4 / +2.1 % | 0.457 / 1.019 (+9.2 %) | **0.365 / 0.844 (-9.5 %)**, per run -12.9 / -5.0 / +0.0 % |
| turning_xt @2 s | 709 (20, 9) | 0.407 / 0.934 | 0.393 / 0.899 (-3.7 %) | 0.473 / 1.059 (+13.4 %) | **0.379 / 0.874 (-6.3 %)** |
| wag, 2 s endpoint (median / p90) | | 0.091 / 0.171 | 0.109 / 0.213 (1.20x) | 0.055 / 0.140 | **0.104 / 0.216 (1.14x)** |

Without the circle runs: moving @2 s gives ctrv -0.6 % and gru_pp -5.4 %.
Turning @2 s (457 windows, 20 tracks, 8 runs) gives ctrv -0.1 % and gru_pp
**-13.0 %**, with per-run change -14.3 / -6.9 / +1.8 %.

The existing `trajectory_baselines` ctrv (its two-half heading fit) is +12.3 %
on moving @2 s and +1.1 % on turning. Its wag is 0.278 m, so it is not usable
for display.

### fused (lidar plus camera-only tracks, closest to what the GUI draws)

| subset | n | cv FDE | ctrv | **gru_pp** |
|---|---|---|---|---|
| moving @1 s | 2732 (48 tracks, 17 runs) | 0.330 (ADE 0.168) | -2.4 % | **-6.7 % (ADE 0.154)** |
| moving @2 s | 2673 | 0.843 | -3.2 % | **-5.0 %** |
| turning @2 s | 878 (25, 10) | 0.934 | -3.5 % | **-9.6 %**, per run **-12.9 / -5.0 / -1.3 %** |
| wag | | 0.090 | 1.20x | **1.14x** |

### c3 (camera tracks; thin)

Moving @2 s has 374 windows (21 tracks, 8 runs): gru_pp -3.8 %, ctrv +0.3 %.
Turning @2 s has 62 windows (13 tracks, 5 runs, only 2 with 10 or more
windows): gru_pp -27.6 %, ctrv -0.6 %. C3 has only about 550 moving windows
in total. C3's cv wag median is 0.000, because consecutive ndjson rows repeat
the same C3 state, so the wag rule cannot be applied to C3.

### OAK v3 windows (traj-baselines-2026-11-08), held-out run walk-g-r7 only

The deployed OAK GRU was trained on every other run, so only this run is
out-of-sample: 529 windows. Moving @2 s, 361 windows: cv FDE 0.709, ctrv
0.715, gru 0.532 (-25 %). Turning @2 s, 33 windows: cv 1.017, ctrv 1.014, gru
0.640. The OAK GRU works on the camera tracks it was trained on and does not
transfer to Livox tracks (+5.9 % on lidar moving @2 s above). That is why a
new model was trained.

### Rule check

| rule | ctrv (lidar) | gru_pp (lidar) | gru_pp (fused) |
|---|---|---|---|
| 1. moving: FDE@2 s and ADE@1 s within +3 % | pass (-3.2 %, -1.8 %) | pass (-5.0 %, -7.9 %) | pass (-5.0 %, -8.3 %) |
| 2. lowest turning FDE@2 s, gain beyond the per-run spread | fail (per-run IQR crosses 0) | lowest; IQR upper quartile **+0.0 %** (borderline) | pass (IQR -12.9 / -5.0 / -1.3 %) |
| 3. wag <= 1.5x cv | pass (1.20x) | pass (1.14x) | pass (1.14x) |

## What the curve can and cannot show (diagnostics)

- **Recent turning does not predict the next 2 s, except in sustained
  circles.** These checks used lidar tracks, all runs, and moving windows.
  The correlation between the past turn rate (position chords over the last
  0.6 s) and the future turn rate is 0.14. The least-squares gain is 0.09, so
  the best constant-turn model keeps 9 % of the turn it sees. In circle runs
  the correlation is 0.34 to 0.50. C3 tracks show about 0.
- **When ctrv bends, how often is it the right way?** On TUNE runs outside
  circling it is 36 to 51 % across the gates tried, and a longer 1.5 to 2 s
  history or a two-half consistency check did not raise it above 54 %. On
  REPORT turning windows outside circling, ctrv bends more than 0.1 m in 38 %
  of windows and 55 % of those go the right way; gru_pp is 38 % and 54 %. On
  yard-circle both are 93 %.
- **The GRU's gain is along-track.** On REPORT non-circle turning windows the
  along-track error is cv 0.63 m, ctrv 0.63 m and gru_pp 0.48 m, while the
  cross-track error is 0.58, 0.58 and 0.57 m.
- The CTRV gate that came out of TUNE is fit 0.5 s, gate t > 2, |w| > 0.15
  rad/s, shrink 0.5, tau 0.7 s, clamp 1.2 rad/s. At most it bends w·0.5·0.7
  rad over 2 s, about 20 deg at 1 rad/s. Every looser setting was worse than
  cv on TUNE turning windows. The best grid point was still +0.3 %, because
  TUNE has almost no sustained circling.
- `report_turning_examples.png` shows eight REPORT turning windows with
  history, the true path, cv, ctrv and gru_pp, plus the gru_pp fan. In most of
  them the history gives no sign that the turn is coming.

## Map / structure awareness (option c): feasibility only

The test used static occupancy built from each run's `/scan` (cells hit in at
least 20 % of scans, in odom, with the `laser_link` yaw -3.057 rad taken from
the bag's `tf_static`) over 18 runs. It counted how often the straight 2 s cv
path passes within 0.3 m of a static cell:

| windows | n | cv path within 0.3 m of a static cell |
|---|---|---|
| non-circle turning | 695 | 30 % |
| non-circle straight | 2812 | 23 % |
| circle turning | 428 | 3 % |

A wall ahead hardly separates turns from straight walking, so a wall-bending
rule would bend about as many straight walkers as turners. It was not built.
Doorway attraction was not attempted.

## Implementation

- **`src/person_paths.py`** handles prediction and server glue.
  - Histories are odom positions keyed by (source, id), observed at each
    tracker's own stamp: lidar `stamp`, C3 `state_stamp_s`. Lidar `coasting`
    rows are drawn but not learned from.
  - Stale tracks are forgotten on the monotonic `annotate()` clock, because
    track stamps are wall clock.
  - Predictions are cached per observation. The 20 Hz telemetry loop re-reads
    10 to 15 Hz trackers, so the GRU runs once per tracker update, not once
    per loop. Smoothing is also applied once per observation.
  - Each track gets `path_rel` (16 points over 2 s as base-axes offsets
    `[dfwd, dleft]` from the person, at the robot's current yaw),
    `path_sigma` and `path_alts` (0 or 2 alternatives at ±0.6 rad/s, damped
    by tau 0.7). The fan shows when |fitted turn| > 0.4 rad/s or the
    turn-rate std error > 0.25. On TUNE that is about 35 % of moving people
    and 44 % of turning windows.
  - In GRU mode the forward pass is batched. The path is fit with a cubic in
    time; on TUNE that costs +0.5 % FDE@2 s and removes the step-to-step
    wiggle. The path shape is smoothed with a 0.15 s time constant; on TUNE
    that takes wag from 0.124 to 0.083 m, against cv's 0.081, at +3 %
    FDE@2 s.
  - A missing GRU model logs a warning and falls back to ctrv.
- **`src/server_x3.py`** reads `X3_PERSON_PATH` (and `X3_PERSON_PATH_MODEL` to
  override the model) at startup. In `broadcast_loop` it runs
  `annotate_telemetry` on `c3_tracks`, `lidar_persons` and
  `velocity_estimates`. v3 estimates get odom fields only internally; the
  `odom_*` fields are not added to the payload. Any exception disables the
  feature and is logged; the loop keeps running. With the flag off, the same
  objects pass through unchanged (tested).
- **Cost per telemetry tick**, measured on the laptop CPU with no caching:
  ctrv 0.19 / 0.73 / 1.49 ms and gru 0.51 / 1.65 / 2.81 ms for 1 / 4 / 8
  tracks. With the per-observation cache, about half of the ticks are a dict
  lookup. **Not measured on the Orin.** Check it live, since this loop has
  stalled `/cmd_vel` before.
- **`src/web/scene-view.js`** reads `?curvedIntent=1`. It copies
  `path_rel`, `path_sigma` and `path_alts` through `avatarEntries` (C3,
  lidar-held, and v3 when C3 is off) and anchors the offsets at each avatar:
  scene `x = avatar.x - dleft`, `z = avatar.z - dfwd`.
- **`src/web/reality-mode.js`** (intent block only) draws a curved entry as a
  centripetal Catmull-Rom through its points, three samples per span.
  - Width uses today's 0.22 to 0.55 m widening as a floor, wider when
    `sigma` is larger.
  - The tangent is held where a path stalls or steps back, so a predicted
    stop cannot fold the ribbon.
  - The amber/red conflict colour and the end ring are kept. The fan is two
    extra ribbons at 0.45 strength.
  - Without the flag, the straight trail is vertex-for-vertex the old one.
  - `GUI.html` cache-busts `reality-mode.js?v=15` and `scene-view.js?v=45`.
    Other agents editing these files will conflict on those two lines; keep
    the higher number.
- **`config/traj_gru_person_paths.npz`** was trained with
  `train_traj_gru.py --hidden 128 --layers 1 --nll --deploy` on 17 479 TUNE
  windows (8379 lidar and 9100 c3). Its 5-fold grouped cross-validation on
  TUNE gave moving FDE@2 s of cv 0.725 against GRU 0.659 (-9 %). A variant
  that added the OAK windows (-10 % on a mixed set) was not chosen, so the
  model stays matched to the sources the GUI draws.
- **Tests:** `tests/test_person_paths.py` has 15 tests. They cover the flag
  off, straight and left-turn walkers, base axes with the robot yawed 90 deg,
  coasting, stale tracks on the wall-clock/monotonic split, the v3 transform,
  GRU batching, model fallback, the fan and smoothing idempotence. The full
  suite gives 4 failed, 1110 passed, 8 errors, the same 4 + 8 as the baseline.

## Screenshots

These are in `artifacts/surroundings-2026-11-27/curved-intent/`. They come
from `src/tools/viewers/person_paths_demo.py`, a fake server_x3 run with the
robot yawed 0.6 rad in a 7.5 x 6.5 m room and three synthetic Livox people:
one circling at 0.7 rad/s, one crossing straight, one weaving. The real GUI
was rendered in headless Chrome.

- `flag_off_top.png`: today's straight trails.
- `ctrv_top.png`, `ctrv_chase.png`: full-length ribbons, mild bends, fans on
  the turners.
- `gru_top.png`, `gru_chase.png`: the GRU predicts the constant-speed demo
  walkers slowing down, so these ribbons are shorter than on real tracks.
- `report_turning_examples.png`: real held-out windows.

## Needs a live check

1. With `X3_PERSON_PATH=gru` and the GUI at `?curvedIntent=1`, walk past the
   robot, turn a corner and walk a circle. Check that the ribbons start at the
   feet and point the right way while the robot is yawed and while it is
   driving; the odom-to-base rotation is exercised in tests only. Check that
   ribbons do not flicker between fan and no fan.
2. Telemetry loop cost on the Orin with about 5 people: log
   `person_paths.stats` (`ms_last`, `ms_max`).
3. Compare `X3_PERSON_PATH=ctrv`. It draws longer ribbons that bend only on
   sustained turns, and it may read better even though it scores worse.
4. If GRU ribbons look too short for people walking steadily, the model is
   predicting slowdowns that are common in these recordings (stop-and-go,
   seated runs). Retraining on moving windows only is the next thing to try.
