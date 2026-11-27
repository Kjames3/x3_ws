# Curved person trajectories in the Reality view (2026-11-27, overnight)

Branch `overnight/curved-person-trajectories-2026-11-27`. Display only: nothing
here feeds the CBF or Nav2. Nothing was run on the robot.

## Result

- **Shipped behind two flags, both off by default.** The server flag is
  `X3_PERSON_PATH=cv|ctrv|gru|off` and the GUI flag is `?curvedIntent=1`.
  With either one off, the Reality view draws exactly what it draws today.
- **Recommended for the live look: `X3_PERSON_PATH=gru` with
  `?curvedIntent=1`.** It uses a new GRU, `config/traj_gru_person_paths.npz`,
  trained tonight on Livox and C3 tracks from runs before 2026-11-10. On
  held-out Livox tracks it beats today's straight line:
  - FDE@2 s is -5.3 % on moving windows and -10.4 % on turning windows.
  - ADE@1 s is -8.5 % on moving windows.
  - Frame-to-frame wag is 1.09x cv's.
  - Rules 1 and 3 below pass. Rule 2 passes only under a per-run reading of
    "beyond the run-to-run spread", and that reading was fixed after looking
    (see the Rule check).
  - These REPORT runs were scored about five times while the method changed.
    Every change was tuned on TUNE, but REPORT is no longer a clean, look-once
    holdout.
- **Nothing here reliably predicts a turn that has not started.** Most of the
  GRU's gain is along-track: it predicts slowing and stopping.
  - On held-out turning windows outside circling, it bends more than 0.1 m in
    30 % of windows, and 68 % of those bend the right way. CTRV bends in 38 %,
    55 % right.
  - A curve points the right way reliably only during a turn that is already
    under way: yard-circle gives 97 % for the GRU and 93 % for CTRV.
  - Even there, both models under-bend. On a synthetic 0.71 rad/s circle CTRV
    turns 12 deg of the true 82 deg over 2 s, and the GRU does not curve at
    all.
  - **So expect mild bends on the robot, not paths that swing around corners.**
    When a turn is likely but its direction is not, the faint fan of
    alternatives is the honest display.
- **Front people (C3) are the least supported.**
  - The GUI draws the C3 avatar in front of the robot and hides the
    co-located lidar track.
  - C3 updates at 3 to 6 Hz. After dropping repeated states, the held-out C3
    evaluation has only 74 moving windows at 1 s and 50 at 2 s, too thin to
    say anything.
  - Replaying the REPORT ndjson through the runtime, only **34 %** of moving
    C3 display time has the 1 s history a model path needs. The rest is the
    straight fallback with no fan. For Livox tracks it is 86 %.
  - **Fix shipped:** a C3 track now borrows the path of the lidar track
    seeded from it (`borrow_lidar_paths`), which raises this to **48 %**.
  - **Knob, untested:** a 0.5 s gap tolerance for C3
    (`person_paths.SOURCE_GAP_S = {"c3": 0.5}`) would reach 68 %. It is off,
    because its accuracy was not scored.
  - Front people with no linked lidar track and a broken history stay
    straight.
- **Map awareness (option c) was checked for feasibility, then dropped.** The
  straight-line path runs into a static obstacle in 30 % of turning windows
  and 23 % of straight ones. That difference is too small to bend paths on.

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
- TURNING_XT (added later as a sensitivity check): TURNING, and the 2 s
  endpoint is also more than 0.4 m off the t0 heading line. The TURNING label
  is real, not jitter: its median cross-track offset at 2 s is 0.64 m, against
  0.25 m for moving non-turning windows.

## What changed after the first REPORT look (all tuned on TUNE)

1. **A new GRU trained on TUNE Livox and C3 tracks.** The OAK GRU did not
   transfer (below).
2. **Path-shape smoothing for the GRU, time constant 0.15 s.** The unsmoothed
   GRU wagged 1.62x cv on REPORT. 0.15 s is the smallest value whose TUNE wag
   is under 1.4x cv.
3. **A cubic-in-time fit of the GRU path.** The raw path wiggles step to step;
   the fit cost +0.5 % FDE@2 s on TUNE.
4. **De-repeated C3 rows in evaluation and training.**
   `lidar_persons.ndjson` writes the newest C3 state on every lidar tick, so
   about half the C3 rows repeat a state under a later stamp. The first model
   was trained on those stair-stepped C3 tracks. The shipped model is the
   retrain on de-repeated tracks; the runtime never saw the repeats, because
   it keys C3 on `state_stamp_s`.
5. **`gru_bend` tried and not adopted.** This is the GRU path plus CTRV's
   gated bend. On TUNE it was slightly worse than plain GRU: moving FDE@2 s
   0.611 against 0.600, turning 0.865 against 0.858. On REPORT it would have
   scored better, because of yard-circle: -13.1 % against -10.4 % on turning.
   It stays as `person_paths.GRU_BEND = False`, available for a live A/B.

## Data and protocol

- Tracks come from all 62 `lidar_persons.ndjson` recordings under
  `~/x3_ws/artifacts/person-lidar-2026-1*` (2026-10-30 to 2026-11-24). They
  are loaded the `trajectory_baselines.tracks_from_ndjson` way: `lidar` is the
  Livox tracker with coasting rows dropped, `c3` is measured camera tracks,
  and `fused` is lidar plus camera tracks with no lidar track within 0.5 m. In
  the evaluation tool, rows that repeat the previous state are then dropped.
- The split is by date. TUNE is the 33 runs before 2026-11-10 (Oct 30 to
  Nov 9). REPORT is the 29 runs from Nov 20 to Nov 24 (yard-*, crowd-r1..4,
  seeded*-walk, left-check, and others). REPORT was never used to fit or tune.
- Scoring replays every track through the runtime class
  (`person_paths.PersonPaths`) at the tracker's own stamps, one instance per
  mode, and scores each prediction against the track's own future
  (interpolated, with no gap > 0.35 s in between). Only predictions where
  every mode had a full 1 s history are scored, so all modes see the same
  windows. Windows are stride-1 and overlap heavily, so the effective n is
  the track and run counts given with each table.
- **`fused` is not a second confirmation.** It is lidar plus about 10 extra
  windows, and its numbers match lidar to within 0.3 %. It is also the
  opposite of the GUI's priority: `avatarEntries` draws C3 first and adds a
  lidar track only where no C3 avatar is within 0.7 m. The GUI's mix is lidar
  for side, rear and distant people and C3 in front. Only lidar is
  well-measured here.
- Tool: `src/tools/perception/curved_path_eval.py`. Example:
  `python3 src/tools/perception/curved_path_eval.py ~/x3_ws/artifacts/person-lidar-2026-1*/* --source lidar`

## Numbers: REPORT runs, final configuration, Livox (`lidar`) tracks

29 runs, 18 526 scored predictions. FDE and ADE are in metres. The percentage
is the FDE change against cv. "Per run" is the 25th, 50th and 75th percentile
of the per-run FDE change, over runs with at least 10 windows. `gru` is the
old OAK model and `gru_pp` is the new one.

| subset | n (tracks, runs) | cv ADE / FDE | ctrv | gru (OAK) | **gru_pp (shipped)** | gru_bend (not adopted) |
|---|---|---|---|---|---|---|
| moving @1 s | 2671 (40, 13) | 0.165 / 0.326 | 0.162 / 0.318 (-2.5 %) | 0.182 / 0.356 (+9.3 %) | **0.151 / 0.305 (-6.5 %)** | 0.150 / 0.300 (-7.9 %) |
| turning @1 s | 401 (24, 9) | 0.186 / 0.388 | -0.6 % | +3.5 % | **0.171 / 0.360 (-7.1 %)** | -8.3 % |
| moving @2 s | 2655 (35, 12) | 0.377 / 0.844 | 0.366 / 0.817 (-3.2 %), per run -1.4 / -0.3 / +1.2 | 0.407 / 0.894 (+5.9 %) | **0.352 / 0.799 (-5.3 %)**, per run -2.7 / -0.0 / +9.4 | 0.345 / 0.783 (-7.2 %) |
| turning @2 s | 876 (23, 9) | 0.404 / 0.933 | 0.390 / 0.900 (-3.5 %), per run -3.3 / -0.4 / +2.1 | 0.457 / 1.019 (+9.2 %) | **0.363 / 0.837 (-10.4 %)**, per run **-9.7 / -8.6 / -2.0** | 0.352 / 0.811 (-13.1 %) |
| turning_xt @2 s | 709 (20, 9) | 0.407 / 0.934 | -3.7 % | +13.4 % | **0.375 / 0.865 (-7.3 %)** | -10.4 % |
| wag, 2 s endpoint (median / p90) | | 0.091 / 0.171 | 0.109 / 0.213 (1.20x) | 0.055 / 0.140 | **0.099 / 0.209 (1.09x)** | 0.111 / 0.222 |

Without the circle runs:
- Moving @2 s: ctrv -0.6 % and gru_pp -5.2 % (per run -2.2 / +0.7 / +10.0).
- Turning @2 s, 457 windows (20 tracks, 8 runs): ctrv -0.1 % and gru_pp
  **-13.1 %**, per run -10.0 / -8.8 / +0.0.
- Turning_xt @2 s, 321 windows: gru_pp -7.7 %.

The old `trajectory_baselines` ctrv (its two-half heading fit) is +12.3 % on
moving @2 s. It wags 0.278 m, so it is not usable for display.

**Fan.** The two alternative bends show on 29 % of turning windows. Where they
show, picking the best of the main path and the two arms after the fact (an
oracle, not something the display can do) gives an FDE@2 s of 0.65 m against
0.85 m for the main path alone. That measures how well the fan covers where
people actually went, not a prediction.

**C3, de-repeated.** C3 is too thin to judge. Moving @1 s, 74 windows: cv
0.516, gru_pp 0.473 (-8.2 %), old OAK gru 0.454 (-12.0 %). Moving @2 s, 50
windows: gru_pp 0.921 against cv 0.922. Turning @2 s has 2 windows.

**OAK v3 windows, held-out run walk-g-r7** (the OAK GRU was trained on every
other run). Moving @2 s, 361 windows: cv 0.709, OAK gru 0.537, gru_pp **0.808
(+14 %)**. Turning @2 s, 33 windows: cv 1.017, OAK gru 0.641, gru_pp 0.843.
Each model wins only on its own trackers. The runtime therefore uses the OAK
model for v3 tracks (`source_models={"v3": traj_gru_deploy.npz}`), which the
GUI draws only when C3 is off.

### Rule check (lidar, final configuration)

| rule | ctrv | gru_pp |
|---|---|---|
| 1. moving: FDE@2 s and ADE@1 s within +3 % | pass (-3.2 %, -1.8 %) | pass (-5.3 %, -8.5 %) |
| 2. lowest turning FDE@2 s; gain beyond the per-run spread | fail (IQR crosses 0) | lowest of the TUNE-chosen candidates; upper per-run quartile -2.0 % (without circles +0.0 %) |
| 3. wag <= 1.5x cv | pass (1.20x) | pass (1.09x) |

**What "beyond the run-to-run spread" was taken to mean.** It was
operationalised as "the upper quartile of the per-run change is below 0". That
reading was chosen after the numbers were seen. With 7 runs, it is weak
evidence either way.

## What the curve can and cannot show (diagnostics)

- **Recent turning does not predict the next 2 s, except in sustained
  circles.** These checks used lidar tracks, all runs, and moving windows.
  The correlation between the past turn rate (position chords over the last
  0.6 s) and the future turn rate is 0.14. The least-squares gain is 0.09. In
  circle runs the correlation is 0.34 to 0.50. C3 tracks show about 0.
- **When CTRV bends, how often is it the right way?** On TUNE runs outside
  circling it is 36 to 51 % across the gates tried, and a longer 1.5 to 2 s
  history or a two-half consistency check did not raise it above 54 %.
- **Final models on REPORT turning windows:**
  - Outside circling, ctrv bends more than 0.1 m in 38 % of windows (55 %
    right); gru_pp bends in 30 % (68 % right).
  - On yard-circle, ctrv bends in 46 % (93 % right); gru_pp in 33 % (97 %
    right).
  - The gain is along-track. Outside circling the along-track error is cv
    0.63 m, ctrv 0.63 m and gru_pp 0.48 m, while the cross-track error is
    0.58, 0.58 and 0.57 m.
- **On a clean synthetic circle (1 m/s, r = 1.4 m, true heading change 82 deg
  over 2 s)** CTRV turns +12 deg. The GRU turns 0 deg on noise-free input and
  -21 deg with 3 cm noise, and predicts 0.9 to 1.6 m of travel instead of
  2 m.
  - **Why the GRU does this:** constant-speed synthetic walking is unlike the
    stop-and-go recordings it was trained on.
  - **Why CTRV under-bends:** its shrink 0.5 and tau 0.7 s were tuned on TUNE,
    which has only about 19 circling windows with full history (circle-r1).
    In the regime where the bend direction is right 93 % of the time, it
    bends about a seventh of the real turn.
  - A circling-aware gate would need circling data that is not in REPORT, for
    example yard-circle split by track for tuning. That was not done.
- The CTRV gate that came out of TUNE is fit 0.5 s, gate t > 2, |w| > 0.15
  rad/s, shrink 0.5, tau 0.7 s, clamp 1.2 rad/s. Every looser setting was
  worse than cv on TUNE turning windows.
- `report_turning_examples.png` shows eight REPORT turning windows with
  history, the true path, cv, ctrv and gru_pp, plus the gru_pp fan. In most of
  them the history gives no sign that the turn is coming.

## Exploratory: a sustained-turn tier (`SUSTAINED_ON`, off)

The goal is to draw a turn that is already under way at closer to its real
rate. The gate fires when the turn has held for 1.5 s:
- |w| > 0.3 rad/s and t > 3 on a 1.5 s heading fit;
- both halves of that window turn the same way, each faster than 0.3 rad/s.

When it fires, the path uses the full fitted rate with a 2 s decay instead of
shrink 0.5 and tau 0.7, so it bends at most 0.63·w·2 rad over 2 s. The values
are fixed, not tuned. **yard-circle, which is in REPORT, informed the design,**
so these numbers are reported separately from the rule table and the tier
ships off.

- **Firing outside circling:** 1.7 % of moving TUNE windows and 2.4 % of
  REPORT. The bend direction there is a coin flip, 43 % and 58 % right.
- **Firing on yard-circle:** 16 % of moving windows. The realised 2 s turn
  rate is 0.84x the fitted one (median), the same sign 100 % of the time.
- **TUNE cost** (in-sample for the GRU): ctrv FDE@2 s moving +0.4 %, turning
  +0.7 %; gru_pp moving +0.3 %, turning +0.9 %.
- **REPORT turning @2 s:** gru_pp -10.4 % against gru_sus **-13.1 %**, with
  per-run change -12.2 / -9.0 / -2.8 %. Without circles both are -13 %. On the
  yard-circle turning windows alone (419), derived from the two subsets, FDE
  is cv 0.856, gru_pp 0.799 and gru_sus 0.748 m.
- **Synthetic 0.71 rad/s circle:** CTRV turns 48 deg of the true 82 deg,
  against 12 deg without the tier.
- **To try it live,** set `person_paths.SUSTAINED_ON = True`. The demo takes
  `--sustained`.
- **Screenshots:** in `gru_sustained_top.png` and `gru_sustained_chase.png`
  the circling walker's ribbon visibly curves.

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
  - **Histories:** odom positions keyed by (source, id) at each tracker's own
    stamp: lidar `stamp`, C3 `state_stamp_s`.
    - Not learned from (they are the trackers' own extrapolation): lidar
      `coasting` rows and C3 `measured: false` rows.
    - v3 estimates have no stamp, so a re-read of an unchanged estimate is
      not a new observation.
    - Stale tracks are forgotten on the monotonic `annotate()` clock, because
      track stamps are wall clock.
  - **Which rows are learned from:** `learnable()` applies the same filter as
    the evaluation's loader. Lidar rows need confirmed, not coasting, and
    `lidar_age_s < 0.15`; C3 rows need `measured` not false.
  - **Per tick:** tracks slower than 0.15 m/s skip prediction entirely. A
    prediction is cached until its track has a new observation, so the 20 Hz
    telemetry loop runs the GRU once per tracker update.
  - **Output per track:**
    - `path_rel`: 16 points over 2 s, as base-axes offsets `[dfwd, dleft]`
      from the person at the robot's current yaw.
    - `path_sigma`: the spread at each point.
    - `path_model`: false when the path is the straight fallback.
    - All of these are plain Python floats. **server_x3 encodes the readout
      with orjson, which rejects numpy scalars.** The first version shipped
      numpy floats, which would have made `orjson_dumps(msg)` raise in
      `broadcast_loop` with the flag on. Caught in review; a test now runs the
      payload through orjson.
    - `path_alts`: 0 or 2 alternatives at ±0.6 rad/s, damped by tau 0.7. The
      fan shows when |fitted turn| > 0.4 rad/s or the turn-rate std error >
      0.25, on about 35 % of moving people on TUNE.
  - **GRU mode:** the forward pass is batched per model, with v3 tracks sent
    to the OAK GRU. The path gets the cubic fit and the 0.15 s shape
    smoothing. A missing model logs a warning and falls back to ctrv.
- **`src/server_x3.py`**
  - **Startup:** reads `X3_PERSON_PATH`, and `X3_PERSON_PATH_MODEL` to
    override the model.
  - **`broadcast_loop`:** runs `annotate_telemetry` on `c3_tracks`,
    `lidar_persons` and `velocity_estimates`.
    - v3 is annotated only when C3 is not running, because that is the only
      time the GUI draws v3 people.
    - Static blobs and `is_person: false` estimates get no path.
  - **C3 borrowing:** C3 tracks then borrow the path of their linked lidar
    track. The link is a lidar `camera_id` equal to the C3 id; side cameras
    are `left:N`, `right:N` and `rear:N` and never match.
  - **Failure handling:** any exception disables the feature and is logged;
    the loop keeps running.
  - **Flag off:** the same objects pass through unchanged (tested).
- **Cost per tick.** On the laptop CPU, before the cache and the slow-track
  skip: ctrv 0.19 / 0.73 / 1.49 ms and gru 0.51 / 1.65 / 2.81 ms for 1 / 4 / 8
  moving tracks. **Not measured on the Orin.** `person_paths.stats` keeps
  `ms_last` and `ms_max`. Check them live, since this loop has stalled
  `/cmd_vel` before.
- **`src/web/scene-view.js`** reads `?curvedIntent=1` and copies `path_*`
  through `avatarEntries`. It anchors the offsets at each avatar with
  `RealityMode.relToScene`: scene `x = avatar.x - dleft`,
  `z = avatar.z - dfwd`.
- **`src/web/reality-mode.js`** (intent block plus two exported pure helpers)
  draws a curved entry as a centripetal Catmull-Rom through its points, three
  samples per span.
  - Width uses today's 0.22 to 0.55 m widening as a floor, wider when
    `sigma` is larger.
  - The tangent is held where a path stalls or steps back, so a predicted
    stop does not fold the ribbon.
  - The amber/red conflict colour and the end ring are kept. The fan is two
    extra ribbons at 0.45 strength.
  - Without the flag, the straight trail is vertex-for-vertex the old one.
  - `GUI.html` cache-busts `reality-mode.js?v=15` and `scene-view.js?v=45`.
    Other agents editing these files will conflict on those two lines; keep
    the higher number.
- **Tests**
  - `tests/test_person_paths.py` has 25 tests (the extra one covers the
    sustained-turn gate): flag off; straight and
    left-turn walkers; base axes with the robot yawed 90 deg; coasting rows
    and C3 unmeasured rows; v3 re-reads, static and C3-active handling; stale
    tracks; GRU batching and per-source models; model fallback; the fan;
    smoothing idempotence; the slow-track skip; C3 borrowing; the
    `learnable` filter; the payload through orjson in all three modes.
  - `tests/web/reality_curved_intent_check.cjs` checks the offset-to-scene
    signs and spline fidelity: it passes through every control point, stays
    within 5 mm of a 1.5 m arc, keeps a straight line straight and stays
    bounded through a stop.
  - The full pytest suite gives 4 failed and 8 errors, the same as the
    baseline.

### Reproducing the model

```bash
S=/tmp/pp; mkdir -p $S
for s in lidar c3; do
  python3 src/tools/perception/curved_path_eval.py ~/x3_ws/artifacts/person-lidar-2026-1*/* \
      --source $s --export-tune $S/tune_$s.npz          # TUNE runs only (< 2026-11-10), de-repeated
done
python3 - <<'EOF'
import numpy as np
H, F, R, names = [], [], [], []
for tag in ('lidar', 'c3'):
    d = np.load('/tmp/pp/tune_%s.npz' % tag)
    H.append(d['hist']); F.append(d['fut']); R.append(d['run'] + len(names)); names += list(d['runs'])
np.savez_compressed('/tmp/pp/train.npz', hist=np.concatenate(H), fut=np.concatenate(F),
                    run=np.concatenate(R), runs=np.array(names), rate=10.0)
EOF
python3 src/tools/perception/train_traj_gru.py $S/train.npz --out $S/cv  --hidden 128 --layers 1 --nll           # 5-fold CV
python3 src/tools/perception/train_traj_gru.py $S/train.npz --out $S/dep --hidden 128 --layers 1 --nll --deploy
cp $S/dep/gru_deploy.npz config/traj_gru_person_paths.npz
```

The training set is 16 825 windows (8379 lidar and 8446 c3). Five-fold grouped
cross-validation on TUNE gave moving FDE@2 s of cv 0.720 against GRU 0.665
(-7.6 %).

## Screenshots

These are in `artifacts/surroundings-2026-11-27/curved-intent/`. They come
from `src/tools/viewers/person_paths_demo.py`, a fake server_x3 run with the
robot yawed 0.6 rad in a 7.5 x 6.5 m room and three synthetic Livox people:
one circling at 0.7 rad/s, one crossing straight, one weaving. The real GUI
was rendered in headless Chrome.

- `flag_off_top.png`: today's straight trails.
- `ctrv_top.png`, `ctrv_chase.png`: full-length ribbons, mild bends, fans on
  the turners.
- `gru_top.png`, `gru_chase.png`: shorter ribbons, because the GRU predicts
  the constant-speed synthetic walkers slowing down.
- `gru_sustained_top.png`, `gru_sustained_chase.png`: the same with the
  off-by-default sustained-turn tier, where the circling walker's ribbon
  curves.
- `report_turning_examples.png`: real held-out windows.

## Needs a live check

1. Set `X3_PERSON_PATH=gru` and open the GUI at `?curvedIntent=1`, then walk
   past the robot, turn a corner and walk a circle.
   - Ribbons start at the feet and point the right way while the robot is
     yawed and while it is driving. The odom-to-base rotation is exercised in
     tests only.
   - Fans do not flicker on and off.
   - Expect mild bends. A sharp turn is not anticipated.
2. Telemetry loop cost on the Orin with about 5 people: `person_paths.stats`
   `ms_last` and `ms_max`.
3. Compare `X3_PERSON_PATH=ctrv`, `GRU_BEND = True` and
   `SUSTAINED_ON = True` (walk a circle and round a corner). CTRV draws
   full-length ribbons that bend only on sustained turns, and may read better
   even though it scores worse.
4. Front (C3) people. In replay about half of moving C3 display time still
   falls back to straight. If it looks that way live, try
   `SOURCE_GAP_S = {"c3": 0.5}`.
5. If GRU ribbons look too short for steady walkers, the model is predicting
   the stop-and-go common in these recordings. Retraining on moving windows
   only is the next thing to try.
