# Velocity MLP improvement plan (2026-10-02)

Supersedes the 2026-08-30 plan where they overlap. History and earlier sweeps:
`notes/velocity_estimator_history.md`.

## Where things stand

- v3 is the deployed model. Its training noise came from the OAK-D Lite; the
  robot now carries the OAK-D Pro W (floor sd about 0.011·Z² with subpixel on).
- The estimator's largest error source is its input, not its weights: the single
  1.5–4 m depth band merges a person with the background behind them. For
  people, the C3 tracker (arm C) already beats v3 on recall, static false
  motion and onset.
- Three ideas from the backlog were never trained: Pro W noise, range as an
  input, and an uncertainty output. Static-jitter negatives were trained in
  August and won, but against the Lite noise law.

## Goal

A model that (a) reads walking speed without bias at 0.5–4 m on the Pro W,
(b) reports near-zero speed for static objects, and (c) states an uncertainty
that the STL monitor and MPC can use. Decide afterwards whether the MLP serves
people, or only non-person moving obstacles next to C3.

## Phase 1 — retrain for the Pro W (started 2026-10-02, REAL-1)

Script: `src/sweep_velocity_prow.py`. Runs in `~/x3_velsweep4/` on REAL-1,
results in `~/x3_velsweep4/out/results.jsonl`, log in `out/sweep.log`.

Axes:

| Axis | Values | Deployable without robot code change |
|---|---|---|
| `noise` | Lite table, Pro W 0.011·Z² | yes |
| `jitter` | 0, 10%, 20% static negatives | yes |
| `zdist` | native Thor-Magni ranges, re-placed into 0.5–4.5 m | yes |
| `zfeat` | first-frame range as a 41st input | no (`_build_window_features`) |
| `nll` | mean + log-variance output, Gaussian NLL | no (output unpacking) |

20 configs: the full noise × jitter(0, 10%) × zdist grid on the stock model (8),
two 20%-jitter runs, six model-change runs on Pro W data, and four seed repeats
so differences can be compared against run-to-run spread. The August sweep had
no seed repeats, so its "within noise" calls were judgement.

Fixed evaluation sets, the same for every model:

- `sic_held`: SIC Courtyard. Referee only.
- `prow_near` / `prow_far`: Thor-Magni test sequences, Pro W noise, deployed
  ranges, split at 2 m. The main accuracy number.
- `lite_native`: the August test set, for continuity.
- `fast`: time-compressed 1.6–4.0 m/s.
- `jit_lite` / `jit_prow`: synthetic static objects; share above 0.30 m/s and
  above 0.15 m/s (the CBF's injection threshold).
- For `nll` models: 95% region coverage and the share of static windows whose
  region excludes zero.

Pick rules:

1. A difference counts only if it exceeds the spread across the seed repeats.
2. Drop-in candidate: lowest `prow_near`/`prow_far` moving RMSE with ratio
   closest to 1.0, subject to `jit_prow` share above 0.15 m/s no worse than the
   Lite control.
3. `zfeat` and `nll` earn their robot changes only if they beat the best
   drop-in model by more than seed spread (`zfeat`) or reach 90–97% coverage
   (`nll`).

Limits of this phase: the Pro W noise law is a per-pixel floor measurement, not
a measured centroid noise, and the jitter generator is synthetic. Offline
numbers rank models; they do not prove behaviour on the robot.

## Phase 2 — replay on real captures (laptop, no robot needed)

1. Pull the top two or three models and run `src/score_velocity_models.py` over
   the existing feature logs against v3. It replays the full chain (scaler,
   clip, confidence, per-component acceleration clamp), so the model is the
   only variable.
2. Run the 27 parked C3 captures through `c3_replay.py` / `c3_score.py` with
   each candidate in arm A. Reference is YOLO11x, not ground truth.
3. Check that each TorchScript file loads under the robot's torch build before
   anything is copied there.

## Phase 3 — input and estimator changes

Ordered by value:

1. Feed the MLP positions from person boxes, pose or seg depth (what C3
   computes) in place of depth-blob centroids. Same 40 features, no retraining.
2. If `zfeat` or `nll` won in Phase 1, add the range feature and the variance
   output to the estimator behind the model's scaler JSON (`input_dim`,
   `out_dim` are recorded there), and extend `test_velocity_robustness.py`.
3. If the new model keeps static objects quiet, try `MIN_COHERENT_FRAMES` 4 → 2
   to return about 0.2 s of latency. Decide on a walking test.
4. Loosen the ±0.25 delta clip and the acceleration clamp only after 1–3; a
   model and its delta clip must ship together.
5. Update `c3_person_tracker` from the Lite's 0.0025·Z² to the Pro W law.

## Phase 4 — robot tests

1. Parked: 60 s empty scene and a taped-lane walk at known pace, v3 vs
   candidate, same session.
2. Moving robot past static furniture at 0.2 m/s. Ego-motion compensation has
   never been drive-tested; furniture should read about 0.
3. `ab_comparison_test.py --mode predictive` with a walker inside 1.8 m, Motion
   Lock within reach.
4. Mocap ground truth in CRIS or CISL for the capstone numbers.

## Decision after Phase 2

If no candidate beats C3 arm C for people on the same captures, keep C3 for
persons, restrict the MLP to non-person moving obstacles, and move the
uncertainty work to the Kalman filter.
