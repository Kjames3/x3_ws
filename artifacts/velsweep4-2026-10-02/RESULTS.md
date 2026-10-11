# Velocity MLP sweep #4 results (2026-10-02, REAL-1)

Script `src/sweep_velocity_prow.py`; raw rows in `results.jsonl`, log in `sweep.log`.
All 20 configs finished, none failed. Plan: `notes/velocity_mlp_improvement_plan_2026-10-02.md`.

Seed spread (3 seeds each): moving RMSE ±0.003, SIC ±0.002–0.004, static share
> 0.15 m/s ±0.01 (Lite control) / ±0.002 (Pro W j10 deploy).

| Config | SIC | near mv RMSE | far mv RMSE | fast | static > 0.15 (Pro W) |
|---|---|---|---|---|---|
| lite_j00_native (≈v3 recipe) | 0.282 | 0.551 | 0.595 | 1.267 | 32% |
| lite_j10_native (Aug winner) | 0.282 | 0.545 | 0.592 | 1.261 | 17% |
| prow_j10_deploy | 0.283 | 0.529 | 0.580 | 1.231 | 2.2% |
| **prow_j20_deploy** | 0.284 | 0.529 | 0.579 | 1.233 | **1.1%** |
| prow_j10_deploy_zfeat | 0.288 | 0.523 | 0.578 | 1.228 | 1.7% |
| prow_j10_deploy_nll | 0.283 | 0.542 | 0.590 | 1.250 | 1.0% |

Findings:

1. Pro W-law jitter negatives are the big win: static objects above the CBF's
   0.15 m/s threshold fall from 32% to 2.2% (10%) or 1.1% (20%), with no accuracy
   cost. The August Lite-law jitter only reaches 17%, because its noise is too
   small for the Pro W under 2 m.
2. Training at deployed ranges cuts moving RMSE about 0.02 m/s (several times seed
   spread). Caveat: the test set is built the same way, so part is in-distribution.
3. The noise law alone, without jitter, changes nothing.
4. Range input: accuracy gain within ~1–2x seed spread and SIC gets worse. Not worth
   the estimator change.
5. NLL head: 95% coverage is 95–97% and no static window is confidently moving,
   but moving RMSE is 0.01–0.02 worse. Calibration is against synthetic noise only.
6. Every model still under-reads moving speed by 3–6% (ratio 0.94–0.97).

Recommended drop-in candidate: `prow_j20_deploy` (40 in / 2 out, same as v3).
Not yet replayed on real captures or load-tested on the robot's torch.

## Replay on real lab captures (2026-10-02)

`src/velsweep_replay.py`: each model as arm A of the C3 replay (deployed
`VelocityEstimator._step`, weights swapped), 18 runs (`c3-captures/*-r[23]`),
scored against the YOLO11x reference (not ground truth). Raw: `replay/`.

| Metric | v3 | prow_j20_deploy | prow_j10_deploy |
|---|---|---|---|
| Non-person outputs > 0.15 m/s, cross | 39.1% | 25.2% | 27.0% |
| Non-person outputs > 0.15 m/s, stand | 32.1% | 22.7% | 24.1% |
| Non-person outputs > 0.15 m/s, approach | 35.5% | 27.0% | 26.7% |
| Non-person outputs > 0.15 m/s, empty | 9.0% | 6.3% | 7.0% |
| Standing person > 0.15 m/s, 1.8–3 m | 7.4% | 2.5% | 3.8% |
| Standing person > 0.15 m/s, 3–4 m | 47.2% | 32.5% | 34.0% |
| Approach speed ratio, 0.5–1.8 m | 0.60 | 0.56 | 0.59 |
| Approach speed ratio, 1.8–3 m | 0.87 | 0.85 | 0.86 |
| Approach speed ratio, 3–4 m | 0.66 | 0.63 | 0.63 |
| Go delay (median of run medians) | 0.45 s | 0.65 s | 0.65 s |

Recall is identical for all three (same blobs, same tracker).

- False motion drops by roughly a quarter to a third, far less than the synthetic
  32% -> 1%. Real phantoms come from unstable blob segmentation, not pixel noise.
- Cost: walking speed reads a few points lower and a start registers 0.2 s later.
- `prow_j10_deploy` keeps almost all of the false-motion gain with less under-read
  than j20, so it is the better trade on real data.
- All three under-read walking badly (0.56–0.87) and miss crossings (recall 3–45%):
  that is the depth-blob input, which no weights fix. C3 arm C remains the answer
  for people.
