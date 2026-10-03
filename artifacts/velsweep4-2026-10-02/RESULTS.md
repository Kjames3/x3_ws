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
