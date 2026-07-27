# Velocity Estimator — Self-Supervised Auto-Labeling & Offline Retrain Plan

Status: **DESIGN SKETCH** (not implemented). Created 2026-07-25.
Owner: TBD. Addresses the Astra→OAK-D domain gap for the velocity MLP.

---

## 1. Why

The velocity estimator (`src/velocity_estimator.py`) is a small TorchScript MLP that maps a
10-frame window of an obstacle's metric 3D centroid history → `(vx, vy)`. It was trained on
**Astra Pro** depth data, but the robot now infers on **OAK-D Lite** depth/spatial detections.
Different depth sensors ⇒ different noise profiles, effective frame rate, and centroid bias, so
the model is running in a domain it wasn't trained for.

The robust fix is **not** live weight updates (feedback-loop / catastrophic-forgetting risk on a
safety-relevant signal). Instead: **collect real OAK-D deployment data, auto-label it from the
future, retrain offline, gate on validation, and only then hot-swap the model.** This closes the
domain gap with real target-domain data and keeps a human/CI gate in the loop.

### Core trick: labels come for free from the future
The deployed model is **causal** — it predicts velocity from a *past* window ending at frame `t`.
Offline we can look at frames **after** `t` and compute the *true* velocity by finite-differencing
the object's world-frame position. No manual labeling. The only cost is a short latency in when a
sample becomes labelable (must wait `LABEL_HORIZON` frames).

---

## 2. Feature/label contract (MUST match serving exactly)

Train/serve skew is the #1 way these systems silently fail. The offline pipeline MUST reuse the
**exact** feature builder from serving, not a reimplementation.

From `velocity_estimator.py`:
- Input per obstacle: **40 features** = `WINDOW_SIZE(10) × [rel_x_norm, rel_y_norm, dx, dy]`
  - `rel_x = cz` (depth / robot-X forward), `rel_y = -cx` (robot-Y left)
  - translation-normalized to the first frame of the window (`rx0, ry0`)
  - `dx, dy` = per-frame displacement, **clamped ±0.25 m**
  - built by `VelocityEstimator._build_window_features(history_local)` → `(1, 40)`
- Output: `(vx, vy)` in the same rotated robot frame, m/s
- Scalers: `scaler_params.json` holds `scaler_X.{mean,scale}` and `scaler_y.{mean,scale}`
  (sklearn StandardScaler). Model consumes scaled X, emits scaled y; code inverse-transforms.
- Constants: `WINDOW_SIZE=10`, `INFER_HZ=10`, `MAX_OBSTACLES=5`, `MAX_RANGE_M=5.0`
- Artifacts: `MODEL_PATH = src/velocity_mlp.torchscript`, `SCALER_PARAMS_PATH = src/scaler_params.json`

**Action item:** refactor `_build_window_features` into an importable pure function (or copy it
verbatim into a shared `velocity_features.py`) so both the estimator and the trainer call the same
code. Any divergence here poisons training.

---

## 3. Ground-truth label: use the GLOBAL frame to remove ego-motion

`dx, dy` in the local frame conflate the robot's own motion with the object's motion. The tracker
already keeps `history_global` (world-frame centroids). Compute the **label from world-frame
positions**, then rotate into the model's target frame:

```
# For a window ending at frame t, look ahead LABEL_HORIZON frames.
# Central difference around t in WORLD frame is less biased than forward difference.
v_world = (pos_global[t + H] - pos_global[t - H]) / (t_time[t+H] - t_time[t-H])   # (vx_w, vy_w)
# Rotate world velocity into robot frame at t (theta from robot_pose_fn), then map to
# the model's (rel_x=+forward, rel_y=+left) convention to match training targets.
```

Decisions to lock down before implementing:
- **Target frame**: robot-relative (recommended, matches feature convention) vs world. Must match
  whatever the *original* training targets were — inspect the original training script / EE_244
  pipeline first. If unknown, re-derive from the sign conventions in `_build_window_features`.
- **`H` (LABEL_HORIZON)**: e.g. 3–5 frames (0.3–0.5 s at 10 Hz). Larger = smoother/less noisy
  label but more lag and more discarded tail samples.
- **Ego-motion source**: `robot_pose_fn()['twist']` (EKF) for compensation / sanity checks.

---

## 4. Pipeline

```
[deployment] ──log──► raw track buffers (JSONL, on-robot)
                          │
                   (offline, on laptop/CI)
                          ▼
                 label generation (future-diff, quality gates)
                          ▼
                 dataset assembly (rolling window, time-split)
                          ▼
                 train MLP + refit scalers  ──►  candidate artifacts
                          ▼
                 validation gate (vs incumbent + golden set)
                          ▼
                 promote (atomic swap) or reject
```

### Phase 1 — Deployment logging (on-robot, cheap)
Add an optional recorder to `VelocityEstimator` (gated by env var, default OFF). Per active track,
append time-stamped samples to a JSONL file. **Critical:** `history_global` is `maxlen=WINDOW_SIZE`
(10) — too short to hold the future horizon. The recorder needs its **own** longer buffer, or
simply log every centroid continuously per track id and reconstruct windows offline.

```python
# velocity_estimator.py — inside the tracker update / inference loop, when RECORD_TRACKS is set
record = {
    "t": time.time(),
    "track_id": tid,
    "centroid_local":  [cx, cy, cz],          # raw, pre-normalization
    "centroid_global": [gx, gy, gz],
    "visible_count":   visible_count,
    "robot_pose":  robot_pose_fn().get("pose")  if robot_pose_fn else None,
    "robot_twist": robot_pose_fn().get("twist") if robot_pose_fn else None,
    "sensor": "oakd",                          # tag the domain!
}
# append JSON line to  data/vel_logs/<date>_<session>.jsonl
```
Keep it lightweight: one dict per (track, frame), fsync-batched, rotated by session. Tag `sensor`
so Astra vs OAK-D data never gets silently mixed.

### Phase 2 — Label generation (offline)
`scripts/vel_selftrain/gen_labels.py`:
1. Load a session JSONL, group by `track_id`, sort by `t`.
2. Reject whole tracks that are too short (`< WINDOW_SIZE + 2*H`), or `sensor != oakd`.
3. Slide a window: for each end-frame `t` with a valid past window AND `H` future frames:
   - Build X via the **shared** `build_window_features(history_local[t-9..t])`.
   - Build y via world-frame central difference (Section 3) → rotate to target frame.
4. **Quality gates** (drop bad labels — noisy labels are worse than fewer labels):
   - low `visible_count` (occlusion) → drop
   - centroid range `> MAX_RANGE_M` → drop (depth unreliable far out)
   - implausible acceleration between consecutive labels (spike) → drop
   - near depth-hole / NaN centroid → drop
   - optional: require agreement within tolerance vs a LiDAR-derived velocity if available
5. Emit a parquet/npz shard: `X (N,40) float32`, `y (N,2) float32`, plus metadata
   (session, track_id, t, range, speed_bin) for stratification and debugging.

### Phase 3 — Dataset assembly
`scripts/vel_selftrain/build_dataset.py`:
- Concatenate a **rolling window** of the last N sessions/days (bounded so old domain doesn't
  dominate). Keep a small **replay buffer** of curated older samples to resist forgetting.
- **Split by session/time, NOT randomly** — random split leaks near-duplicate consecutive frames
  across train/val and inflates metrics. Hold out whole sessions for validation.
- **Stratify/balance by speed bins** (stopped / slow / fast) and by range — raw logs are dominated
  by near-stationary objects; without balancing the model collapses to "predict ~0".
- Refit `scaler_X`, `scaler_y` on the **training split only**; write to a new `scaler_params.json`.

### Phase 4 — Training
`scripts/vel_selftrain/train.py`:
- Reconstruct the **same MLP architecture** as the current TorchScript model. If the arch isn't
  recorded anywhere, recover it from the original EE_244 training code (find it first) or by
  introspecting the TorchScript graph. Document the arch in this repo so it's not lost again.
- Loss: **Huber/SmoothL1** on scaled targets (robust to residual label noise). Report MAE in m/s.
- Early stopping on the held-out session set. Fixed seed. Log train/val curves.
- Export: `torch.jit.script`/`trace` → `velocity_mlp.torchscript.candidate` + matching
  `scaler_params.json.candidate`. **Model and scaler are a matched pair — always ship together.**

### Phase 5 — Validation gate (the safety net)
`scripts/vel_selftrain/evaluate.py`. Compare **candidate vs incumbent** on:
- the held-out sessions, AND
- a fixed, version-controlled **"golden" benchmark set** (a handful of hand-verified OAK-D tracks
  with trusted labels) that never changes, so metrics are comparable across model generations.

Metrics: MAE(vx), MAE(vy), speed MAE, direction error (deg), **lag/latency** (cross-correlation
peak offset vs truth), **smoothness/jerk**, and **stopped-detection accuracy** (ties into the
`is_stopped` gate). Promote **only if**:
- candidate beats incumbent by a margin on the golden set, AND
- passes absolute thresholds (e.g. speed MAE < X m/s, no regression on stopped-accuracy).
Otherwise reject and keep incumbent. Emit a one-page report per run.

### Phase 6 — Promotion / deploy
- Version artifacts: `velocity_mlp.<gitsha>_<date>.torchscript` + paired scaler; symlink
  `velocity_mlp.torchscript` → current. Keep the last K for rollback.
- Atomic swap (write new, symlink flip). The server can reload via
  `velocity_estimator._load_model()` (already called on model-change paths ~L1667) or a service
  restart. Confirm which is safe mid-run.
- **Auto-rollback** if post-deploy live metrics regress (e.g. sudden jump in estimate variance).

---

## 5. Safeguards & failure modes (read before building)
- **No live weight updates.** Offline retrain + gate only.
- **No self-referential labels.** Labels come from geometry/future positions, never from the
  model's own predictions — otherwise errors compound.
- **Train/serve skew** — shared feature code is mandatory (Section 2).
- **Model+scaler coupling** — never swap one without the other.
- **Domain tagging** — never mix Astra and OAK-D samples unlabeled; the whole point is OAK-D data.
- **Label-noise > label-scarcity** — be aggressive with quality gates.
- **Time-based splits** — random splits lie.
- **Class imbalance** — balance speed bins or it predicts ~0.
- **Golden set is immutable** — it's the cross-generation yardstick.

---

## 6. Adjacent idea (separate track): tune the non-differentiable knobs
The MLP weights should be learned by gradient descent, but the surrounding pipeline knobs are
**non-differentiable** and currently hand-set: `WINDOW_SIZE`, the ±0.25 displacement clamp,
tracker association/gating thresholds, `is_stopped` triggers, `INFER_HZ`, `max_delta` smoothing.
Optimize these with **CMA-ES or Bayesian optimization** against the same validation fitness
(accuracy + smoothness + latency) on the golden set. Complementary to, not part of, the retrain
loop above.

---

## 7. First milestone (smallest useful slice)
1. Extract `build_window_features` into a shared importable module (kills train/serve skew).
2. Add Phase-1 JSONL logging behind an env flag; collect a few OAK-D sessions with varied motion.
3. Write `gen_labels.py` + a quick notebook to eyeball auto-labels vs a few hand-checked tracks.
4. Only then build train/eval/promote.

Deliberately defer the full CI automation until the label quality is visually trusted.

---

## 8. Open questions to resolve first
- Where is the **original training script** (EE_244 folder)? Recover the MLP architecture, the exact
  target-frame convention, and how the current `scaler_params.json` was fit — before writing `train.py`.
- Is the estimator fed OAK **spatial-detection** centroids, **depth-blob** centroids, or both? The
  label pipeline must log whichever is actually used at inference.
- Target frame: robot-relative vs world — confirm against original targets.
- Storage/retention policy for logs on the Jetson (disk is limited).
