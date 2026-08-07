# Perception Performance Monitoring

How the robot keeps a running score of its own perception stack: **OAK-D detection
accuracy + latency** and **velocity-MLP accuracy + latency**.

Implementation: [`src/perf_monitor.py`](src/perf_monitor.py) (collection),
[`src/perf_report.py`](src/perf_report.py) (offline reporting), GUI card
"Perception Performance".

---

## The problem with "accuracy" on a live robot

Latency is easy — you time it. Accuracy normally needs ground truth, which you
don't have while driving around. So the method uses a different source of truth
for each of the two models:

| Model | Ground truth source | What you get |
|---|---|---|
| OAK-D YOLO detector | none available online → **quality proxies** that move when the detector degrades, plus an optional offline labelled score folded into the same scorecard | early warning, not an absolute number |
| Velocity MLP | **the future** — the model is causal, so its prediction at time *t* can be checked against the track's actual motion measured after *t* | a real, absolute error in m/s |

---

## 1. Latency

Every instrumented stage feeds a fixed-size rolling series (600 samples) and is
summarised as **p50 / p95 / p99 / max**, plus an event rate over a 5 s window.
Percentiles, not means — a mean hides the 1-in-20 frame that arrives 150 ms late,
and that frame is the one that makes the robot bump into someone.

| Series | Where it's taken |
|---|---|
| `oak.nn_decode_ms` | host-side decode of the `[85, 6300]` tensor + NMS + depth back-projection (`OakDCamera._process_nn`) |
| `oak.det_e2e_ms` | device→host transport: `dai.Clock.now() - nndata.getTimestamp()` |
| `oak.det_interval_ms`, rate `oak.det` | detection frame arrival cadence |
| `vel.centroid_ms` | depth blob extraction → centroids |
| `vel.infer_ms` | the MLP forward pass alone |
| `vel.cycle_ms`, `vel.cycle_interval_ms` | whole estimator cycle, and its achieved period vs the 10 Hz target |

`vel.cycle_ms` creeping toward 100 ms means the estimator is about to miss its
`INFER_HZ` budget — that shows up here before it shows up as bad tracking.

## 2. Detection accuracy (label-free proxies)

Recorded once per inference frame in `record_detection_frame()`:

- **`conf_mean` / `conf_p10`** — the low tail moves first when the scene drifts
  out of the model's domain (lighting, distance, motion blur).
- **`det_per_frame`** and **`empty_frame_pct`** — sudden drops = misses.
- **`depth_valid_pct`** — fraction of boxes that got a valid 3D back-projection.
  A box with no depth is useless to navigation, so this is an accuracy metric,
  not a plumbing one. It catches depth/RGB misalignment specifically.
- **`flicker_per_100f`** — the strongest label-free signal available: a label
  present, absent for 1–2 frames, then present again. The object didn't leave
  and come back; the detector missed it. Flicker is a direct proxy for the false
  negative rate, and unlike confidence it can't be gamed by a confident-but-wrong
  model.

None of these prove the detector is *correct* — they detect **degradation**, which
is what you want continuously. For an absolute number, run a labelled evaluation
offline (annotated set, or replay a rosbag against a stronger reference model) and
push the result in with:

```python
get_monitor().record_labeled_eval(precision=0.91, recall=0.84, map50=0.78,
                                  n_images=300, source="bag_20260807")
```

It then rides along in the same snapshot and GUI card, so one view answers both
"is it accurate?" and "is it degrading right now?".

## 3. Velocity MLP accuracy (self-supervised, no labels)

The MLP maps a 10-frame window of an obstacle's centroid history ending at time
*t* → `(vx, vy)`. It is **causal**, so a few hundred milliseconds later the true
velocity at *t* is simply measurable:

```
                 world-frame track positions
   ... ──●───────────●───────────●── ...
       t-0.35s       t        t+0.35s
          └──── central difference ────┘   =  true (vx, vy) in world frame
                       │
                 rotate by -θ_robot(t)
                       ↓
          true velocity in the frame the model predicted in
```

Each cycle the estimator calls `record_velocity_sample(tid, t, world_x, world_y,
robot_theta, pred_vx, pred_vy)`. The monitor buffers the stream per track and
scores each prediction once samples exist on **both** sides of *t*
(`LABEL_HORIZON_S = 0.35 s`, ±0.12 s matching tolerance). Predictions whose track
dies before the horizon are counted as `unlabelable` rather than silently dropped.

Two details that matter:

- **World frame, then rotate.** Differencing the *local* centroid conflates the
  pedestrian's motion with the robot's own. The tracker already maintains
  `history_global`, so the label is ego-motion-free by construction.
- **Central difference, not forward.** Symmetric around *t*, so a constant-speed
  target gives an unbiased label; forward difference lags by half the horizon.

Reported over the last 600 scored predictions:

| Metric | Reads as |
|---|---|
| `mae_vx`, `mae_vy`, `rmse` | typical error in m/s — compare against ~1.2 m/s walking speed |
| `bias_vx`, `bias_vy` | systematic offset; non-zero = a calibration/domain problem, not noise |
| `speed_mae`, `speed_bias` | magnitude error; a negative `speed_bias` means the model under-reacts (the dangerous direction) |
| `heading_mae_deg` | direction error on moving targets only |
| `r2` | ≤ 0 means the model is no better than predicting a constant |
| `moving_pct`, `gt_speed_mean` | context — a great score on a room full of stationary chairs means nothing |

Zero-velocity outputs from the gating logic (stop-trigger, low visible-count,
>1.8 m) are fed in too, so **false zeros on a moving pedestrian are counted as
errors** rather than hidden.

This is the same auto-labelling trick as
[`VELOCITY_SELF_TRAINING_PLAN.md`](VELOCITY_SELF_TRAINING_PLAN.md), used here for
measurement instead of retraining — which also means these numbers are exactly
the validation gate that plan needs before a model hot-swap.

---

## Using it

**Live (GUI):** the "Perception Performance" card updates at 2 Hz from the
`readout_slow` telemetry lane. Colour-coded against operational thresholds.
"Reset stats" clears the window; switching velocity models resets automatically
so the old model's errors don't pollute the new one's score.

**Live (WebSocket):**
```json
{"type": "get_perf"}     → {"type": "perf_snapshot", "perf": { ...full breakdown... }}
{"type": "reset_perf"}
```

**Offline:** a snapshot is appended to `src/logs/perf/perf_<timestamp>.jsonl`
every 10 s while the server runs.

```bash
python3 src/perf_report.py            # scorecard + trend for the newest run
python3 src/perf_report.py --all      # one line per run
python3 src/perf_report.py --watch    # live tail
```

Disable file logging with `X3_PERF_LOG=0`.

**A/B-ing a model:** reset, drive the same route, snapshot; swap model, repeat.
`rmse` / `speed_bias` / `heading_mae_deg` on the two runs are directly comparable
because the ground truth is generated the same way for both.

## Cost

Bounded deques throughout (600 latency samples, 300 detection frames, 600 scored
predictions, 40 positions per track, tracks reaped 3 s after last sight). The hot
path does one deque append and a handful of float ops; percentile sorting only
happens when a snapshot is built (2 Hz for the compact form, 0.1 Hz for the full
one). No allocation growth over a long run.
