---
id: A-58
title: "One `scaler_params.json` Behind a Module Constant Serves Two Different Model Artifacts, and the Runtime Model Switch Cannot Swap It"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-58"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-58 — One `scaler_params.json` Behind a Module Constant Serves Two Different Model Artifacts, and the Runtime Model Switch Cannot Swap It

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 8)
- **ROI Tier:** **High ROI** (One path derivation, one digest field, and moving three assignments below a `try` — closes an unbounded, entirely silent gain error on the exported velocity)
- **Problem:** The model path is a *variable* and the scaler path is a *constant*, and the GUI can change only the first.
  ```python
  MODEL_PATH         = str(_SRC_DIR / "velocity_mlp.torchscript")      # line 30
  SCALER_PARAMS_PATH = str(_SRC_DIR / "scaler_params.json")            # line 31
  ...
  self.model_path = model_path or MODEL_PATH                           # line 156 — overridable
  ...
  self._model = torch.jit.load(self.model_path, ...)                   # line 181 — follows the override
  with open(SCALER_PARAMS_PATH, "r") as f:                             # line 193 — follows the constant
  ```
  There is no `self.scaler_path`. `_load_model` (lines 179–204) is the *only* place either is read, and it reloads the network from an instance attribute while reloading the normalisation from a module global.

  `src/server_x3.py:1715–1726` exposes exactly that mismatch to the operator. The `set_velocity_model` message accepts two names, `velocity_mlp` and `velocity_mlp_finetuned`, then does:
  ```python
  velocity_estimator.stop()
  velocity_estimator.model_path = new_path
  velocity_estimator._load_model()
  velocity_estimator.start()
  ```
  The weights swap; the scaler cannot. And the two artifacts are genuinely different networks — `velocity_mlp.torchscript` and `velocity_mlp_finetuned.torchscript` are both $241\,842$ bytes (same architecture) with different digests (`23d2feb…` vs `4ac71b5…`). There is exactly one scaler in the tree: `scaler_params.json`, `scaler_X.pkl`, `scaler_y.pkl`. No `*_finetuned` variant exists.

  A `StandardScaler` is a property of the *dataset*, not of the architecture. If the fine-tune touched the data at all — which is the entire premise of `VELOCITY_SELF_TRAINING_PLAN.md` and the only reason a second artifact exists — then $(\mu_{\text{ft}}, \sigma_{\text{ft}}) \neq (\mu_{\text{base}}, \sigma_{\text{base}})$ and serving applies the wrong transform on **both** ends:
  $$x_{\text{fed}} = \frac{x - \mu_{X,\text{base}}}{\sigma_{X,\text{base}}} \;\neq\; \frac{x - \mu_{X,\text{ft}}}{\sigma_{X,\text{ft}}}, \qquad \hat v = f_{\text{ft}}\!\left(x_{\text{fed}}\right)\cdot\sigma_{y,\text{base}} + \mu_{y,\text{base}}$$
  The output side is the dangerous one, because $\sigma_y$ is a **pure multiplicative gain on the exported velocity** and nothing downstream can tell a gain error from a fast pedestrian. The shipped values are $\sigma_y = [0.9047,\ 0.4549]$, $\mu_y = [0.0031,\ 0.0013]$ (`scaler_params.json`). A fine-tune whose speed distribution is merely 30% narrower — an unremarkable outcome when the follow-up dataset is collected indoors at walking pace — yields $\sigma_{y,\text{ft}} \approx [0.633,\ 0.318]$, and serving those weights through the base scaler multiplies every reported $v_x$ by $0.9047 / 0.633 = \mathbf{1.43}$. A pedestrian at a true $1.0\text{ m/s}$ is exported at $1.43\text{ m/s}$ to the speed scaler and the CBF, with **no symptom at all**: the values stay inside the $\pm 2.5\text{ m/s}$ clip at line 569, inside the $0.3\text{ m/s}$ acceleration limit at line 590, and inside every plausibility range a human would eyeball in the throttled printout at lines 612–615. The error is a scale factor, so it survives every smoother in the pipeline including Idea A-39's, and it is the *opposite* sign of safe if the fine-tune widened the distribution instead.

  A second failure rides on the same eleven lines. `_load_model`'s `except` at lines 203–204 **only logs**:
  - If `torch.jit.load` raises (truncated file, wrong torch version), `self._model` keeps the *previously loaded object* while `self.model_path` was already rebound at `server_x3.py:1722`. No exception escapes, so the `try` at lines 1721–1726 completes and the GUI is sent `{"success": true, "model": "velocity_mlp_finetuned"}`. The operator is told the fine-tuned model is live; the base model is running.
  - If the load succeeds but the scaler read fails, `self.scaler_X_mean` and friends retain the *previous* arrays and inference continues on a **half-swapped configuration** — new weights, old normalisation — which is the top defect above, now reached by accident rather than by design.
  - Idea A-52's `torch.jit.freeze` and Idea A-37's fixed-shape trace both land inside this same function, so hardening it once pays for all three.
- **Proposed Solution:** Make the scaler travel with the model, bind them by digest, and fail loudly.
  1. **Make it an instance attribute.** Derive it from the model path — `Path(self.model_path).with_suffix('') .parent / f"{Path(self.model_path).stem}_scaler.json"`, falling back to `SCALER_PARAMS_PATH` when that file is absent — and store it as `self.scaler_path` so it is swappable exactly like `self.model_path`. Accept an optional `scaler_path=` constructor argument for symmetry with `model_path=`.
  2. **Ship the missing artifact.** Export `velocity_mlp_finetuned_scaler.json` from the fine-tune's own training set (the recipe is already written down in `VELOCITY_SELF_TRAINING_PLAN.md` §3). If the fine-tune provably reused the base dataset's statistics, ship a copy anyway — an explicit duplicate is a checkable claim; a shared global is an assumption.
  3. **Bind them cryptographically.** Emit into every scaler JSON at export time:
     ```json
     {"model_sha256": "...", "window": 10, "infer_hz": 10,
      "feature_layout": "rel_x,rel_y,dx,dy x10", "translation_normalised": false}
     ```
     and assert the loaded artifact's digest matches in `_load_model`; refuse to start on mismatch rather than serving a silently mis-scaled velocity. The extra fields are not decoration: `window`/`infer_hz` pin the $\Delta t = 0.1\text{ s}$ contract that Idea A-43 shows is currently an unchecked assumption, and `translation_normalised` records the flag Idea A-30 shows is currently ambiguous between the training script and the serving path — the one bit whose value decides whether 20 of the 40 input channels are real data or a frozen constant.
  4. **Publish atomically.** Load the model and the scaler into *locals*, and only assign `self._model` and the five scaler arrays after **both** succeed; then re-raise instead of swallowing. `server_x3.py:1727–1737` already has the `except` branch that reports `success: false` with the error text to the GUI — it is dead code today purely because `_load_model` never raises. On failure the estimator keeps running the previous, self-consistent pair.
  5. **Show it in the readout.** Report the active model name *and* the first eight hex digits of the scaler digest alongside the estimates, so the GUI displays what is running rather than what was clicked.
- **Expected Benefit:** Closes a silent, unbounded multiplicative error on the exported velocity — a fine-tune with a 30% narrower speed distribution alone produces a **43% over-report on $v_x$** with no visible symptom, on the number the CBF and speed scaler treat as ground truth — and makes the currently-invisible half-swap on a failed model switch an explicit, operator-visible failure. The embedded `window`/`infer_hz`/`translation_normalised` fields turn three assumptions that Ideas A-30, A-43 and A-45 each had to *infer* from scaler statistics into declared, asserted metadata, so the next fine-tune cannot silently break them.

---

## 3. Performance & Execution Efficiency

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-58`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
