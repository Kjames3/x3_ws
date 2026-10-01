---
id: A-52
title: "Traced Graph Is Never Frozen — BatchNorm and Dropout Dispatched as Live Ops in a 40→256→128→64→2 MLP"
status: Logged
domain: Performance
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-52"
session: "3. Performance & Execution Efficiency"
tags: [idea]
---

# A-52 — Traced Graph Is Never Frozen — BatchNorm and Dropout Dispatched as Live Ops in a 40→256→128→64→2 MLP

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (One line replacing `torch.jit.trace`; deletes 6 of the graph's 13 dispatched ops, all of which do no inference-time work)
- **Problem:** `_load_model` loads the TorchScript module, calls `.eval()`, and then traces it (`src/velocity_estimator.py:181–187`):
  ```python
  self._model = torch.jit.load(self.model_path, map_location='cpu')
  self._model.eval()
  dummy_input = torch.zeros((MAX_OBSTACLES, 40), dtype=torch.float32)
  self._model = torch.jit.trace(self._model, dummy_input)
  ```
  It never calls `torch.jit.freeze` or `torch.jit.optimize_for_inference`. That matters because of what is actually inside the archive. Unzipping `src/velocity_mlp.torchscript` and reading `code/__torch__/torch/nn/modules/container.py` gives the served `Sequential` verbatim — thirteen submodules:
  $$\texttt{Linear} \to \texttt{BatchNorm1d} \to \texttt{ReLU} \to \texttt{Dropout}\;(\times 3\ \text{blocks}) \to \texttt{Linear}$$
  and the tensor storages fix every dimension exactly: `data/0` is $40{,}960\text{ B} = 256 \times 40$ fp32, `data/7` is $131{,}072\text{ B} = 128 \times 256$, `data/14` is $32{,}768\text{ B} = 64 \times 128$, `data/21` is $512\text{ B} = 2 \times 64$. Each block additionally carries five same-width vectors (bias, $\gamma$, $\beta$, running mean, running variance) plus an 8-byte `num_batches_tracked`. So the model is **$40 \to 256 \to 128 \to 64 \to 2$**, $51{,}328$ MACs per sample, $\approx 209\text{ KiB}$ of tensor data in a $236\text{ KiB}$ archive.

  Every forward pass therefore dispatches **13 ATen ops**: 4 × `aten::linear`, 3 × `aten::batch_norm`, 3 × `aten::relu`, 3 × `aten::dropout`. Six of those thirteen — the BatchNorms and the Dropouts — perform no inference-time function whatsoever:
  - `aten::dropout` with `train=False` returns its input unchanged. It is pure dispatcher cost.
  - `aten::batch_norm` in eval mode is a fixed per-channel affine, which is **exactly absorbable** into the preceding `aten::linear`.

  `torch.jit.trace` at line 187 cannot remove either. Tracing an already-scripted module re-records the same `aten` calls with the parameters still bound as **module attributes**; the passes that would fix this — `FoldFrozenLinearBatchnorm`, dropout removal, and constant propagation — are all gated on the module being *frozen*, which promotes attributes to graph constants. Idea 137's trace is thus a no-op with respect to the two optimisations that actually apply to this graph, which is consistent with Idea A-37's separate finding that it also fails to hold its shape specialisation.

  Magnitude: at batch 5 the network is $256{,}640$ MACs $= 513\text{ kFLOP}$, roughly $86\ \mu\text{s}$ of arithmetic for a single-threaded NEON sgemm at $\approx 6\text{ GFLOP/s}$ on the Orin's Cortex-A78AE. Thirteen ATen dispatches at $2\text{–}5\ \mu\text{s}$ each (dispatcher plus `TensorIterator` setup) is $26\text{–}65\ \mu\text{s}$ — **23–43% of the forward pass is dispatch overhead, and just under half of the dispatched ops are dead.** The BatchNorms additionally read-modify-write $256 + 128 + 64 = 448$ channels $\times$ batch 5 $= 2{,}240$ elements per cycle ($\approx 18\text{ kB}$ of traffic) to apply a transform that a folded weight matrix applies for free.
- **Proposed Solution:** Freeze the module so the eval-mode collapse actually happens, then optionally take the folded weights out of PyTorch entirely.
  1. **Replace line 187** with the frozen-and-optimised form:
     ```python
     self._model = torch.jit.optimize_for_inference(torch.jit.freeze(self._model))
     ```
     `freeze()` requires `.eval()` (already called at line 183) and inlines the parameters as constants; `optimize_for_inference` then folds each BatchNorm into its preceding Linear and drops the Dropouts as dead code. The graph goes from **13 dispatched ops to 7** (4 × `linear`, 3 × `relu`). Keep the existing `try/except` so a failure falls back to the untraced module exactly as it does today.
  2. **The fold is exact**, which is why it is safe to do at load time. For $y = Wx + b$ followed by $z = \gamma\,(y - \mu)/\sqrt{\sigma^2 + \varepsilon} + \beta$:
     $$W' = \operatorname{diag}\!\left(\frac{\gamma}{\sqrt{\sigma^2+\varepsilon}}\right)W, \qquad b' = \frac{\gamma\,(b - \mu)}{\sqrt{\sigma^2+\varepsilon}} + \beta$$
     Bit-for-bit output equality is not guaranteed (different summation order), but the difference is at the fp32 rounding floor — five orders of magnitude below the $0.047\text{ m/s}$ estimator noise Idea A-48 measures. Verify with one assertion at load time: `"aten::batch_norm" not in self._model.inlined_graph.str()`.
  3. **It closes a live hazard in Idea A-37.** A-37 proposes running permanently at batch 5 with `self.x_tensor_preallocated[num_tracks:].zero_()`. That is safe *today* only because this particular export happens to carry `training = False`; a retrain exported without `.eval()` would leave `aten::batch_norm` in training mode, and the zero-padded rows would then enter the batch statistics and corrupt **every real track's output in the batch**. After freezing, BN is a constant affine and no such coupling can exist by construction. The hot-swap path at `src/server_x3.py:1720–1726` reloads a model mid-drive, so this is not hypothetical — it is one bad export away, and it would fail silently.
  4. **Optional follow-on, now trivial:** once folded there are exactly four weight matrices ($256\times40$, $128\times256$, $64\times128$, $2\times64$) and four bias vectors. Dump them once to a `.npz` beside `scaler_params.json` and serve the model as four NumPy GEMMs with three in-place `np.maximum(..., 0, out=...)` calls — no dispatcher, no shape guard to fail (which makes A-37's concern moot rather than mitigated), and no `torch.from_numpy` / `.copy_()` / `.numpy()` round-trip (Idea J-01's target). This is the cheaper half of Idea J-03: ONNX Runtime is a $\approx 50\text{ MB}$ dependency for a 52-thousand-parameter model, while NumPy is already imported at line 13.
  5. **Explicitly not claimed:** this does not reduce process RSS. `src/server_x3.py` imports `torch` at line 47 and `ultralytics.YOLO` at line 66 regardless of the estimator, so the PyTorch runtime stays resident either way. The win here is per-call latency and determinism, not memory.
- **Expected Benefit:** Cuts the served graph from 13 dispatched ATen ops to 7, deleting six that do no inference-time work — an estimated $12\text{–}30\ \mu\text{s}$ of dispatch and $\approx 18\text{ kB}$ of elementwise BatchNorm traffic per cycle, against a forward pass whose actual arithmetic is only $\approx 86\ \mu\text{s}$, i.e. a **15–25% reduction in MLP forward time**. Independent of and additive to Idea A-37: A-37 keeps the optimised graph *resident* across cycles, this makes the optimised graph *smaller*. Also converts the eval-mode BatchNorm collapse from an incidental property of one export into a structural guarantee, closing a silent-corruption path in the mid-drive model hot-swap. Cost is one line.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-52`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
