---
id: A-34
title: "Visual Gating Reprojects OAK-D Centroids With Astra Intrinsics and No Inter-Camera Extrinsic"
status: Logged
domain: Sensor Fusion
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-34"
session: "4. Sensor Fusion & Hardware Integration"
tags: [idea]
---

# A-34 — Visual Gating Reprojects OAK-D Centroids With Astra Intrinsics and No Inter-Camera Extrinsic

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 2)
- **ROI Tier:** **High ROI** (Moderate effort, converts a gate that is disabled because it never worked into a functioning person filter)
- **Problem:** The Visual-LiDAR Fusion Gate (Idea 146) at `src/velocity_estimator.py:290–315` projects each depth centroid into image space with hardcoded constants — `fx_full = 554.0`, `cx_full = 320.0`, `cy_full = 240.0` (lines 298–300) — and tests the result against `detections_fn()` boxes. Every term in that projection belongs to a different camera than the data it is applied to:
  1. **The 3D point is OAK-D.** `x_m` is built at line 287 with `fx = 277.0` (the Astra's $554$ halved for the $[::2,::2]$ decimation) but the frame is the OAK-D's, whose CAM_A focal length at the NN output width of $480\text{ px}$ is $f_x \approx 240/\tan(34.5^\circ) \approx 349$, i.e. $\approx 175$ after decimation. The driver already reads the true values from on-device calibration and stores them (`src/oakd_driver.py:380–389`), and the estimator never asks for them.
  2. **The boxes are Astra.** `get_latest_yolo_detections` (`src/server_x3.py:1148–1150`) returns `last_detections`, produced by the host YOLO at lines 1904–1919 on `camera.get_frame()` — the Orbbec Astra Pro RGB stream at $640\times480$ — while the depth source ranks the OAK first (lines 1128–1133). These are two physically separate cameras with an unmodelled baseline: the OAK mounts at `base_link` $x = 0.0435,\ z = 0.185$ (`src/oakd_driver.py:60–61`); no extrinsic transform is applied anywhere in the gate.
  3. **The principal point is transposed.** With spatial detection active the OAK depth is aligned to CAM_A and emitted at the NN resolution `(nn_w, nn_h) = (480, 640)` — portrait (`src/oakd_driver.py:254, 260–261`) — so its true centre is $(240, 320)$. The code assumes the Astra's landscape $(320, 240)$, an $80\text{ px}$ error on **both** axes before any other term.

  Composing (1) and the projection at line 301 gives $u = X_{\text{true}} \cdot 349 / Z + 320$, whereas a correct projection into the Astra image is $u = X_{\text{true}} \cdot 554 / Z + 320$: the lateral offset is compressed toward the image centre by $37\%$. A pedestrian at $Z = 2.0\text{ m}$, $X = +1.0\text{ m}$ should land at $u = 597$ but is placed at $u = 494$ — a $103\text{ px}$ error against a person box whose half-width at that range is only $\approx 69\text{ px}$, so even with the $15\text{ px}$ pad at line 307 the point falls outside and the centroid is rejected. The vertical term is worse still, carrying the same $37\%$ compression plus the $80\text{ px}$ principal-point transposition plus the unmodelled camera height difference ($0.10\text{ m}$ of mount offset alone is $28\text{ px}$ at $Z = 2\text{ m}$). This is almost certainly why the gate's `continue` is commented out at line 312 with `# [DISABLED FOR TESTING]`: with the projection wrong, enabling it would discard correctly detected pedestrians, so Idea 146 is presently inert and every depth blob — walls, chair legs, doorframes — is tracked and fed to the MLP.
- **Proposed Solution:** Project with measured parameters and a real transform, or do not project at all.
  1. Add `get_intrinsics()` to `OakDCamera`, returning the values already captured by `_read_intrinsics` as `(fx, fy, cx, cy, w, h)`, and pass a `intrinsics_fn` into `VelocityEstimator.__init__`. Use $f_x, f_y, c_x, c_y$ scaled by the decimation factor for `x_m`/`y_m` at lines 287–288 (this also supplies the calibrated $K$ that Idea J-07 needs for its undistortion step) and cache them, re-querying only when the source object changes.
  2. Gate against **same-camera** detections. `OakDCamera.get_spatial_detections()` already returns person boxes *in the OAK CAM_A frame* together with `xyz_m`, so when `oak.spatial_active` is true, source the gate from there and the projection collapses to the OAK's own $K$ — no extrinsic needed, and the $15\text{ px}$ pad regains its intended meaning. Fall back to the Astra boxes only when the OAK NN is inactive, and in that case apply the static OAK$\to$Astra transform from the URDF rather than assuming the cameras are coincident.
  3. **Fail closed, not silently:** if intrinsics are unavailable, leave the gate disabled and log once, instead of running a projection built from constants that do not describe the sensor.
  Combine with Idea 259 (hoist `detections_fn()` out of the per-contour loop) so the corrected gate costs one call per frame rather than one per contour.
- **Expected Benefit:** Turns Idea 146 from permanently disabled code into a working person filter, which is the cheapest available route to suppressing the static clutter that currently consumes the $\texttt{MAX\_OBSTACLES} = 5$ budget (the failure Idea A-27 mitigates by ranking). Fixing (1) independently removes a $37\%$ systematic underestimate of every centroid's lateral coordinate $X$ — which propagates directly into the $r_y$ feature and therefore scales predicted lateral velocity $v_y$ by $0.63$, understating exactly the crossing-pedestrian motion the bypass-direction logic depends on.

---

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-34`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
