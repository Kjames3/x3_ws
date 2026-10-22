---
id: A-42
title: "Depth Intrinsics Read From CAM_A While the Map Is Aligned to CAM_B in the Non-Spatial Pipeline"
status: Logged
domain: Sensor Fusion
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-42"
session: "4. Sensor Fusion & Hardware Integration"
tags: [idea]
---

# A-42 — Depth Intrinsics Read From CAM_A While the Map Is Aligned to CAM_B in the Non-Spatial Pipeline

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (Six lines in the driver plus one assertion, and it is the precondition without which A-34 and A-38 each apply the wrong camera model)
- **Problem:** `OakDCamera._build_pipeline` produces the depth map in **two mutually exclusive geometries**:
  - With the spatial NN (`with_spatial`, `src/oakd_driver.py:247–261`): `stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)` and `stereo.setOutputSize(self.nn_w, self.nn_h)` — a **$480\times640$ portrait** map in the **colour camera's** frame.
  - Without it (lines 272–276): `setDepthAlign(CAM_B)` (left mono, since `align_depth_to_left=True` at line 107) and `setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())` — a **$640\times400$ landscape** map in the **left mono camera's** frame (`THE_400_P`, line 223).

  `_read_intrinsics` (lines 380–389) unconditionally reads `calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self.nn_w, self.nn_h)` — CAM_A at $480\times640$ — regardless of which branch built the pipeline. In the non-spatial branch the stored $f_x, f_y, c_x, c_y$ therefore describe a **different physical sensor, at a different resolution, in the opposite aspect orientation** from the frame they would be applied to.

  Magnitudes, from nominal OAK-D Lite specs (the exact figures are on-device, and reading them is precisely the fix). CAM_A (IMX214, HFOV $\approx 69^\circ$) at $w = 480$: $f_x \approx 240/\tan 34.5^\circ \approx 349\text{ px}$, $(c_x, c_y) \approx (240, 320)$. CAM_B (OV7251 mono, HFOV $\approx 73^\circ$) at $w = 640$: $f_x \approx 320/\tan 36.5^\circ \approx 432\text{ px}$, $(c_x, c_y) \approx (320, 200)$. Substituting CAM_A's numbers into a CAM_B-aligned frame gives, through $X = (u - c_x)Z/f_x$,
  $$\frac{432}{349} = 1.24$$
  a **24% overestimate of every centroid's lateral coordinate**, on top of an $80\text{ px}$ error in $c_x$ and a **$120\text{ px}$** error in $c_y$, plus an unmodelled $\approx 3.75\text{ cm}$ lateral offset between the two optical centres (the $7.5\text{ cm}$ stereo baseline is nominally symmetric about the colour camera).

  This is a **prerequisite defect for two ideas already logged**. A-34 asks the estimator to consume a new `get_intrinsics()` for `x_m`/`y_m` at lines 287–288; A-38 consumes $f_y, c_y$ for its ground-plane test $h(v, Z) = z_{\text{cam}} - Z(v - c_y)/f_y$. With $c_y$ wrong by $120\text{ px}$ and $f_y$ taken from the wrong sensor, A-38's horizon at $Z = 2.0\text{ m}$ is misplaced by $2.0 \times 120/432 = 0.56\text{ m}$ of height — enough to either carve the torso out of every pedestrian or leave the entire floor inside the mask, i.e. **worse than having no height test at all**. Both fixes must consume the *aligned socket's* intrinsics, not CAM_A's.

  Worse, the branch is not fixed at startup. `_run` recomputes `with_spatial = self._want_spatial and self._spatial_ok` on every reconnect (line 285) and sets `self._spatial_ok = False` after repeated spatial failures (line 372), so a blob-load or NN failure flips the depth frame from $480\times640$ portrait to $640\times400$ landscape **mid-session** while `_read_intrinsics` keeps reporting CAM_A@$480\times640$ and its log line at line 386 keeps saying `"CAM_A intrinsics"`. Meanwhile `_extract_depth_centroids` derives its principal point from whatever arrives — `h, w = raw_depth_frame.shape[:2]` at line 245, `x_m = (cx - w/2.0) * Z / fx` at line 287 with a hardcoded `fx = 277.0` at line 286 — so the geometry transposes underneath the estimator and **nothing anywhere notices**.
- **Proposed Solution:** Read the intrinsics of the socket the pipeline actually aligned to, at the size it actually emitted, and make the result self-describing so a mid-session rebuild cannot go unnoticed.
  1. Record the choice where it is made, in `_build_pipeline`:
     ```python
     self._depth_socket = (dai.CameraBoardSocket.CAM_A if with_spatial
                           else (dai.CameraBoardSocket.CAM_B if self.align_depth_to_left
                                 else dai.CameraBoardSocket.CAM_C))
     self._depth_wh = (self.nn_w, self.nn_h) if with_spatial else (640, 400)
     ```
     and in `_read_intrinsics` replace line 383 with `M = calib.getCameraIntrinsics(self._depth_socket, *self._depth_wh)`. Include the socket name and resolution in the log at line 386 so the configuration is visible in `journalctl -u x3_server` instead of asserted.
  2. Make the `get_intrinsics()` accessor A-34 asks for return **`(fx, fy, cx, cy, w, h)`**, and have the estimator compare `(w, h)` against `raw_depth_frame.shape[1::-1]` whenever the frame shape changes. On a match, rescale by the decimation factor and re-derive A-38's cached $k_v = (v - c_y)/f_y$ row vector. On a mismatch, **fail closed** — skip the height test and the projection-based visual gate, log once — rather than back-project with another sensor's model. This is the same fail-closed discipline as A-34(3) and A-38, extended to cover the case where intrinsics exist but describe the wrong camera, which is strictly more dangerous than their absence.
  3. In the non-spatial branch the depth map is in CAM_B's frame, so A-34(2)'s clean route (gate against `get_spatial_detections()`, same camera, no extrinsic) is unavailable by construction — the NN is not running. Note that explicitly in A-34's fallback: when `spatial_active` is false, both the OAK$\to$Astra transform *and* the CAM_A$\to$CAM_B offset are in play, which is a further argument for leaving that gate disabled rather than approximating it.
- **Expected Benefit:** Removes a 24% systematic scale error and an $80/120\text{ px}$ principal-point error from every 3D centroid whenever the OAK-D runs without the spatial NN — the configuration the driver enters *automatically* on any blob failure, and the one a bench setup without the blob runs in permanently. Because the lateral coordinate feeds $r_y$ directly (lines 287, 399), that scale error propagates one-for-one into predicted lateral velocity $v_y$. Just as important, it is the precondition without which A-34's corrected projection and A-38's ground-plane rejection would each be applied with the wrong camera model — converting two accuracy fixes into two new accuracy defects — and it makes a silent mid-session geometry transposition impossible.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-42`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
