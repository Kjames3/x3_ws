---
id: A-38
title: "No Ground-Plane Rejection — the Floor Is Inside the Acceptance Band and Merges With Every Pedestrian"
status: Logged
domain: Sensor Fusion
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-38"
session: "4. Sensor Fusion & Hardware Integration"
tags: [idea]
---

# A-38 — No Ground-Plane Rejection — the Floor Is Inside the Acceptance Band and Merges With Every Pedestrian

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 3)
- **ROI Tier:** **High ROI** (One precomputed row vector plus two vectorised ops, deletes the largest false blob present in every single frame)
- **Problem:** `_extract_depth_centroids` builds its obstacle mask from a depth **range** test alone — `(raw >= 0.5) & (raw < 1.5)` at `src/velocity_estimator.py:224` and `(raw >= 1.5) & (raw <= 4.0)` at line 235. There is no height test anywhere in the pipeline. The vertical coordinate is even computed and then discarded: `y_m` at line 288 is carried through the tracker as `cy_l` (lines 82, 89) and never read again, because feature construction uses only $r_x = c_z$ and $r_y = -c_x$ (lines 398–399, 533).

  The OAK-D mounts at `base_link` $z = 0.185\text{ m}$ with no pitch (`OAK_MOUNT_Z = 0.185`, `src/oakd_driver.py:60–61`). For a forward-looking camera at height $h$ with vertical half-FOV $\phi$, the floor first enters the image at ground range
  $$d_{\min} = \frac{h}{\tan\phi} = \frac{0.185}{\tan 25^\circ} = 0.40\text{ m}$$
  which is **below the $0.5\text{ m}$ near clip**. Every floor pixel across the entire usable range therefore falls inside the acceptance band, so the lower portion of every depth frame is a solid, permanently-lit region of the mask. Three consequences follow, and the second is the serious one:
  1. That region is the largest connected component in the frame. It survives the $5\times5$ `MORPH_OPEN`/`MORPH_CLOSE` at lines 239–241 trivially and clears the adaptive area gate (line 258) by orders of magnitude, so it permanently consumes at least one of the `MAX_OBSTACLES = 5` slots — the exact scarcity Idea A-27 has to ration.
  2. The floor region is **contiguous with the feet** of anyone standing on it, so `cv2.findContours(..., RETR_EXTERNAL)` at line 243 returns *one* contour containing both. The blob's median depth (line 280) is then the median of a floor wedge spanning $0.5\text{–}4.0\text{ m}$ rather than of the person: for a pedestrian at $2.0\text{ m}$ the merged median lands nearer $1.2\text{–}1.5\text{ m}$, a $0.5\text{–}0.8\text{ m}$ bias. Worse, the bias *moves with the scene* — as the robot drives or the subject steps, the floor-to-person pixel ratio shifts and the median slides — injecting a phantom depth velocity of order $0.3\text{–}0.5\text{ m/s}$ directly into $r_x$ and $dx$. That is larger than the entire sensor-noise budget Ideas A-26 and A-29 are aimed at.
  3. The `cv2.moments` centroid (lines 264–265) of the merged blob is dragged toward the image bottom and toward the floor's horizontal centre of mass, corrupting $x_m$ and therefore $r_y$ and all lateral velocity.

  This is also why `MIN_BLOB_AREA` and the range-adaptive gate are so hard to tune: the dominant blob in every frame is not an obstacle.
- **Proposed Solution:** Reject the ground plane before contour extraction, using quantities the codebase already holds. The height of the surface seen at working-resolution row $v$ and depth $Z$ is
  $$h(v, Z) = z_{\text{cam}} - Z\,\frac{v - c_y}{f_y}, \qquad z_{\text{cam}} = 0.185\text{ m}$$
  Precompute the row coefficient $k_v = (v - c_y)/f_y$ **once** in `__init__` as an $(H, 1)$ vector (~200 floats at the $320\times200$ working grid); the whole test is then two broadcast operations on the already-decimated frame:
  ```python
  height = self._z_cam - ds * self._k_v          # (H, W)
  np.logical_and(mask_bool, height > 0.15, out=mask_bool)
  ```
  Threshold at $0.15\text{ m}$ — below the X3's chassis clearance, so nothing the robot can actually strike is discarded, while the floor (true height $0$) is cut with a margin of roughly $5\times$ the OAK-D's depth noise at $2\text{ m}$. An upper bound handles ceilings and overhead signage ($h > 1.9\text{ m}$) with the same two ops.

  $f_y$ and $c_y$ come from `OakDCamera._read_intrinsics` (`src/oakd_driver.py:380–389`), via the `get_intrinsics()` accessor Idea A-34 already asks for. Note that $f_y \ne f_x$ on this pipeline: `setPreviewKeepAspectRatio(False)` (`src/oakd_driver.py:257`) squeezes the 1080p CAM_A sensor into the $480\times640$ preview (`nn_w, nn_h`, line 128), so the horizontal and vertical scale factors differ by $\tfrac{1920/480}{1080/640} = \tfrac{4.00}{1.69} = 2.37$. Reusing `fx` for the vertical axis — as line 288 does today for `y_m` — would misplace the horizon by more than a factor of two, which is exactly why this test must consume the measured $f_y$ rather than the hardcoded $277.0$. Fail safe: if intrinsics are unavailable (Astra path, or the OAK NN inactive), skip the height test and log once rather than apply a wrong $f_y$ — the same fail-closed discipline as Idea A-34(3). Apply it to the single mask Idea A-28 produces, **before** the morphology at lines 239–241, so the floor never reaches `findContours` and Idea A-02's `connectedComponentsWithStats` sees only real obstacles.
- **Expected Benefit:** Removes the single largest, always-present false blob from every frame; frees one to two of the five track slots for genuine obstacles; and eliminates the $0.5\text{–}0.8\text{ m}$ depth bias and $0.3\text{–}0.5\text{ m/s}$ phantom depth velocity that floor-to-person contour merging injects into $r_x$ and $dx$ on every standing or walking pedestrian — a larger accuracy defect than the whole depth-noise budget the smoothing ideas address. Also cuts contour-stage work by deleting the biggest region in the mask, for a cost of one cached $(H,1)$ vector and two vectorised operations ($\approx 0.15\text{ ms}$ at $320\times200$).

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-38`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
