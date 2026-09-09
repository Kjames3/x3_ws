---
id: A-54
title: "A 0.185 m Mount With a 24.8° Vertical Half-FOV Crops the Pedestrian at the Waist — the Depth Reference Migrates to the Legs and Picks Up Gait"
status: Logged
domain: Sensor Fusion
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-54"
session: "4. Sensor Fusion & Hardware Integration"
tags: [idea]
---

# A-54 — A 0.185 m Mount With a 24.8° Vertical Half-FOV Crops the Pedestrian at the Waist — the Depth Reference Migrates to the Legs and Picks Up Gait

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (Reuses the per-row height vector Idea A-38 already precomputes, applied as a selector instead of a rejector; removes a gait-coherent oscillation no smoother in this log can touch)
- **Problem:** The OAK-D mounts at `base_link` $z = 0.185\text{ m}$ with no pitch (`OAK_MOUNT_Z = 0.185`, `src/oakd_driver.py:61`) and streams stereo at `THE_400_P`, i.e. $640\times400$ (`src/oakd_driver.py:222–226`). Using the CAM_B focal length Idea A-42 derives — $f_y \approx 432\text{ px}$ at $640$ width, square pixels — the vertical half-FOV is
  $$\phi = \arctan\!\left(\frac{200}{432}\right) = 24.8^\circ, \qquad \tan\phi = 0.463$$
  so the highest surface visible at ground range $Z$ is
  $$h_{\max}(Z) = 0.185 + 0.463\,Z$$
  A $1.70\text{ m}$ person is fully in frame only beyond $Z = (1.70 - 0.185)/0.463 = \mathbf{3.27\text{ m}}$. But the estimator's acceptance band ends at $4.0\text{ m}$ (`src/velocity_estimator.py:235, 274`) and the proximity gate zeroes velocity beyond $1.8\text{ m}$ (line 485), so **the MLP is only ever run on tracks in $[0.5, 1.8]\text{ m}$** — a range across which $h_{\max}$ runs from $0.42\text{ m}$ to $1.02\text{ m}$:

  | $Z$ (m) | 1.8 | 1.5 | 1.2 | 1.0 | 0.8 | 0.5 |
  |:--|:--|:--|:--|:--|:--|:--|
  | $h_{\max}$ (m) | 1.02 | 0.88 | 0.74 | 0.65 | 0.56 | 0.42 |
  | body part | waist | hips | upper thigh | mid-thigh | knee | shin |

  Over its entire operating band the estimator therefore measures **legs**, and the leg fraction of the blob rises monotonically as the person approaches. Three consequences.

  1. **The blob median at line 280 acquires a gait oscillation.** During walking the swing foot translates $\approx 0.6\text{–}0.7\text{ m}$ fore–aft per stride and the knee $\approx \pm 0.2\text{ m}$. The two legs partially cancel (one forward while the other is back), but not exactly: the forward leg is nearer and so projects larger ($\text{px} \propto 1/Z^2$), biasing the pixel-weighted median toward it during fore-swing. The residual median oscillation is $\approx \pm 0.05\text{–}0.10\text{ m}$ at the step frequency, $f \approx 1.8\text{–}2.0\text{ Hz}$ for normal walking. A $\pm 0.075\text{ m}$ sinusoid at $1.9\text{ Hz}$ has peak derivative
     $$2\pi f A = 2\pi (1.9)(0.075) = 0.90\text{ m/s}$$
     i.e. a per-frame displacement of $0.090\text{ m}$ — **45% of the $0.198\text{ m}$ training spread of the `dx` channel** and $2.1\times$ the $0.042\text{ m}$ sensor-noise term Idea A-39 is costed against. Unlike sensor noise it is **coherent**, so no smoother removes it: at $f/f_s = 0.19$ the 5-point quadratic Savitzky–Golay kernel passes it through nearly intact. And at $10\text{ Hz}$ it is sampled only $5.3\times$ per cycle, barely above Nyquist, so it aliases badly under the $\Delta t$ jitter Idea A-43 documents.
  2. **It defeats Idea A-29's repaired stop gate in the worst direction.** For roughly half the gait cycle the visible legs are in stance — planted, momentarily stationary — while the torso translates. The measured depth is briefly still, so a stop gate raised to $>95\%$ sensitivity will **fire twice per stride on a pedestrian who is actively walking toward the robot**, hard-zeroing $v_x$ at exactly the moment TTC scaling needs it. A-29 is a good fix that this defect turns into a hazard, so the two must be considered together.
  3. **The reference point slides with range, injecting a systematic drift.** Because $h_{\max}$ is a function of $Z$, the body part being measured changes continuously as the pedestrian closes — torso at $1.8\text{ m}$, thigh at $1.2\text{ m}$, shin at $0.5\text{ m}$. That is a slow, monotonic bias on top of the oscillation, and it is inseparable from real approach motion by any filter operating on $Z$ alone. It is also a further reason the visual gate never worked: Idea A-34's projection compares a legs-only depth centroid against a full-body YOLO `person` box whose vertical extent is dominated by a torso and head the depth frame physically cannot see.

  This is distinct from everything already logged. Idea A-38 addresses the **floor** merging into the blob from below; this is the **pedestrian's own body** being cropped from above. Idea J-20 replaces the median with a histogram modal peak — which at these ranges locks onto the *legs*, since they dominate the pixel count once the head is out of frame, so J-20 does not fix it either.
- **Proposed Solution:** Select a fixed band of *physical height* instead of taking the whole blob, using the row vector A-38 already builds.
  1. **Reuse $k_v$.** A-38 precomputes $k_v = (v - c_y)/f_y$ once as an $(H,1)$ vector to reject the floor. The same vector gives each pixel's height, $h(v, Z) = z_{\text{cam}} - Z\,k_v$ with $z_{\text{cam}} = 0.185$. Inside the per-blob gather at lines 272–274, prefer the trunk band:
     ```python
     h_px  = self._z_cam - depth_slice * self._k_v[y_b:y_b+h_b]
     trunk = (h_px >= 0.9) & (h_px <= 1.5) & (cnt_mask == 255)
     sel   = trunk if trunk.sum() >= 40 else top_band
     Z     = float(np.median(depth_slice[sel]))
     ```
     where `top_band` is the fallback for close range: the top $0.35\text{ m}$ of whatever is visible, $h \in [h_{\max} - 0.35,\ h_{\max}]$, with $h_{\max}$ read directly from the blob's own topmost row so no extra pass is needed. The top of a cropped body is always the part nearest the trunk and least affected by leg swing.
  2. **Why a height band and not a depth band:** because the selector is defined in metres of height, the estimator measures the *same physical part of the body at every range*. That removes consequence (3) — the migration — outright, not just the oscillation, which is the larger of the two errors over an approach from $1.8\text{ m}$ to $0.5\text{ m}$.
  3. **Hard precondition: Idea A-42.** This consumes measured $f_y$ and $c_y$, and A-42 shows the driver currently reports CAM_A's intrinsics for a CAM_B-aligned frame — with $c_y$ wrong by $120\text{ px}$ the band would be placed on the wrong rows entirely. Fail closed exactly as A-38 specifies: if intrinsics are unavailable or their declared $(w, h)$ disagrees with the frame, skip the band selection, log once, and fall back to the full-blob median.
  4. **Complementary hardware change: pitch the mount up.** With an upward pitch $\theta_p$ about the camera $x$-axis the height map generalises exactly to
     $$h(v, Z) = z_{\text{cam}} - Z\,(k_v\cos\theta_p - \sin\theta_p)$$
     which reduces to A-38's expression at $\theta_p = 0$. At $\theta_p = 15^\circ$, $h_{\max}(Z) = 0.185 + 0.706\,Z$, so $h_{\max}(1.8\text{ m}) = 1.46\text{ m}$ — chest-height coverage across the whole gate range — and the floor's entry range moves from A-38's $0.40\text{ m}$ to $0.98\text{ m}$, halving the floor footprint A-38 has to mask. At $\theta_p \ge 24.8^\circ$ the floor leaves the image entirely, at the cost of near-field coverage below $\approx 0.6\text{ m}$. This requires a URDF/TF change and re-deriving $k_v$ with the pitch term, so it is the follow-up; the height-band selector above is implementable today with no hardware change and is the right first move.
- **Expected Benefit:** Removes $\pm 0.05\text{–}0.10\text{ m}$ of gait-coherent oscillation from the depth reference — peak $0.90\text{ m/s}$ of phantom velocity, $45\%$ of the `dx` channel's entire training spread — which no idea in this log currently addresses, because every logged smoother (A-39, A-26, A-51) targets noise or steps, and this is a periodic signal sitting inside the passband. Also deletes the systematic torso-to-shin drift of the reference point across an approach from $1.8\text{ m}$ to $0.5\text{ m}$, and prevents Idea A-29's repaired stop gate from firing twice per stride on a walking pedestrian. Cost is a reuse of A-38's existing $(H,1)$ vector and one boolean mask per blob; the optional $15^\circ$ mount pitch additionally halves the floor region A-38 exists to remove.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-54`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
