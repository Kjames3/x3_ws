# C3 person depth: box clipping and histogram mode (2026-11-21)

Tool: `src/tools/perception/c3_depth_roi_eval.py RUN...` (person-lidar runs with `c3/c3.ndjson`,
`c3/c3_depth.bin`, `lidar_persons.ndjson`). Re-measures every logged post-NMS box on its recorded
depth frame (stored at step 2, repeated back to the full grid) and compares with the Livox candidate on
the box's bearing. Sanity gate: the `old` variant matches the logged live z to 0-4 mm median.

Reference caveat: the lidar candidate centroid sits ~0.1-0.15 m behind the surface the camera sees, so
every number below flatters the camera by that much. Seated candidates are sparse on some runs (n shown).
No fx / depth-scale correction is applied anywhere. Walk runs 11-20 have boxes mostly off the left and top
of the image, so their rows are weak evidence.

Median camera-minus-lidar range (m), n matched boxes in brackets:

| run | old (unclipped window) | clip | clip + hist60 | clip + hist100 |
|---|---|---|---|---|
| 11-20 seated-r1 | +0.86 (114) | +0.41 | +0.40 | +0.42 |
| 11-20 seated-r2 | +0.89 (160) | +0.42 | +0.38 | +0.36 |
| 11-20 bias-check-r1 | +0.79 (53) | +0.55 | +0.43 | +0.41 |
| 11-20 seated-r3 | +0.37 (3) | +0.54 | +0.20 | +0.21 |
| 11-20 walk-r1 | -0.61 (12), yield 0.21 | -0.11 (17), yield 0.27 | +0.14 (73), yield 0.60 | -1.48 (130) |
| 11-20 walk-r2 | -0.12 (22), yield 0.26 | -0.09 (28), yield 0.34 | +0.02 (74), yield 0.57 | -1.66 (130) |
| apartment runs 11-07..11-09 (16 runs) | identical to clip | identical | within 0.02, couch-r1/r2 -0.12/-0.21 | within 0.03 |

Findings:
- Clipping halves the 11-20 seated over-read (+0.86 -> +0.41). In-frame boxes (all apartment runs)
  are unchanged to the millimetre, so it is safe as a default.
- A residual +0.2 to +0.35 m remains on every apartment run too, where nothing is clipped: that is not
  an ROI problem (scale/offset; measured separately).
- hist60 (nearest 5 cm-bin peak holding >= 15 % of valid pixels, top 60 % of the clipped box) adds a
  little on 11-20 seated and doubles walker yield, but its frame-to-frame jitter on seated-r1/r2 is worse
  (0.17 -> 0.29 / 0.22 m median |dz|). In the seated-r1 depth image the person is a ~70 px wide blob
  inside a ~380 px wide box, below the top 60 % rows, so the row cap sometimes misses it.
- hist100 (whole box) is stable on seated runs but reads floor / near objects for the walkers (-1.5 m).
  The floor needs a real ground mask; none exists on this path.
- Do not turn hist on by default.
