# Purple box registration diagnostic

Current Pro W, 384x512. User reports approximate width 250 mm; camera-to-face
distance not yet supplied. Capture on robot:
~/x3_ws/artifacts/oak-colored-cloud-2026-10-30/purple-box-01.
60 scored pairs, audit failures empty, p95 timestamp skew 14.91 ms,
estimated depth age p95 153.6 ms. No camera settings/application edits.

ROI (142,181,254,347) selected from rectified RGB before looking at depth.
Raw depth interior median 1385 mm, corrected 1243.6 mm. **1385 mm passed to the
alignment tool is only a segmentation seed derived from sensor depth, NOT a
user-measured ground truth.** Its target_distance_mm field must not be treated
as a reference measurement. No depth accuracy verdict is possible yet.
Nominal width-derived range (~1.21 m with 112px and fx=541.56) is a rough
consistency check only; box width is approximate and orientation is unmeasured.

Ten sampled frames: median left edge offset -9 px, right +9 px, top +1 px.
These are opposite outward offsets, not a single translation. Side depth
silhouette broadening is consistent with stereo boundary effects, but cause is
not established. Top p95 upper error 26.5 px despite small median; a hand/ruler
is present near the top in the RGB image and may confound this edge. Bottom has
no unique depth transition in all scored samples (floor contact).
Existing provisional 5px p95 edge gate fails. Do not declare RGB-depth alignment
qualified. Next capture should have no ruler/hand at boundary, independent
camera-to-front-face distance, and target raised on a narrower support so the
bottom edge has a distinct background. Test interior-only color validity and
edge rejection offline before live publishing.

User subsequently measured approximately 1240 mm camera-to-front-face distance,
and raised the target on a 75 mm tall box. Width remains approximately 250 mm.
1240 mm agrees approximately with the first corrected interior estimate, but
this is one placement with approximate tape references, not a calibration fit.
