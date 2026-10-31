# Raised purple box

User references: approximately 250 mm wide, 1240 mm from camera before raising,
75 mm support height. No new horizontal distance measurement supplied after lift.
60 complete scored pairs, audit failures empty. ROI (136,149,250,318) selected
from rectified RGB independently of depth. Strap protrusions and center support
remain confounders; this is not a precision calibration target.

Raw-depth segmentation seed 1381 mm is the approximate inverse of the known
correction at 1240 mm, not a second physical distance measurement. Existing
alignment CLI consumes raw depth. Corrected interior median in first frame:
1233.9 mm, consistent with approximate tape reference; not an accuracy bound.

Ten sampled pairs: median signed offsets left -7, right +5, top +3, bottom +5 px.
Per-edge p95 absolute upper errors 12.5,15.5,7.5,10.5 px respectively. Bottom
measurable fraction 94.5%; all-edge gate still fails (5px p95 requirement).
Source bag remains on robot at artifacts/oak-colored-cloud-2026-10-30/
purple-box-raised-01. No calibration, application or camera settings changed.

Next independent work: offline colored-cloud prototype with explicit edge/support
rejection and retained-point accounting. Do not relabel a filtered interior pass
as full-image registration qualification or deploy until its limits are measured.
No more target adjustments needed for that offline experiment.
