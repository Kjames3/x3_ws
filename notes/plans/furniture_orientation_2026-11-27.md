# Reality view furniture yaw · 2026-11-27

## Enable

Open the GUI with `?furnitureYawV2=1` and turn on Reality structure in Surroundings. The browser remembers the flag in `localStorage.furnitureYawV2`. Use `?furnitureYawV2=0` to restore the default estimator. The flag affects fitting and tracking in the browser. Session landmarks still use the existing server schema and a corrected heading can be restored after refresh.

## Findings and implementation

- `furnitureSurfaces` finds tabletop and seat patches in 12 cm columns. The existing table fit uses PCA and then searches for a smaller rectangle over the occupied columns. `pointBounds` trims extents in the chosen frame. Partial tops can skew the estimate, and small changes in coverage can switch which side is longer. V2 searches minimum trimmed area on the tabletop voxels in 3° steps. Its fourth-angle mean fuses the two orthogonal axes. Consistently rectangular fits can also establish which of those axes is the long side.
- The existing lidar chair fit uses the raised cells on one side of a seat. Camera-labeled couches and chairs with no fitted surface can instead take their heading from the whole cluster's PCA, or from the camera bearing. Neither geometric fallback identifies the back. V2 fits a line through elevated returns and uses the observed seat on the open side to resolve its direction. Narrow high patches, ambiguous open sides and tall arms with a seat span wider than plausible couch depth carry no vote.
- The legacy camera path can hold heading by the fourth detection and has no sustained-turn path. Commit `ac47d9b` added a separate 25° / 20-fit rule for lidar-only landmarks. V2 uses a bounded circular mean of distinct clouds, discounts repeated observations from the same bearing, and accepts a sustained, coherent correction for established or restored objects. It also discards the old lookalike voxels when the box frame turns so they can rebuild in the corrected frame.
- The tracker already converted fitted base yaw to map or odom. The base pose used for the conversion was the newer GUI readout. V2 uses the paired Livox capture pose, composes it with the readout's map-from-odom transform, and projects tracked objects to the readout base for rendering. If the cloud arrives first, it waits for the matching pose before counting the cloud's yaw.
- The wall prior is local. A measured nearby wall can refine a tabletop heading within 8° when its axes support a rectangle within 2.5% of the best area. In the voxel test it reduced an aligned tabletop's 0.8° quantization error to 0° and left a tabletop angled 33° to the wall at its measured heading. A stronger room-grid constraint would override deliberately angled furniture.
- `src/oakd_cloud.py` back-projects depth. `src/oak_surroundings.py` publishes a colored cloud in odom, which the GUI can fuse into a lookalike. OAK spatial detections provide a label and a base-frame point. `server_x3.py` stores browser-supplied landmark yaw. The server-side paths do not estimate furniture heading.

## Paired replay

I found no recording pairing the GUI's Livox furniture voxels and capture poses with known furniture orientations. I checked `artifacts/`, `tests/data/`, `~/bags`, and metadata in the older `~/EE_244_Final_Project/bags` corpus. The saved OAK CDR decodes to a single cloud of 5,504 colored points. Both estimators ran on its 799 voxels at 5 cm and proposed one unconfirmed candidate. The navigation bag in `~/bags` has odometry and a planar scan. Neither supplies a reference furniture heading or a sequence of the GUI's Livox clouds.

Run `node tests/web/reality_furniture_yaw_check.cjs`. The paired test uses 16 sequences per kind, 48 frames per sequence, the last 16 frames for scoring, and the same clouds in both runs. The inputs include square and rectangular tables, partial faces and diagonal occlusion, temporary missing backs, noisy dropped returns on 2.5 cm and 5 cm grids, a moving observer, and 8–16° readout lag on half the sequences. It exercises labeled and lidar-only tracking. Errors use 90° symmetry for square tables, 180° for rectangular tables, and full directed yaw for seats.

| Furniture | Median yaw error, current → V2 | Within 10°, current → V2 | Flips, current → V2 |
| --- | ---: | ---: | ---: |
| Tables | 10.3° → 0.5° | 50.0% → 100% | 0 → 0 |
| Chairs | 7.1° → 0.4° | 68.8% → 100% | 0 → 0 |
| Couches | 19.6° → 0.0° | 37.5% → 100% | 0 → 0 |

All 256 scored frames per kind showed the object in both versions. A flip is a consecutive-frame yaw change above 30° for a square tabletop or 45° for other pieces after applying the indicated symmetry. This measures discontinuity in stable scenes. A separate case corrects a restored couch with a 180° error using distinct paired clouds, keeps its ID, and rejects duplicate clouds and a tall arm with a hidden back. The Chrome capture shows that same class of 180° error in the production render path. [Synthetic couch comparison](../../artifacts/surroundings-2026-11-27/furniture-yaw/before-after.png)

On the same replay at 10 cm voxel size, the V2 medians rose to 1.5°, 0.9° and 0.7° for tables, chairs and couches. All scored poses were within 10°. Forcing V2 to use the later readout pose in the fine-grid replay increased those medians to 6.8°, 12.5° and 9.0°. The paired pose matters even when the geometry fit is good.

Existing Reality and Surroundings Node regressions passed. The unsourced `python3 -m pytest -q tests/` run matched baseline with 1,095 passed, 30 skipped, 3 xfailed, 4 failures, and 8 URDF setup errors.

## Live check

Use the actual apartment dining table and ladder-back chairs, plus the lab couch. Walk two sides of each piece with the GUI at its default 5 cm voxel size. Compare wall-aligned and deliberately angled pieces, inspect the held pose while the robot turns, and verify back direction after temporary occlusion and after reloading a map landmark. A side-only view or a back flush with a wall can leave the initial heading provisional because the estimator requires a visible raised strip and enough seat returns to establish its open side. Check that pose pairing is available at the live cloud rate and that the lookalike rebuilds after a correction.
