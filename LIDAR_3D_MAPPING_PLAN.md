# 3D apartment mapping from the 2D 4ROS lidar + OAK-D Lite

Feasibility study and implementation plan for adapting:

> **Direct 3D mapping with a 2D LiDAR using sparse reference maps**
> ISPRS Open Journal of Photogrammetry and Remote Sensing, 2025.
> DOI: [10.1016/j.ophoto.2025.100109](https://doi.org/10.1016/j.ophoto.2025.100109) (gold OA)

Status: **Phase 0 implemented (§7), Phases 1–4 design only.** Written 2026-08-08.

Phases 1–3 need a tilt bracket, which is deferred — there is a spare **STS3215**
serial bus servo in storage earmarked for it. Note that an STS3215 is a bus
servo, not PWM, so it would use `Rosmaster_Lib.set_uart_servo_angle()`
(`src/Rosmaster_Lib.py:709`) rather than the `set_pwm_servo()` path assumed in
§3.3 — and its position feedback would make the α-estimation caveat in §3.5
much less of a worry.

---

## 1. What the paper actually does

Three moving parts:

1. **Inputs** — a 2D lidar, an initial 6-DoF trajectory (GNSS–INS or 3D
   lidar-inertial odometry), and a *sparse* 3D reference map from airborne or
   mobile laser scanning (ALS/MLS).
2. **Batch optimisation** — buffer a window of 2D scans, transform them into
   the world frame using the trajectory, and jointly co-register that whole
   buffer against the 3D reference map. Refines the trajectory *and* the map
   together. The key claim is that this needs **no scan-to-scan overlap and no
   segmentation** — the reference map does the work that scan overlap normally
   does.
3. **Targetless extrinsic calibration** between the 2D lidar and the other
   sensors, by aligning motion against the 3D map, explicitly **without
   requiring overlapping fields of view**. Converges from 40° / 3 m initial
   misalignment.

Reported: ~0.1 m mean localisation accuracy on forest roads, drift cut ~9× in
translation and ~6× in rotation.

The division of labour is what matters for us: **the reference map is coarse
but globally correct; the 2D lidar is dense and locally accurate but
geometrically crippled.** The optimisation trades one against the other.

> Full text note: Elsevier's bot wall (403 / captcha) blocked automated
> retrieval of the PDF from ScienceDirect, DOAJ, and the Helsinki repository
> mirror, so this plan is built from the abstract, the highlights, and the
> method class. Before implementing Phase 3, someone should pull the PDF
> manually and check §3 for the exact cost-function terms and the buffer/window
> sizing — that's the one place guessing would cost real time.

---

## 2. Two blockers, one of them fatal as things stand

### Blocker A — our scan plane never sweeps (fatal)

`src/yahboomcar_description/urdf/yahboomcar_X3.urdf.xacro:135`

```xml
<fixed_joint name="laser_joint" parent="base_link" child="laser_link"
             xyz="-0.0115 5.25826986680105E-05 0.191" rpy="0 0 3.14159265"/>
```

Roll 0, pitch 0. The 4ROS scan plane is **exactly horizontal** at z = 0.191 m.

The X3 drives on a flat apartment floor. A horizontal scan plane on a
planar-moving platform re-measures *the same horizontal slice of the apartment*
forever. It does not matter how good the optimiser is — there is no 3D
information in that data to recover. Zero.

The paper gets 3D because its 2D lidar is a **push-broom profiler** on a
vehicle: the scan plane is oriented across the direction of travel, and forest
roads pitch and roll the platform. Both conditions fail here.

**So the paper's method cannot produce a 3D map from our lidar as mounted.**
This is a mounting problem, not a software problem. §3 fixes it.

### Blocker B — there is no ALS/MLS reference map of your apartment

Obviously. Nobody flew a laser scanner over your living room.

This one is *not* fatal, and it's where the design gets interesting.

---

## 3. The adaptation

### 3.1 The OAK-D Lite plays the role of the sparse reference map

The paper's reference map is coarse, noisy relative to the 2D lidar, but
genuinely 3D and globally consistent. That is a good description of OAK-D Lite
stereo depth.

`src/oakd_driver.py` already exposes exactly what's needed:

- `get_raw_depth_frame()` → float32 **metres**, 0 = no return (line ~20)
- on-device `StereoDepth` aligned to `CAM_A` (line ~260)
- CAM_A intrinsics read from the device at startup (line ~145)

So back-projecting to a metric point cloud is a few lines against code that
already works.

Depth error for the Lite (baseline 75 mm, f ≈ 450 px at 640×480, ~0.2 px
subpixel) goes roughly as `0.006·z²`:

| range | depth error |
|---|---|
| 2 m | ~2 cm |
| 4 m | ~10 cm |
| 6 m | ~21 cm |

Useful to ~4 m, coarse beyond. Meanwhile the 4ROS is cm-accurate to 10 m. The
accuracy asymmetry is the **same shape** as the paper's ALS-vs-2D-lidar
asymmetry, which is why the method transfers rather than merely being
borrowed.

### 3.2 Tilt the lidar, and let in-place spins do the sweeping

Tilt the scan plane by angle **α** from horizontal. The robot is a mecanum
platform — it can rotate in place.

Ask which points a full in-place yaw spin can reach. The tilted scan plane
through the lidar origin has normal `n(φ) = (sinα·cosφ, sinα·sinφ, cosα)`
where `φ` is robot yaw. A point `p` at horizontal radius `ρ` and height `z`
(relative to the lidar) is measured at some yaw iff `p·n = 0` has a solution:

```
sinα·ρ·cos(φ−ψ) + z·cosα = 0
⟺  |z| / ρ  ≤  tanα
⟺  |elevation of p|  ≤  α
```

**A single in-place 360° spin sweeps every direction within ±α elevation.**
That is real, dense 3D — the classic spinning-tilted-lidar scanner geometry,
available on this robot for the price of a bracket.

Sizing α for an apartment. A target at horizontal distance `d` and height `h`
above the lidar is covered when `d·tanα ≥ h`, i.e. **beyond** a minimum radius
`d ≥ h/tanα`. So a *larger* α buys better near-field vertical coverage; the
cost is that more of the 10 m range is spent pointing at floor and ceiling
instead of down the length of the room.

Lidar sits at 0.191 m. Minimum radius at which each target height is reached:

| α | 0.8 m tabletop (h=0.61) | 1.5 m counter/shelf (h=1.31) | 2.4 m ceiling (h=2.21) |
|---|---|---|---|
| 25° | 1.31 m | 2.81 m | 4.74 m |
| **40°** | **0.73 m** | **1.56 m** | **2.63 m** |
| 55° | 0.43 m | 0.92 m | 1.55 m |

At 25° the whole near field above table height is missed — useless for the
overhang problem that motivates this. At 55° you cover a small room well but
throw away most of the sensor's range. **α ≈ 40°** covers tabletops from 0.7 m
out and ceilings from 2.6 m out, which fits apartment rooms scanned from near
their centre. Tunable.

Point budget per station, spinning at 0.1 rad/s (≈63 s for 360°):

- 4ROS: 20 kHz sample rate ÷ 8 Hz scan = **2500 pts/revolution**
- 63 s × 8 Hz = ~500 scan planes → **~1.25 M points per station**
- consecutive scan planes separated by 0.72° of yaw ≈ **3.8 cm at 3 m range**

That is a dense, high-quality room scan. Spinning faster (0.3 rad/s, 21 s)
drops you to 2.15° / 11 cm plane spacing — noticeably gappy. Prefer slow.

### 3.3 Mode switching, because a tilted lidar can't navigate

A permanently tilted lidar **destroys 2D SLAM and Nav2** — `slam_toolbox` and
AMCL both assume a horizontal scan. So tilt has to be a *mode*, not a mount.

Good news: the Rosmaster board already drives PWM servos.
`src/Rosmaster_Lib.py:408` — `set_pwm_servo(servo_id, angle)`, `servo_id ∈
[1,4]`, `angle ∈ [0,180]`. A single hobby servo on a printed bracket, on a free
channel, with two detents:

- **NAV mode** — α = 0°, horizontal. 2D SLAM, AMCL, Nav2 all behave exactly as
  today. Nothing in the existing stack changes.
- **SCAN mode** — α ≈ 40°. Robot is stationary. Nav2 must be idle.

Hard interlock: **never tilt while Nav2 has an active goal.**

### 3.4 Pipeline

```
per station:
  drive to station          [NAV mode, existing 2D SLAM/Nav2, unchanged]
  stop, tilt to α           [servo]
  spin 360° @ 0.1 rad/s     [accumulate ~1.25M lidar pts + OAK-D depth clouds]
  de-tilt to 0°             [servo]

offline / background:
  OAK-D clouds + 2D SLAM poses  ──→  sparse 3D reference map
  buffered tilted scans          ──→  ┐
                                      ├─ paper's batch co-registration
  per-station 6-DoF pose priors  ──→  ┘   (jointly refine poses + map)
                                          ↓
                                   dense 3D apartment map
                                          ↓
                        ┌─────────────────┴─────────────────┐
                   3D voxel / octomap              static-structure mask
                   → Nav2 costmap layer            → dynamic obstacle filter
```

Note the trajectory prior comes from the robot's existing 2D SLAM — which is
*accurate in-plane* and needs no roll/pitch estimate on a flat floor. That's a
much better prior than the paper's GNSS–INS, so the optimisation starts from a
friendlier place than the forest-road experiments did.

### 3.5 Extrinsic calibration

We need lidar↔camera extrinsics with the lidar tilted 40° down and the camera
looking forward — partial FOV overlap at best, and a servo detent whose
repeatability is ~1–2°.

This is precisely the problem the paper's §3 targetless method solves (no
overlapping FOV required, converges from 40°/3 m). It is arguably the most
directly reusable part of the paper for us.

Pragmatic addition: **treat α itself as a state in the batch optimisation.**
A 1° servo error is 5 cm of height error at 3 m — worth estimating rather than
trusting.

---

## 4. What this actually buys navigation — and what it doesn't

Being blunt here, because the premise in the original ask doesn't survive
contact with the code.

### It does not improve the velocity MLP directly

`src/velocity_estimator.py:363` `_build_window_features()` builds the model
input as `[rel_x, rel_y, dx, dy] × 10 frames = 40 features`, where
`rel_x = cz`, `rel_y = -cx`. The centroid's **height `cy` is taken in and then
discarded**. The model is 2D. Feeding it a 3D map changes nothing about its
inputs, and the existing `.torchscript` weights can't consume height without a
retrain.

### It does help, via three real mechanisms

1. **Static subtraction upstream of the tracker.** Today `_extract_depth_centroids()`
   spawns tracks on walls, table edges, and door frames, and the MLP has to
   *infer* ~0 velocity for them. With a static 3D prior you delete those
   centroids before they ever become tracks. This directly attacks the known
   weak spot — the recorded MLP dead band below 0.4 m/s is exactly the regime
   where static clutter and slow real motion are hardest to tell apart.
2. **Better ego-pose.** Features are in the robot frame and converted via
   `robot_pose_fn`; ego-motion error injects *apparent* velocity into genuinely
   static objects. The paper's whole selling point is trajectory correction —
   9× translation, 6× rotation. That lowers the noise floor the MLP sits on.
3. **Overhang obstacles, which the robot currently cannot see at all.** A scan
   plane at z = 0.191 m sees table *legs* and nothing else. Tabletops,
   countertops, chair seats, open drawers, and the sofa arm are invisible to
   Nav2 today. A 3D costmap layer fixes a real, current navigation failure.

Mechanism 3 is probably the largest practical win and it is worth having on its
own merits, independent of the velocity estimator.

If you *want* the map to feed the MLP directly, that's a retrain with height
and static-prior features — a separate project, and it should be scoped against
the existing velocity-estimator backlog rather than bolted onto this.

---

## 5. Phasing

**Phase 0 — OAK-D depth into the Nav2 costmap. ✅ IMPLEMENTED (2026-08-08).**
See §7 below. No servo, no tilt, no risk to the nav stack; delivers mechanism 3
(overhangs), which is the standalone win.

**Phase 1 — tilt bracket.** Print/mount, wire to a free Rosmaster PWM channel,
add NAV/SCAN mode API with the Nav2 interlock. Verify NAV mode reproduces
today's 2D SLAM behaviour before touching anything else.

**Phase 2 — stop-and-spin capture.** Station routine, raw capture to disk,
visual sanity check of a single-station cloud. A single good station scan is
the go/no-go gate for Phase 3.

**Phase 3 — the paper's method.** Batch co-registration of buffered tilted
scans against the OAK-D reference, jointly refining per-station poses and α.
Pull the PDF first.

**Phase 4 — consume.** 3D costmap layer + static mask into the velocity
estimator's centroid extraction.

---

## 6. Risks and gotchas

- **Self-returns come back.** `LIDAR_MOUNT_PLAN.md` records that the lidar was
  deliberately *raised* to z = 0.191 so the horizontal plane clears the
  chassis. Tilting 40° down puts the floor intercept at `0.191/tan40° ≈ 0.23 m`
  radius — inside the chassis footprint. Expect chassis and wheel returns and
  budget for an angular/range mask in SCAN mode. Do **not** apply that mask in
  NAV mode.
- **Grazing incidence on the floor.** A single-channel TOF lidar hitting a
  hard floor at a shallow angle returns poorly. Consider tilting *up* instead
  of down — ceilings and upper walls are the structure Nav2 is missing anyway,
  and the floor is already well covered by the horizontal NAV-mode scan.
- **Servo repeatability** ~1–2°, i.e. 5–10 cm at 3 m. Mitigate by estimating α
  in the optimisation (§3.5), not by trusting the detent.
- **Nav2 interlock is safety-critical.** A tilted lidar mid-goal means Nav2 is
  planning against garbage.
- **Missing libs on the Jetson.** `open3d` and `small_gicp` are both absent;
  `numpy`, `scipy`, `sklearn`, `cv2`, `torch`, `depthai` are present, and ROS
  ships `octomap`, `pcl_conversions`, `depth_image_proc`. Either install a
  registration lib or implement ICP/GICP on `scipy.spatial.KDTree`.
- **One thing that gets *easier*:** the variable-beam-count problem that forces
  the `scan_resampler` → `/scan_fixed` workaround for Karto is a 2D-SLAM
  constraint only. 3D point accumulation does not care how many beams a scan
  has. Consume raw `/scan` in SCAN mode.

---

## 7. Phase 0 as built

Goal: let Nav2 avoid the things the horizontal scan plane cannot see. Not a
persistent 3D map — a live camera obstacle source. That is the whole of the
navigation benefit (mechanism 3) without any hardware change.

### Running it

```bash
python3 src/server_x3.py --domain-id 42 --oak-cloud
```

`--oak-cloud` is independent of `--oak-ros-publish`: it adds `/oak/points` only,
skipping the image topics, which exist for bagging and cost real CPU and USB
bandwidth. Tunable with `--oak-cloud-rate` (default 5 Hz) and
`--oak-cloud-max-range` (default 4.0 m).

### What changed

| File | Change |
|---|---|
| `src/oakd_cloud.py` | **new** — depth → `(N,3)` back-projection + `PointCloud2` builder + `scale_intrinsics()`. ROS-free math so it tests off-robot. |
| `src/test_oakd_cloud.py` | **new** — 12 offline tests: projection round-trip, stride indexing, range gating, cap, intrinsics rescaling. |
| `src/oakd_driver.py` | adopts the robot's untracked depth-intrinsics version (below) — a rescue, not a rewrite. |
| `src/oakd_ros_publisher.py` | publishes `/oak/points`; `publish_images` now separable so cloud-only is cheap. |
| `src/server_x3.py` | `--oak-cloud`, `--oak-cloud-rate`, `--oak-cloud-max-range`. |
| `nav2_params_x3.yaml` | `oak_voxel_layer` (`VoxelLayer`) on the **local** costmap. |

### The intrinsics question — already solved on the robot

The committed `oakd_driver.py` read **CAM_A at 480×640** unconditionally, which
is only correct when the spatial-NN branch is built; without it the stereo node
aligns depth to CAM_B/CAM_C at mono **640×400**, so the intrinsics belonged to
the wrong camera at the wrong resolution and every back-projected point would
have been skewed.

That was **already fixed in the robot's working copy** — more thoroughly than
the fix originally written here. `_read_intrinsics(device, with_spatial)` now
reads a separate set of depth intrinsics for whichever socket and size the
pipeline actually configured, and exposes them via `get_depth_intrinsics()`
returning `(fx, fy, cx, cy, w, h)`.

Two things follow:

1. That version is now committed here. It had been living **untracked** on the
   robot (`git status` showed `?? src/oakd_driver.py`), so it existed in exactly
   one place and would have been lost with the SD card. Adopted wholesale rather
   than reconciled — it is the better implementation.
2. Per its docstring ("consumers that downsample must scale these"), the
   rescaling lives in the consumer: `oakd_cloud.scale_intrinsics(intr, shape)`.
   Normally a no-op; it exists so a depth frame arriving at an unexpected size
   produces a correct cloud rather than a plausible-looking stretched one.

`/oak/depth/camera_info` now advertises the depth intrinsics too, instead of
CAM_A's.

### Design decisions worth knowing

- **Points are published in the optical frame**, not pre-transformed. The URDF
  already defines `base_link → oak-d-base-frame → oak_rgb_camera_optical_frame`
  (camera at z = 0.185, level, facing forward), so TF places the cloud and the
  costmap's `min/max_obstacle_height` do the height filtering *after* that
  transform. No new static publisher, no duplicated extrinsics.
- **Local costmap only.** The camera's ~70° FOV can only raytrace-clear what it
  currently sees, so on the non-rolling global costmap a bad mark persists after
  the robot turns away and can permanently block a doorway. The local costmap is
  rolling, so bad marks age out. Cost: the planner may still route through a
  table and the controller deviates locally. Revisit only with a doorway test.
- **`mark_threshold: 1`** — a column needs >1 marked voxel to become lethal, so
  isolated stereo speckle does not invent obstacles.
- **`min_obstacle_height: 0.05`** — floor deadband. Without it, stereo noise on
  a flat floor marks the ground and the robot refuses to move.
- **Mark to 3.0 m, raytrace to 4.0 m.** Far points are trusted to *clear* stale
  cells but not to *create* obstacles, because `0.006·z²` is ~8 cm at 3.5 m.
- **Degrades safe.** No OAK-D, or server started without `--oak-cloud`, and the
  layer contributes nothing; lidar avoidance is exactly as before.

### Verified

Offline: 12/12 tests pass; `PointCloud2` round-trips byte-exact through
serialization (19200 pts → 230400 B data, `point_step` 12, `array('B')` fast
path); Nav2 YAML parses with the layer and source resolving; conversion costs
**0.26 ms/frame** at defaults (~0.13% of one core at 5 Hz), so it stays clear of
the OAK driver and velocity estimator already topping the Jetson CPU profile.

**Verified on hardware 2026-08-09** (jetson @ 10.13.245.167, Nav2 running
against `apartment3_test`, robot stationary):

- `/oak/points` — 57 msgs / 15 s (~3.8 Hz), frame `oak_rgb_camera_optical_frame`,
  8000 pts/msg (the cap), `point_step` 12, z spanning 0.50–3.96 m.
- TF `base_link → oak_rgb_camera_optical_frame` = (0.0435, 0.0001, 0.1850),
  exactly the URDF value.
- `local_costmap` **subscribes to `/oak/points`**; `/local_costmap/voxel_grid`
  publishes; `/scan` unaffected (57 msgs / 8 s).
- Depth intrinsics read as CAM_A @ 480×640 fx=615.6 fy=615.5 cx=242.7 cy=324.6.
- **Camera-only obstacles confirmed**: 570 of 700 marked voxels sit above the
  0.191 m lidar plane, i.e. structure the lidar physically cannot see.
- **No regression**: A/B via `ros2 param set … oak_voxel_layer.enabled` gave
  77.2% lethal without the layer vs 77.5% with it — +0.3%.

Two things the hardware run changed:

1. **`min_obstacle_height` 0.05 → 0.12.** The original default marked the floor:
   3123 of 8000 points (39%) landed in the 0.05–0.10 m band at 0.78 m median
   range. It was masked in a cluttered room whose cells were already lethal from
   the lidar, but in open space it would carpet the area ahead with phantom
   obstacles. After the change the 0.00–0.10 m bin is empty and the camera-only
   marks above the lidar plane remain.
2. **Decoding `/local_costmap/voxel_grid` is easy to get backwards.** nav2 packs
   a column into one `uint32`: **upper 16 bits = MARKED, lower 16 = UNKNOWN**
   (`voxel_grid.hpp`, `getVoxel`). Reading it the other way makes every column
   look marked, because nearly every column has *unknown* voxels.

Operational gotcha: `stop_nav2` leaves the composed `nav2_container` running,
and a relaunch on top of the stale one comes up with **zero publishers** and no
error. Kill the container by PID before relaunching. Consistent with the
existing note that `use_composition` hides failures.

## 7b. How to test Phase 0

First, a framing correction that saves wasted effort: **Phase 0 builds no 3D
map.** There is no accumulated structure anywhere. What exists is `/oak/points`,
a per-frame cloud that is discarded on the next frame, and a 16-bin x 0.10 m
voxel *local* costmap that rolls with the robot. So the thing to measure is
obstacle detection and clearing quality, not map quality. Anything phrased as
"how good is the 3D map" has no answer yet.

Two tools do the measuring:

- `scripts/oak_obstacle_check.py` — for one angular sector, reports the nearest
  lidar return, the nearest OAK return, the nearest OAK return *inside the
  voxel layer's height gate*, and the nearest lethal costmap cell. The
  in-gate number is the one that predicts marking; the raw nearest return is
  almost always the floor and is meaningless on its own.
- `scripts/oak_floor_check.py` — bins the forward cone by range and reports the
  reconstructed floor height per bin, then fits a line. This catches the failure
  mode that would quietly wreck everything: a floor that tilts up with range
  crosses `min_obstacle_height` and turns into a phantom wall.

**Trap, learned the hard way.** The 4ROS is mounted `rpy="0 0 pi"`, so
laser-frame 0 deg points *backwards* in `base_link`. Any script that compares a
lidar bearing against a camera bearing must put both through TF first.
Skipping that produces confident, coherent, completely wrong numbers.

**Second trap.** Estimate the floor with a low percentile (p25), not the median.
Furniture standing on the floor drags the median up and fabricates a pitch
error — a first pass here reported a 3.12 deg tilt that did not exist. The p25
fit on the same data gave 0.03 deg.

Baseline measured 2026-08-10, robot stationary in the apartment:

| range bin | floor p25 |
|---|---|
| 0.4-0.8 m | +0.048 m |
| 1.2-1.6 m | +0.037 m |
| 2.4-2.8 m | +0.041 m |
| 3.2-3.6 m | +0.024 m |

Fit `z = +0.0006 * range + 0.0439`, i.e. **0.03 deg of tilt — flat**. The depth
extrinsics are sound and there is no phantom-floor risk inside the 3.0 m
`obstacle_max_range`.

One real defect remains: the floor reconstructs **~4.5 cm above ground** where it
should read 0. That constant bias eats about 37% of the margin between the floor
and the 0.12 m gate. It is safe today but it is the reason the gate could not be
lowered to catch genuinely low obstacles. Worth chasing to the camera's `z` in
the URDF before ever lowering `min_obstacle_height`.

A/B with `ros2 param set /local_costmap/local_costmap oak_voxel_layer.enabled
<bool>` confirmed the layer contributes: 21 lethal cells in the forward sector
with it on, 7 with it off. Always restore `true` afterwards.

Still untested, and the tests that matter most next:

1. **Camera-only obstacle.** Put a real overhang — a table edge, a shelf, a
   chair seat — where the lidar plane passes under it. Success is
   `oak_obstacle_check.py` printing `CAMERA-ONLY OBSTACLE` with the costmap
   marking at the tape-measured range.
2. **Clearing.** Remove that obstacle and time how long the marks survive. This
   is the likeliest failure mode for camera marking, and the reason the layer
   was deliberately kept off the global costmap.
3. **Behaviour.** Send a Nav2 goal past the overhang with the layer on and off.
   The whole point is that the robot routes around something it used to hit.

## 8. Honest bottom line

The paper is a good match *in structure* — coarse-3D-anchor plus
dense-2D-lidar is exactly our OAK-D plus 4ROS situation, and its targetless
no-overlap calibration solves a problem we would otherwise have to invent a
solution for.

But it cannot be applied to this robot as currently built. The horizontal mount
means there is no 3D signal to recover, and that is a bracket-and-servo
problem, not an algorithm problem. Anyone who tries this in software alone will
burn a week before noticing.

Recommendation: **do Phase 0 now** — it is software-only, it delivers the
biggest concrete navigation win (overhanging obstacles), and it builds the
infrastructure the rest needs. Treat Phases 1–3 as a genuine project gated on
whether a tilt bracket is acceptable.
