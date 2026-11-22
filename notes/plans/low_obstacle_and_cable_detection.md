# Low obstacles and floor cables during auto-explore (2026-11-22)

Problem: in the new lab, auto-explore gets stuck on cables and sometimes hits tables.

## Why the current stack can't see them

- Nav2's local costmap and the CBF see only `/scan`. On the AGX that is
  `livox_scan_node` slicing the Mid-360S cloud at **z = 0.31 ± 0.05 m**.
  Anything entirely below ~0.26 m (table/truss base bars, chair star bases,
  caster feet, cart shelves, power strips, cables) is not in `/scan` at all.
- The ToF arrays that covered the low band are off the chassis, so nothing
  covers it right now (local costmap still lists `tof_upper tof_lower`, which publish nothing).
- The Livox's lowest ray is -7.5°, so from 0.31 m it sees an object of height
  h only beyond d = (0.31 - h) / tan 7.5° ≈ (0.31 - h) / 0.132:
  a 5 cm bar from ~2.0 m out, a 15 cm bar from ~1.2 m. Inside that it is blind.
  The cloud already sees these bars as the robot approaches; we throw them away
  in the slice.
- A 5–10 mm cable is below anything the lidar or stereo depth can separate from
  the floor. Only the cameras can see it.

The live front-camera frame from today shows it: cables along the wall, the
truss base, and chair star bases are all low-band obstacles.

## Plan, cheapest first

### 1. Livox low band into the costmap (no new sensor, ~1 day) — fixes "hits tables"
Add a height-filtered cloud source to the local (and global) costmap obstacle layer:
- New topic `/livox/obstacles`: points with 0.05 m < z_base < 0.45 m
  (robot height plus margin), range 0.2–4 m, voxel-downsampled to 5 cm,
  in `base_footprint`. Produce it in `livox_scan_node` from the cloud it already decodes
  (it already parses the raw CDR bytes, which is cheap).
- Costmap source: `marking: True, clearing: False`, `min_obstacle_height 0.04`.
  Mark-only matters: once the robot is inside the blind radius the bar is no
  longer seen, and a clearing source would erase it. `/scan` raytracing still clears
  cells it passes over, the same arrangement the ToF used.
- Floor false positives: the floor is z≈0 ± 2 cm in the floor fit; 0.3° pitch
  error is 1.3 cm at 2.5 m. 0.05 m holds; check with a recorded lab bag first.
- Also feed the same points to the CBF obstacle set (it already merges ToF points).
- Frontier explorer: after this, reject frontiers whose centroid is in lethal space
  on the costmap, not only on `/map`.

### 2. Camera floor-hazard segmentation (cables) (~3–5 days)
Monocular depth is the wrong tool here: Depth Anything-style models give
relative, smoothed depth, and a 5 mm cable is at or below their resolution. The
floor is a known plane, so 2D segmentation plus the ground-plane homography gives
metric positions with no depth at all:
1. Segment "cable / cord / power strip" in the front camera image.
2. Back-project each mask pixel onto z = 0 using the camera's calibrated
   intrinsics and extrinsics (URDF, front OAK re-solved 2026-11-20). Error grows
   with range; keep only ≤ 2.5 m.
3. Publish `/floor_hazards` (PointCloud2, `odom` frame, z = 0.02) as a mark-only
   costmap source with a slightly larger inflation, the same as the Livox low band.

Model path:
- Zero-shot first: YOLOE (open-vocabulary seg) prompted "cable, wire, power cord".
  Not yet run: a first attempt on 3 live frames timed out downloading the
  text encoder. Frames grabbed from ws://orin:8081 work as test input.
- Then a dedicated model: drive the lab with the camera recording, auto-label
  with YOLOE or Grounding-DINO+SAM2 on REAL-1, hand-fix ~200 frames, and fine-tune
  yolo11n-seg with one or two classes. Run it on the AGX GPU (TensorRT FP16;
  there is headroom) rather than the OAK VPU, which is already running detection.
- Optional cross-check: OAK stereo depth minus the fitted floor plane. A > 8 mm
  bump inside 1 m confirms thick cords and power strips; it does not see thin ones.

### 3. Stuck detection and recovery (safety net, ~0.5 day)
Mecanum rollers grab cables. Even with 1 and 2 something will be missed:
- In `server_x3.py`/frontier explorer: commanded speed > 0.1 m/s for 1.5 s while
  odom/FAST-LIO displacement < 3 cm, or wheel current spike -> stop, back out
  along the last 0.3 m of path, mark a 0.3 m lethal disc at the stuck point in a
  keep-out layer, blacklist that frontier, continue.
- The driver already has a 1.5 s stall cut-off (b25b2a6); this hooks recovery onto it.

## Order
1 first: it is cheap and fixes the table/base-bar collisions. 3 next. 2 only
if cables still cause trouble after 3. 2 is the only one needing data collection.
