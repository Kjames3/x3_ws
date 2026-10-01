---
name: urdf-nudge
description: Add a part or mount to the X3 URDF, or move one by a measured amount, and let the user check it in RViz. Use when the user says a mount is installed and its STL is in the stls folder, or asks to "move it up/down/forward/back N mm".
---

# Placing parts in the URDF

The user designs a mount, installs it, drops the STL in `stls/`, and then
iterates on placement by eye in RViz: "move it down 0.5 mm, forward 0.5 mm".

## Files

- `src/yahboomcar_description/urdf/yahboomcar_X3.urdf` is what every launch
  file loads. Edit this one.
- `yahboomcar_X3.urdf.xacro` is NOT the source of the `.urdf`: it has 12
  joints against the `.urdf`'s 31. The `.urdf` is hand-edited. Do not
  regenerate it from the xacro.
- Meshes go in `src/yahboomcar_description/meshes/` and are referenced as
  `package://yahboomcar_description/meshes/<name>.STL`. Copy the STL from
  `stls/` and commit it (the large-file guard allows up to 30 MB).

## Loop

1. Restate the move in metres and in robot axes before editing:
   +X forward, +Y left, +Z up, in the PARENT link's frame. A mount attached to
   a rotated link moves along that link's axes, not the robot's. If the joint
   has a non-zero `rpy`, work out which component "down" really is.
2. Edit the joint `<origin xyz=...>`. Millimetres to metres: 0.5 mm = 0.0005.
3. The user previews on the laptop:
   ```bash
   bash scripts/laptop_sim.sh view
   ```
   They run this themselves and report back; keep going until they say the
   position is correct.
4. `python3 -m pytest tests/test_robot_constants.py -q` checks the tree is
   still one tree, every mesh exists, and the settled constants did not move.
5. Commit only when the user says the position is right.

## Do not move these

- `base_joint` z = 0.0815 (caliper-measured). Sensor joints were adjusted to
  keep their measured floor heights; changing it shifts everything.
- `laser_joint` yaw = pi. Raw laser +X points at the robot's rear.
- ToF pitches: upper 15 deg, lower 40 deg down.

## After a sensor moves

Geometry in the URDF is not the only copy. Re-measure what depends on it:

- Camera moved: `config/camera_ground_plane.json` (run
  `src/floor_plane_validate.py`). The measured lens height (0.213 m) is used
  for ground rejection, not the URDF-derived one.
- ToF bracket touched: re-record `config/tof_floor_baseline.json` with
  `src/tof_floor_baseline.py` (robot still, clear floor).
- The robot needs `colcon build --packages-select yahboomcar_description` and
  a restart before `/tf_static` reflects the change.

## SolidWorks export

The existing meshes are exported in millimetres and loaded with
`scale="0.001 0.001 0.001"`; without it a part shows up 1000x too large. Check
the export's origin is the mounting face you are positioning by.
