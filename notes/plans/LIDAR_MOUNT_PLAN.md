# Lidar Raise + URDF Update Plan

**Goal:** Raise the YDLidar X3 on a mounting bracket above the chassis so it no longer
sees the robot's own body, then update the URDF and roll back the temporary obstacle-filter
workarounds. Once done, the CBF safety filter works with normal parameters and the robot
both stops for real obstacles **and** threads doorways.

**Status:** Planned — execute after the bracket is fabricated and the mesh/dimensions are
available. Created 2026-06-20.

---

## 1. Why this is needed (background)

The YDLidar is mounted low at the **front** of the robot (`laser_joint` at z = 0.11 m,
x = +0.0435 m from `base_link`). Because it sits below the top of the chassis, the rear and
side beams hit the robot's own body, and two fixed sensor/mount posts appear at **+24° and
−18°, ~0.26–0.35 m**, almost straight ahead. Measured self-returns span **~0.25–0.43 m** at
many bearings.

The manual-drive **CBF safety filter** (`src/cbf_filter.py`, fed by `ROS2Bridge._scan_cb`)
treats every return within `safe_distance` as an obstacle. The robot's own body therefore
either freezes translation (when `safe_distance` overlaps the body shell) or, if we filter
the body out by range, opens a blind zone so the robot drives through real walls. **No
range/cone threshold can separate the body from real obstacles**, because they occupy the
same distances and bearings. Raising the lidar removes the body from view entirely and
dissolves the whole problem.

---

## 2. Temporary workarounds currently in the code (to be rolled back)

These were added as stop-gaps and should be reverted once the lidar is raised. All in
`src/server_x3.py` (mirror any change to the robot copy at
`/home/jetson/x3_ws/src/server_x3.py` — the Jetson git checkout is diverged from local).

| Location | Current (temporary) | Restore to (after raise) |
|---|---|---|
| `_scan_cb` range gate | `0.33 < r < 1.0` | `~0.12 < r < 1.0` (lidar's true `range_min`) |
| `_scan_cb` angular cone | `abs(angle) <= (_math.pi / 2)` (front 180°) | widen — full 360° (drop the cone) or `<= 2.35` if minor rear blockage remains |
| `HolonomicCBFFilter(...)` | `safe_distance=0.30, gamma=1.0` | keep `~0.30` (tune); now safe because no body in view |

> Unrelated and **should stay**: the `X3_SIMPLE_DISCOVERY` discovery changes and the
> `scripts/jetson_simple_discovery.sh` drop-ins. Those fix cross-machine RViz, not the lidar.

---

## 3. Hardware step (you)

Fabricate a bracket that raises the lidar's scan plane above the tallest part of the chassis
(camera, boards, wiring). Keep the lidar's **optical/scan origin** as close to the robot
centerline (x≈0, y≈0) as practical to simplify kinematics and keep 360° coverage symmetric.

---

## 4. What to provide when the bracket exists

So the URDF can be updated precisely, capture:

1. **Bracket mesh** — STL (or DAE) of the bracket, exported in **meters**, origin at the
   bracket's mount face. Drop it in
   `src/yahboomcar_description/meshes/sensor/` (next to `laser_link.STL`).
2. **Placement of the bracket on `base_link`** — `xyz` (m) and `rpy` (rad) of the bracket's
   base relative to `base_link` origin.
3. **Lidar scan-origin offset within the bracket** — `xyz`/`rpy` from the bracket mount face
   to the lidar's actual scan plane (the YDLidar's optical center, not the case bottom).
4. **New lidar height** — final z of the scan plane above `base_link` (e.g., 0.20–0.30 m).
5. **Lidar yaw** — confirm the physical "0°/forward" mark direction so we can set/verify the
   joint `rpy` yaw (see §7 — the current joint uses yaw = π).

---

## 5. URDF changes

**Active file (loaded by `x3_bringup.launch.py:44`):**
`src/yahboomcar_description/urdf/yahboomcar_X3.urdf` — loaded from the package **share**
dir, so a `colcon build` of `yahboomcar_description` is required to deploy (see §6).
**Also edit the xacro source** `urdf/yahboomcar_X3.urdf.xacro` (laser at line ~112) so the
flat URDF isn't reverted if regenerated.

Current laser joint (`yahboomcar_X3.urdf:189`):
```xml
<joint name="laser_joint" type="fixed">
    <origin xyz="0.0435 5.25826986680105E-05 0.11" rpy="0 0 3.14159265"/>
    <parent link="base_link"/>
    <child  link="laser_link"/>
</joint>
```

### Option A — minimal (just move the lidar up)
If the bracket geometry doesn't need to be visualized, only update the joint origin z (and
x/y if the scan origin shifts toward center):
```xml
<origin xyz="<new_x> <new_y> <new_z>" rpy="0 0 <new_yaw>"/>
```

### Option B — add the bracket as a link (recommended, accurate TF + visualization)
1. Add a `lidar_mount_link` with the bracket STL (visual + collision).
2. `lidar_mount_joint`: fixed, `parent base_link`, child `lidar_mount_link`, origin =
   bracket placement (§4.2).
3. Re-parent `laser_joint` to `lidar_mount_link`, origin = scan-origin offset (§4.3).

This keeps the model physically truthful and makes RViz show the real standoff.

---

## 6. Build & deploy

```bash
# On the robot (and locally), from workspace root:
source /opt/ros/humble/setup.bash
colcon build --packages-select yahboomcar_description
# redeploys urdf + meshes into install/.../share/yahboomcar_description/
sudo systemctl restart x3_server      # relaunches bringup -> robot_state_publisher reloads URDF
```
Mirror the URDF/mesh edits to the Jetson copy (diverged git) before building there.

---

## 7. Frame / convention verification (do not skip)

The current `laser_joint` has **`rpy = "0 0 π"`** (180° yaw). The CBF builds obstacle points
directly from `/scan` angles in the **laser frame** (`x = r·cos θ`, `y = r·sin θ`) and applies
them against a **base-frame** velocity command (`vx`, `vy`) assuming `θ = 0` is robot-forward.
If the raised lidar's mounting yaw changes, re-verify:

- Subscribe to `/scan` with the robot facing a known single obstacle dead ahead; confirm the
  minimum-range bearing is **θ ≈ 0** (forward), not π.
- If the new mount flips orientation, set the joint `rpy` yaw to match **and** re-check that
  `_scan_cb`'s `x/y` and the front mask still align with base-frame forward. Easiest robust
  option: transform scan points through TF (`laser_link → base_link`) instead of assuming
  alignment — worth doing during this update.

---

## 8. Optional software fallback — footprint subtraction

Only needed if, even raised, a little of the robot is still in view. Instead of a blanket
range gate, learn the body return per bearing and subtract it:

1. `scripts/calibrate_lidar_footprint.py` — robot in **open space** (nothing within ~1.5 m),
   record ~300 `/scan` frames, store the closest consistent return per beam angle →
   `config/lidar_body_profile.json`.
2. In `_scan_cb`: drop any beam with `r <= profile[angle] + margin` (body); keep the rest.
   Then `min_range≈0.15`, `safe_distance≈0.30`, full 360°.

Skip if §5 cleanly clears the body — raising the lidar is the real fix.

---

## 9. Validation checklist

- [ ] RViz: `laser_link` sits at the new height; `/scan` ring no longer paints the chassis.
- [ ] Re-run the scan analysis (see prior session) — **zero** self-returns < ~0.4 m in the
      front 180°.
- [ ] Roll back §2 filter params; restart `x3_server`.
- [ ] Drive forward at a wall → robot brakes and stops at the `safe_distance` standoff.
- [ ] Drive through a doorway → passes without freezing.
- [ ] Rotation, strafe, and diagonal motion all unrestricted in open space.
