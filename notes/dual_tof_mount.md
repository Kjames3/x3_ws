# Dual ToF mount — initial URDF placement, 2026-09-19

The user confirmed that the new bracket attaches in the old mount's location.
It is not installed yet. Upper adapter is 15 degrees downward; lower is 40.
Final chassis placement will be refined with `bash scripts/laptop_sim.sh view`.

Source assembly: `stls/VL53LCX_mount.STEP`. Its coordinates are not robot-relative.
The description meshes `tof_dual_bracket.stl`, `tof_adapter_15.stl`,
`tof_adapter_40.stl`, and `tof_breakout.stl` were tessellated with cascadio 0.01 mm
linear tolerance and trimesh, in their original part frames, in metres.
Do not apply the old millimetre STL scale to these files. Assembly node mapping:
NAUO1 bracket, NAUO2 upper board, NAUO3 upper adapter, NAUO4 lower adapter,
NAUO5 lower board. Each relative placement is inverse(bracket) * component.
The standalone SolidWorks STL exports have translated origins; they were not
used to infer assembly placement.

The sensing point retains the earlier board-local package-face coordinate
(0, 2.45, -1.97) mm. The optical axis points out the board's negative Z face.

Both `yahboomcar_X3.urdf` and `.urdf.xacro` contain the same ToF transforms.
The refined `tof_mount_joint` is xyz=(0.145, 0, -0.030549) m on base_link,
rpy=(pi/2, 0, pi/2). The initial placement reduced x relative to the old joint by 3 mm because
the bracket's rear plane changed from local z=-24.5 to -21.5 mm. That initially preserved
the rear mating plane; the lateral attachment origin remains. The user requested a 3 mm upward
refinement after the first RViz preview, confirmed that height, then requested
a further 16 mm forward translation.
This is a starting alignment, not a measurement of the newly installed mount.

| Frame | Robot X | Robot Y | Floor height | Downward pitch |
|---|---:|---:|---:|---:|
| tof_upper_link | 162.970 mm | -0.100 mm | 161.820 mm | 15 degrees |
| tof_lower_link | 164.463 mm | -0.100 mm | 65.843 mm | 40 degrees |

Each has an optical frame (+Z viewing direction, +X right, +Y down).
The legacy `tof_link`/`tof_optical_frame` names alias the UPPER sensor.
This preserves frame names, not the old sensor pose: the current single-sensor
driver's height/geometry assumptions and dual Teensy ingestion must be updated
before using the new arrangement for perception on the robot. No robot deploy
or firmware upload was performed.

Refine `tof_mount_joint` in both description files to move the whole assembly.
Keep the internal CAD placements together. The adapters are fixed joints;
changing to another printed angle also requires its mesh and board placement,
not just rotating the sensing frame.

Validation: both models passed check_urdf (Xacro expanded with ns:=x3), all mesh
references exist, sensor pitches and optical axes match, and both models give
equal ToF poses. The description package built successfully. See
`artifacts/tof-mount-2026-09-19/side_view.png` and `validated_poses.json`
for the initial placement before the 3 mm upward and 16 mm forward refinements.
Pre-existing Xacro macro calls were prefixed with xacro: so they expand on
Humble; its mecanum mesh references now select the existing _X3 filenames.

## Next: dual-sensor wiring and firmware

Proposed separate buses: upper SDA=18/SCL=19 (Wire), lower SDA=17/SCL=16
(Wire1). Both can retain 7-bit address 0x29. LPn remains high; INT is optional
for polling. Combined sensor power needs checking before final wiring.
This arrangement and a two-sensor benchmark have not yet been hardware-tested.
