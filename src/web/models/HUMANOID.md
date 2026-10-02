# Surroundings humanoid

`../humanoid.js` adapts the visual body hierarchy from
https://github.com/isaac-sim/IsaacGymEnvs/blob/main/assets/mjcf/nv_humanoid.xml
(derived from DeepMind dm_control). The upstream asset license is retained in
`licenses/humanoid-LICENSE.txt`. Physics, actuators, sensors and floor are omitted;
primitives use shared low-resolution geometry, arms are relaxed and the figure
is normalized to a generic 1.7 m height. Height and pose are illustrative.

`X3Humanoid.create({ ghost: false })` returns `{ group, joints, animate }`.
`group` uses viewer coordinates (X right, Y up, -Z forward), with feet at Y=0.
`joints` maps the original MJCF body names to Three.js groups, including torso,
pelvis, left/right upper_arm, lower_arm, hand, thigh, shin and foot. Their local
frames remain MJCF frames (X forward, Y left, Z up). Each group's
`userData.restQuaternion` records its resting rotation as [x, y, z, w].

Future walking clips must be retargeted to these named groups and local axes;
an arbitrary FBX/glTF skeleton will not map automatically. Apply in-place
animation inside the avatar, leaving the outer group's position owned by the
person track. Apply the same pose to predictionJoints for the faded prediction.
Travel direction is used as illustrative heading; it is not observed body yaw.

Meshes share geometry and materials across live/predicted figures. Do not
mutate/dispose these resources per track. Removing a track removes its groups;
only its ArrowHelper materials are independently owned and disposed.

## Walking clip

`humanoid-walk.js` is baked from the user-supplied Downloads archive
`WALK-RUN-CYCLES-MOCAP.zip`, file `10-WalkCycle_01_MIXAMO_769.fbx`.
The archive contains no license/readme; this records provenance without assigning
an inferred license to the motion data.

The full 13.43 s clip includes stops and turns. Frames 224–277 at 30 Hz provide
one approximately 1.77 s stride with closely matching local limb directions.
The baker removes root translation and pelvis heading, maps limb directions to
the target body frames, transfers foot/head orientation relative to bind pose,
closes the quaternion seam smoothly and bakes a floor-height correction.
Target proportions remain generic; the animation is illustrative motion.

Call `avatar.animate(speedMps, deltaSeconds)` each visible frame. Playback rate
uses the source's measured stride travel, scaled by the target/source leg length
ratio (reference speed approximately 0.577 m/s), clamped to 0.25–2.5x.
Motion above 0.15 m/s blends into walking; below it the figure blends to rest.
The outer group remains exclusively controlled by the person track.
The ghost uses the same pose/phase at its predicted location, not a future pose.

To rebuild, install `three@0.128.0` in a temporary directory, extract the original
FBX there, and use Node with that directory's node_modules on NODE_PATH:

```
NODE_PATH=/tmp/x3-gui-check/node_modules node scripts/build_humanoid_walk.cjs /path/to/10-WalkCycle_01_MIXAMO_769.fbx
NODE_PATH=/tmp/x3-gui-check/node_modules node tests/test_humanoid_walk.cjs
```

Three.js/FBXLoader and decompression are offline build dependencies only.
The deployed browser loads the compact baked JS asset; the Jetson sends the same
tracking messages and runs no additional inference.

## Observed body pose

`avatar.pose(directions, dt)` follows `animate` and accepts bone-name entries
`{ direction: THREE.Vector3, confidence: number }` in viewer world coordinates.
The GUI uses both endpoint confidences (minimum 0.5) for arms, thighs and shins,
and shoulder/hip midpoints for torso lean. Confidence controls blend strength;
a missing limb holds for 0.2 s and then fades to the illustrative animation.
Detection messages older than 0.5 s are rejected by the scene view.
The surroundings hint identifies incomplete leg observations. The prediction
copy uses the same observed pose, not a prediction of future joint movement.

After fitting, support-foot grounding lowers the body for visible seated bends
and permits one leg to lift while the other supports it. Track position stays
unchanged. This assumes a level floor and a supported person; jumping, stairs,
and feet suspended above the floor are not reconstructed. Generic body size
remains 1.7 m. Joint depth is still flattened to the torso plane, so frontal
sitting, forward kicks, hidden limbs and exact body yaw remain ambiguous.
All new fitting and smoothing runs in the viewing browser; no inference or
camera-stream changes are required on the robot.

Check with `NODE_PATH=<three module directory> node tests/test_humanoid_pose.cjs`
and the existing walking test. Live sitting/one-leg/walking checks remain needed.

Heading stability: body heading is a smoothed travel estimate, not measured chest
orientation. Updating it requires speed >= 0.25 m/s for >= 0.35 s and net travel
>= 0.18 m. At rest it retains its last target heading. Same-ID dropouts retain
state for 0.75 s while hidden. This suppresses posture-induced velocity spikes;
it cannot recover a stationary turn or resolve front/back ambiguity.

## Leg constraints and stationary foot contacts

`pose-refinement.js` adds browser-only display priors. Both leg observations must
be present for coordinated fitting. Hip elevation is limited to 135 degrees from
down; knee flexion to 150 degrees. A broad 80-degree hip-twist allowance preserves
sideways leg lifts, while the thigh and shin share a bend plane. Constraints fade
with observation confidence. These are generic visualization limits, not measured
joint angles, and the flattened depth still makes some poses ambiguous.

Confident feet near the floor can acquire a contact after 0.2 s of stability.
Fixed-length two-bone fitting corrects small ankle drift (at most 7.5 cm); an
unreachable target releases rather than stretching the leg. Contacts release on
foot lift/motion, lost confidence, track jumps, appreciable heading changes, or
person speed >= 0.2 m/s. They are disabled without robot pose telemetry, during
robot movement, on predicted-only C3 tracks, and on the prediction ghost.
This first pass stabilizes standing/seated support, not walking stance phases.
Tracking position remains untouched. The ghost shares pose observations but does
not inherit physical foot contacts at its predicted location.

The **Estimate body facing (experimental)** checkbox is off by default. When
selected, shoulder and hip left/right ordering must agree, with confidence >= .75
and sufficiently wide projection relative to torso length. A front/back cue must
persist for 0.6 s before overriding travel heading. Side-on views supply no cue;
ambiguous stationary observations retain the previous heading. This estimates
broad front/back orientation, not continuous yaw or independent chest/pelvis twist.
Uncheck to return to the established travel-heading behavior.

Validation: `tests/test_pose_refinement.cjs` covers geometric limits, side lifts,
sitting, fixed-length contact IK, unreachable contacts, release conditions,
synthetic ankle-jitter suppression, and facing-cue ambiguity. Existing humanoid
pose/walk checks cover dropout, heading persistence and moving poses. Browser
comparison shows standing, knee lift, sitting and crouch alongside the previous
version. Live validation of this refinement pass is still pending.
