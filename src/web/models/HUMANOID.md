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
