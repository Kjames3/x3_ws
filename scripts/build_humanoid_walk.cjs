#!/usr/bin/env node
// Offline baker. Requires three@0.128.0 (e.g. installed in a temporary directory
// and exposed through NODE_PATH). Usage: node scripts/build_humanoid_walk.cjs input.fbx
// Input: user-supplied WALK-RUN-CYCLES-MOCAP/10-WalkCycle_01_MIXAMO_769.fbx.
// Output contains only in-place joint rotations; no FBX loader runs in the GUI.
const fs = require('fs'), path = require('path'), vm = require('vm');
global.THREE = require('three');
global.fflate = require('three/examples/js/libs/fflate.min.js');
global.window = { URL: global.URL };
vm.runInThisContext(fs.readFileSync(require.resolve('three/examples/js/loaders/FBXLoader.js'), 'utf8'));
const root = path.resolve(__dirname, '..');
vm.runInThisContext(fs.readFileSync(path.join(root, 'src/web/humanoid.js'), 'utf8'));
const bytes = fs.readFileSync(process.argv[2]);
const source = new THREE.FBXLoader().parse(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '');
const avatar = window.X3Humanoid.create();
const { group, joints } = avatar, frame = group.children[0];
const baseHeight = frame.position.y;
source.updateMatrixWorld(true); group.updateMatrixWorld(true);
const bone = n => source.getObjectByName('mixamorig' + n);
const pos = n => bone(n).getWorldPosition(new THREE.Vector3());
const quat = n => bone(n).getWorldQuaternion(new THREE.Quaternion());
const bind = {}, targetBind = {};
for (const [name, src] of [['left_foot', 'LeftFoot'], ['right_foot', 'RightFoot'], ['head', 'Head']]) {
    bind[name] = quat(src).invert();
    targetBind[name] = joints[name].getWorldQuaternion(new THREE.Quaternion());
}
const rest = Object.fromEntries(Object.entries(joints).map(([n, j]) => [n, j.quaternion.clone()]));
const mixer = new THREE.AnimationMixer(source);
mixer.clipAction(source.animations[0]).play();
const start = 224 / 30, end = 277 / 30, duration = end - start, count = 53;
const axisY = new THREE.Vector3(0, 1, 0);
const flip = new THREE.Quaternion().setFromAxisAngle(axisY, Math.PI);
const names = Object.keys(joints);
const poses = [], positions = [];
function aim(name, localDirection, desiredWorld) {
    const parent = joints[name].parent.getWorldQuaternion(new THREE.Quaternion()).invert();
    joints[name].quaternion.setFromUnitVectors(localDirection.clone().normalize(), desiredWorld.clone().applyQuaternion(parent).normalize());
    group.updateMatrixWorld(true);
}
for (let i = 0; i <= count; i++) {
    mixer.setTime(start + duration * i / count); source.updateMatrixWorld(true);
    positions.push(pos('Hips'));
    for (const n of names) joints[n].quaternion.copy(rest[n]);
    frame.position.y = baseHeight; group.updateMatrixWorld(true);
    const across = pos('LeftUpLeg').sub(pos('RightUpLeg'));
    const heading = Math.atan2(-across.z, across.x);
    const convert = flip.clone().multiply(new THREE.Quaternion().setFromAxisAngle(axisY, -heading));
    const direction = (a, b) => pos(b).sub(pos(a)).normalize().applyQuaternion(convert);
    aim('torso', new THREE.Vector3(0, 0, 1), direction('Hips', 'Neck'));
    for (const [side, src] of [['left', 'Left'], ['right', 'Right']]) {
        aim(side + '_thigh', joints[side + '_shin'].position, direction(src + 'UpLeg', src + 'Leg'));
        aim(side + '_shin', joints[side + '_foot'].position, direction(src + 'Leg', src + 'Foot'));
        aim(side + '_upper_arm', joints[side + '_lower_arm'].position, direction(src + 'Arm', src + 'ForeArm'));
        aim(side + '_lower_arm', joints[side + '_hand'].position, direction(src + 'ForeArm', src + 'Hand'));
    }
    for (const [name, src] of [['left_foot', 'LeftFoot'], ['right_foot', 'RightFoot'], ['head', 'Head']]) {
        const desired = convert.clone().multiply(quat(src)).multiply(bind[name]).multiply(flip.clone().invert()).multiply(targetBind[name]);
        const parentInverse = joints[name].parent.getWorldQuaternion(new THREE.Quaternion()).invert();
        joints[name].quaternion.copy(parentInverse.multiply(desired).normalize());
        group.updateMatrixWorld(true);
    }
    poses.push(names.map(n => joints[n].quaternion.clone()));
}
// Spread the small endpoint discrepancy smoothly over the loop instead of
// snapping or pausing at the seam. Quaternion interpolation uses the short arc.
const corrections = names.map((_, j) => poses[count][j].clone().invert().multiply(poses[0][j]));
const samples = [], floor = [];
for (let i = 0; i <= count; i++) {
    const t = i / count, weight = t * t * (3 - 2 * t);
    const values = [];
    names.forEach((name, j) => {
        const q = poses[i][j].clone().multiply(new THREE.Quaternion().slerp(corrections[j], weight)).normalize();
        joints[name].quaternion.copy(q);
        values.push(...q.toArray().map(v => +v.toFixed(6)));
    });
    frame.position.y = baseHeight; group.updateMatrixWorld(true);
    const left = new THREE.Box3().setFromObject(joints.left_foot);
    const right = new THREE.Box3().setFromObject(joints.right_foot);
    floor.push(+(-Math.min(left.min.y, right.min.y)).toFixed(6));
    samples.push(values);
}
// Source is centimeters. Account for target/source leg-length ratio.
mixer.stopAllAction();
const sourceLeg = (bone('LeftLeg').position.length() + bone('LeftFoot').position.length()) / 100;
const targetLeg = (joints.left_shin.position.length() + joints.left_foot.position.length()) * frame.scale.x;
const displacement = positions[count].clone().sub(positions[0]); displacement.y = 0;
const referenceSpeed = displacement.length() / 100 / duration * targetLeg / sourceLeg;
const data = { source: path.basename(process.argv[2]), sourceStart: start, sourceEnd: end, duration, referenceSpeed: +referenceSpeed.toFixed(4), joints: names, samples, floor };
const output = path.join(root, 'src/web/models/humanoid-walk.js');
fs.writeFileSync(output, '// Baked from user-supplied mocap; see HUMANOID.md. Regenerate with scripts/build_humanoid_walk.cjs.\nwindow.X3WalkCycle = ' + JSON.stringify(data) + ';\n');
console.log(JSON.stringify({ output, duration, referenceSpeed, frames: samples.length, bytes: fs.statSync(output).size }));
