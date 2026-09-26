// NODE_PATH=<directory containing three@0.128.0> node tests/test_humanoid_walk.cjs
const assert = require('assert'), fs = require('fs'), path = require('path'), vm = require('vm');
global.THREE = require('three'); global.window = {};
const root = path.resolve(__dirname, '..');
for (const f of ['models/humanoid-walk.js', 'humanoid.js']) vm.runInThisContext(fs.readFileSync(path.join(root, 'src/web', f), 'utf8'));
const clip = window.X3WalkCycle;
const a = window.X3Humanoid.create(), ghost = window.X3Humanoid.create({ ghost: true });
const initial = Object.fromEntries(Object.entries(a.joints).map(([n, j]) => [n, j.quaternion.clone()]));
a.group.position.set(2, 0, -3);
let minFoot = Infinity, maxFoot = -Infinity, motion = 0;
for (let i = 0; i < 600; i++) {
    a.animate(clip.referenceSpeed, 1 / 60); ghost.animate(clip.referenceSpeed, 1 / 60);
    a.group.updateMatrixWorld(true);
    const feet = ['left_foot', 'right_foot'].map(n => new THREE.Box3().setFromObject(a.joints[n]).min.y);
    if (i > 100) { minFoot = Math.min(minFoot, ...feet); maxFoot = Math.max(maxFoot, Math.min(...feet)); }
    for (const n of clip.joints) {
        assert(a.joints[n].quaternion.toArray().every(Number.isFinite));
        assert(a.joints[n].quaternion.angleTo(ghost.joints[n].quaternion) < 1e-6);
    }
    motion = Math.max(motion, a.joints.left_shin.quaternion.angleTo(initial.left_shin));
}
assert(motion > 0.3, 'leg must visibly bend');
assert(minFoot > -0.006 && maxFoot < 0.006, `planted foot drift: ${minFoot}, ${maxFoot}`);
assert.deepStrictEqual(a.group.position.toArray(), [2, 0, -3], 'clip must not change track position');
for (let i = 0; i < 180; i++) a.animate(0, 1 / 60);
for (const n of clip.joints) assert(a.joints[n].quaternion.angleTo(initial[n]) < 1e-6, 'stop must restore rest pose');
for (let j = 0; j < clip.joints.length; j++) {
    const first = new THREE.Quaternion().fromArray(clip.samples[0], j * 4).normalize();
    const last = new THREE.Quaternion().fromArray(clip.samples[clip.samples.length - 1], j * 4).normalize();
    assert(first.angleTo(last) < 1e-5, 'loop seam must close');
}
const transition = window.X3Humanoid.create();
for (let i = 0; i < 240; i++) {
    transition.animate(i < 120 ? 1 : 0, 1 / 60);
    transition.group.updateMatrixWorld(true);
    const support = Math.min(...['left_foot', 'right_foot'].map(n => new THREE.Box3().setFromObject(transition.joints[n]).min.y));
    assert(Math.abs(support) < 0.006, 'start/stop blend must stay grounded');
}
const slow = window.X3Humanoid.create(), fast = window.X3Humanoid.create();
for (let i = 0; i < 70; i++) { slow.animate(0.4, 1 / 60); fast.animate(1, 1 / 60); }
assert(slow.joints.left_thigh.quaternion.angleTo(fast.joints.left_thigh.quaternion) > 0.05, 'speed must change playback phase');
a.animate(NaN, Infinity); assert(a.joints.left_shin.quaternion.toArray().every(Number.isFinite));
console.log('PASS: limb motion, grounded feet, fixed track root, synchronized ghost, idle blend and closed loop.', { minFoot, maxFoot, motion });
