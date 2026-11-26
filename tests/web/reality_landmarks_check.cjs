// Session landmarks: export -> server copy -> import into a fresh tracker
// (page refresh). Restored furniture stays while out of view on a mapped
// frame and is removed only when the lidar sees through where it stood.
const assert = require('node:assert/strict');
const {Tracker, analyze} = require('../../src/web/reality-structure');

const pose = {x: 0, y: 0, theta: 0};
const saved = [{kind: 'chair', X: 2, Y: 0, yaw: 0, L: .5, D: .5, H: .9, seat: .46},
               {kind: 'table', X: 1, Y: 2, yaw: .3, L: 1.2, D: .7, H: .74}];

// Round trip through JSON, like the server message.
const a = new Tracker();
assert.equal(a.importLandmarks(JSON.parse(JSON.stringify(saved)), 0), 2);
assert.equal(a.importLandmarks(saved, 0), 0, 'duplicates are merged');
const out = a.exportLandmarks();
assert.deepEqual(out.map(o => o.kind).sort(), ['chair', 'table']);
assert.equal(out.find(o => o.kind === 'chair').seat, .46);
assert.equal(a.importLandmarks([{kind: 'spaceship', X: 0, Y: 0, yaw: 0, L: 1, D: 1, H: 1}], 0), 0);

// Out of view for a minute with an empty cloud: still there on a mapped frame.
let objs;
for (let t = 1; t <= 60; t += 1) objs = analyze([], [], pose, a, t, {mapped: true, xyz: new Float32Array(0)}).objects;
assert.equal(objs.length, 2, 'restored landmarks survive while unseen');

// Unmapped frames keep the old 3 s hold.
const b = new Tracker(); b.importLandmarks(saved, 0);
assert.equal(analyze([], [], pose, b, 10, {}).objects.length, 0);

// Lidar returns beyond the chair, along rays through its seat: vacated.
function through(x, y, level, n) {
    const sx = .0432, sz = .31, pts = [];
    for (let i = 0; i < n; i++) {
        const yy = y + (i - n / 2) * .01, f = (x - sx) / (3.5 - sx);
        pts.push(3.5, yy / f, sz + (level - sz) / f);
    }
    return new Float32Array(pts);
}
for (let i = 0; i < 4; i++) objs = analyze([], [], pose, a, 100 + i, {mapped: true, xyz: through(2, 0, .48 * .9, 8)}).objects;
assert.deepEqual(objs.map(o => o.kind), ['table'], 'the moved chair is cleared');
console.log('Landmark round trip, out-of-view hold, unmapped expiry and vacated removal passed');
