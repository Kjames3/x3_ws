// Yaw-v2 furniture rules from the 2026-11-28 apartment walk: chairs at a table,
// backs to walls, round-table latch, unmapped hold, no camera births while turning.
const assert = require('node:assert/strict');
const RS = require('../../src/web/reality-structure');
const deg = d => d * Math.PI / 180, near = (a, b, tol = 1e-6) => Math.abs(a - b) < tol;

// Chairs and a table against a wall (table long axis along x).
const table = {id:1, kind:'table', X:1, Y:0, yaw:0, L:.8, D:.6, H:.7, hits:9};
const end = {id:2, kind:'chair', X:1.5, Y:0, yaw:deg(30), L:.5, D:.4, H:.8, hits:9};   // overlaps the +x end
const inside = {id:3, kind:'chair', X:1.05, Y:.02, yaw:0, L:.5, D:.4, H:.8, hits:9};  // centre inside the top
const beside = {id:4, kind:'chair', X:.3, Y:0, yaw:deg(10), L:.5, D:.4, H:.8, hits:9};  // just clear of the -x end
const far = {id:5, kind:'chair', X:3, Y:2, yaw:deg(10), L:.5, D:.4, H:.8, hits:9};
const tracks = [table, end, inside, beside, far];
RS.seatAtTables(tracks);
assert.ok(inside.drop, 'a chair centred inside the top is table voxels');
assert.ok(near(end.X, 1 + .4 + .2 + .02) && near(end.Y, 0), 'moved out past the end it was over');
// Mesh back is +depth = (-sin yaw, cos yaw): it must point away from the table.
assert.ok(near(-Math.sin(end.yaw), 1) && near(Math.cos(end.yaw), 0, 1e-9), 'end chair faces the table');
assert.ok(near(beside.X, .3), 'a chair already clear is not moved');
assert.ok(near(-Math.sin(beside.yaw), -1), 'but it turns to face the table');
assert.ok(near(far.yaw, deg(10)), 'a chair elsewhere is untouched');

// Back to the wall, no 180 flip from a fit along the front edge.
const arm = {id:6, kind:'chair', X:0, Y:1, yaw:deg(100), L:.55, D:.55, H:1, hits:9};
const toOdom = (x, y) => [x, y];
RS.backToWalls([arm], [{x0:-2, y0:1.3, x1:2, y1:1.3}], toOdom);
assert.ok(near(Math.cos(arm.yaw), 1, 1e-9), 'back (+y) toward the wall at y=1.3');
RS.backToWalls([arm], [{x0:-2, y0:.75, x1:2, y1:.75}], toOdom);
assert.ok(near(Math.cos(arm.yaw), 1, 1e-9), 'a wall fit in front does not flip it');
console.log('Chairs at tables and backs to walls passed');

// Through the tracker: unmapped confirmed furniture survives 3 s out of view; and a camera-only
// chair is not born while the robot turns.
function patch(round) {   // 46 cm tabletop, round or square, 2 m ahead
    const pts = [];
    for (let x = -.4; x <= .4; x += .025) for (let y = -.4; y <= .4; y += .025)
        if (!round || Math.hypot(x, y) <= .4) pts.push(2 + x, y, .44);
    for (const [lx, ly] of [[-.3,-.3],[.3,-.3],[-.3,.3],[.3,.3]]) for (let z = .05; z < .44; z += .05) pts.push(2+lx, ly, z);
    return pts;
}
const SM = require('../../src/web/surroundings-memory');
const tracker = new RS.Tracker({furnitureYawV2:true});
let pose = {x:0, y:0, theta:0}, t = 0;
function step(xyz, dets = [], turn = 0) {
    t += .2; pose = {...pose, theta: pose.theta + turn};
    const cells = SM.voxelize(xyz, .05);
    return RS.analyze(cells, dets, pose, tracker, t, {mapped:false, xyz, detectionStamp:t,
        observationPose:pose, wallPose:{...pose, yaw:pose.theta, stamp:t}}).objects;
}
// Walk: the round table read round in 17% of views, the rectangular one in 1%.
const roundTop = {}, rectTop = {};
for (let i = 0; i < 100; i++) { RS.voteRound(roundTop, i % 6 === 0); RS.voteRound(rectTop, i === 50); }
assert.ok(roundTop.round && !rectTop.round, 'round share decides, not the latest view');
RS.voteRound(roundTop, false);
assert.ok(roundTop.round, 'one square-looking view does not flip it back');
for (let i = 0; i < 30; i++) step(patch(false));
const low = tracker.tracks.find(q => q.kind === 'table');
assert.ok(low, 'table tracked');
const id = low.id;
for (let i = 0; i < 25; i++) step([]);           // 5 s with nothing seen, nothing seen through it
assert.ok(tracker.tracks.some(q => q.id === id), 'unmapped confirmed table held out of view');
const chairDet = [{label:'chair', conf:.9, xyz_base_m:{x:1, y:1.5, z:.5}}];
const before = tracker.tracks.length;
step([], chairDet, deg(10));                     // 50 deg/s
step([], chairDet, deg(10));
assert.equal(tracker.tracks.length, before, 'no camera-only birth while turning');
for (let i = 0; i < 5; i++) step([], chairDet);
assert.ok(tracker.tracks.some(q => q.kind === 'chair'), 'born once the robot is still');
console.log('Round latch, unmapped hold and turning gate passed');
