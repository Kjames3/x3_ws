// Curved intent (?curvedIntent=1): server offsets -> scene, and the spline the
// Reality view draws through them.   node tests/web/reality_curved_intent_check.cjs
const assert = require('node:assert/strict');
const fs = require('fs');
const vm = require('vm');
global.window = {};
vm.runInThisContext(fs.readFileSync(__dirname + '/../../src/web/reality-mode.js', 'utf8'));
const { splineResample, relToScene } = window.RealityMode;

// Scene: forward = -Z, right = +X. An avatar 1 m right and 2 m ahead (x=1, z=-2);
// the server says 1 m forward then 1 m left of the person.
const pts = relToScene(1, -2, [[0, 0], [1, 0], [1, 1]]);
assert.deepEqual(pts[0], [1, -2], 'offsets are anchored at the avatar');
assert.deepEqual(pts[1], [1, -3], 'forward is -Z');
assert.deepEqual(pts[2], [0, -3], 'left is -X');

// A quarter circle (left turn) sampled at 16 points: the spline passes through
// every control point, stays on the arc between them, and keeps its end.
const n = 16, R = 1.5;
const arc = Array.from({ length: n }, (_, i) => {
    const a = (Math.PI / 2) * i / (n - 1);
    return [-(R - R * Math.cos(a)), -R * Math.sin(a)];       // start heading -Z, turning to -X
});
const widths = arc.map((_, i) => 0.2 + 0.3 * i / (n - 1));
const [out, ow] = splineResample(arc, widths, 3);
assert.equal(out.length, (n - 1) * 3 + 1);
assert.equal(ow.length, out.length);
for (let i = 0; i < n; i++) {
    assert.ok(Math.hypot(out[3 * i][0] - arc[i][0], out[3 * i][1] - arc[i][1]) < 1e-9, 'passes through control point ' + i);
}
let worst = 0;
for (const [x, z] of out) worst = Math.max(worst, Math.abs(Math.hypot(x + R, z) - R));
assert.ok(worst < 0.005, 'stays within 5 mm of the arc, got ' + worst);
assert.ok(Math.abs(ow[ow.length - 1] - 0.5) < 1e-9 && Math.abs(ow[1] - (0.2 + 0.1 / 15)) < 1e-9, 'widths interpolate');

// A straight path resamples to a straight line (no overshoot).
const line = Array.from({ length: n }, (_, i) => [0, -0.13 * i]);
const [lo] = splineResample(line, line.map(() => 0.3), 3);
assert.ok(lo.every(([x]) => Math.abs(x) < 1e-12), 'straight stays straight');

// A predicted stop (points bunched at the end, one stepping back): finite, no blow-up.
const stop = Array.from({ length: n }, (_, i) => [0, -Math.min(i, 8) * 0.1 + (i === 12 ? 0.01 : 0)]);
const [so] = splineResample(stop, stop.map(() => 0.3), 3);
assert.ok(so.every(([x, z]) => Number.isFinite(x) && Number.isFinite(z) && z <= 0.02 && z >= -0.85), 'stop stays bounded');
console.log('Curved intent offsets, spline fidelity, straight lines and stops passed');
