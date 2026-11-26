// Apartment furniture from tape measurements (2026-11-26):
//   dining table 76 x 76 cm square top at 76 cm, four legs, two ladder-back chairs
//   coffee table 81 cm round top at 46 cm (2.5 cm panels)
// Synthetic Livox-like returns at ~3 cm spacing on the visible surfaces.
const assert = require('node:assert/strict');
const {furnitureSurfaces} = require('../../src/web/reality-structure');
const cells = [];
const dense = (x0, x1, y0, y1, z, keep = () => true) => {
    for (let x = x0; x <= x1; x += .03) for (let y = y0; y <= y1; y += .03) if (keep(x, y)) cells.push({x, y, z});
};
const leg = (x, y, h) => { for (let z = .1; z < h; z += .06) cells.push({x, y, z}); };
// Dining table centred (2.0, 0), top 0.76.
dense(1.62, 2.38, -.38, .38, .76);
for (const [x, y] of [[1.65,-.35],[1.65,.35],[2.35,-.35],[2.35,.35]]) leg(x, y, .74);
// Chair pushed in on the +y side: seat 0.45 at y .45-.85, ladder back at y .85 up to 0.95.
dense(1.8, 2.2, .45, .85, .45);
for (const [x, y] of [[1.82,.47],[2.18,.47],[1.82,.83],[2.18,.83]]) leg(x, y, .44);
for (let x = 1.8; x <= 2.2; x += .03) for (let z = .5; z <= .95; z += .05) cells.push({x, y: .86, z});
// Round coffee table centred (0.5, 1.5), diameter 0.81, top 0.46, central pedestal.
dense(.09, .91, 1.09, 1.91, .46, (x, y) => Math.hypot(x - .5, y - 1.5) <= .405);
leg(.5, 1.5, .44); leg(.3, 1.5, .44); leg(.7, 1.5, .44);

const out = furnitureSurfaces(cells);
const tables = out.filter(o => o.kind === 'table');
const dining = tables.find(o => !o.low), coffee = tables.find(o => o.low);
assert.ok(dining, 'dining table found on its own');
assert.ok(Math.abs(dining.L - .76) < .14 && Math.abs(dining.D - .76) < .14, `dining ${dining.L.toFixed(2)} x ${dining.D.toFixed(2)}`);
assert.ok(Math.abs(dining.H - .76) < .05, `dining H ${dining.H}`);
assert.ok(Math.hypot(dining.x - 2, dining.y) < .1, 'dining centre');
assert.ok(coffee, 'coffee table found (seat height, no backrest)');
assert.ok(Math.abs(coffee.L - .81) < .14 && Math.abs(coffee.D - .81) < .14, `coffee ${coffee.L.toFixed(2)} x ${coffee.D.toFixed(2)}`);
assert.ok(Math.abs(coffee.H - .46) < .05 && coffee.round, 'coffee table height and round');
assert.ok(!dining.round, 'square table not round');
const chair = out.find(o => o.kind === 'chair');
assert.ok(chair && Math.abs(chair.seat - .45) < .05, 'chair kept separate from the table');
console.log(`Apartment furniture: dining ${(dining.L*100).toFixed(0)}x${(dining.D*100).toFixed(0)}x${(dining.H*100).toFixed(0)} cm, coffee ${(coffee.L*100).toFixed(0)}x${(coffee.D*100).toFixed(0)}x${(coffee.H*100).toFixed(0)} cm round, chair seat ${(chair.seat*100).toFixed(0)} cm`);

// The round flag reaches the tracker output and survives the session round trip.
{
    const {Tracker, analyze} = require('../../src/web/reality-structure');
    const tr = new Tracker(), pose = {x:0, y:0, theta:0};
    let objs;
    for (let i = 0; i <= 40; i++) objs = analyze(cells, [], pose, tr, i * .1, {mapped:true, xyz:[i]}).objects;
    const roundOut = objs.find(o => o.kind === 'table' && o.round);
    assert.ok(roundOut && Math.abs(roundOut.H - .46) < .05, 'round coffee table is drawn round');
    assert.ok(objs.some(o => o.kind === 'table' && !o.round && Math.abs(o.H - .76) < .05), 'square dining table');
    const saved = JSON.parse(JSON.stringify(tr.exportLandmarks()));
    const fresh = new Tracker(); fresh.importLandmarks(saved, 0);
    assert.ok(fresh.tracks.some(t => t.round && Math.abs(t.H - .46) < .05), 'round survives export/import');
    console.log('Round coffee table drawn round and kept through the session memory');
}
