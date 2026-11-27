// Furniture look V2 (src/web/reality-furniture.js): metric layouts stay in the
// measured box, put backs on the convention side, keep discrete choices
// stable, and the renderer reuses/disposes what it creates.
const assert = require('node:assert/strict');
const F = require('../../src/web/reality-furniture');
const THREE = require('../../src/web/vendor/three/three.min.js');

const EPS = 0.011;
function extent(p) {
    // Half extents in the local frame, including a rotation about x.
    const c = Math.abs(Math.cos(p.rx || 0)), s = Math.abs(Math.sin(p.rx || 0));
    return {hx: p.sx / 2, hy: (c * p.sy + s * p.sz) / 2, hz: (s * p.sy + c * p.sz) / 2};
}
function inside(parts, b, label, slack = 0) {
    for (const p of parts) {
        const {hx, hy, hz} = extent(p);
        assert.ok(Math.abs(p.x) + hx <= b.L / 2 + EPS + slack, `${label}: x out of box ${JSON.stringify(p)}`);
        assert.ok(Math.abs(p.z) + hz <= b.D / 2 + EPS + slack, `${label}: z out of box ${JSON.stringify(p)}`);
        assert.ok(p.y - hy >= -EPS && p.y + hy <= b.H + EPS, `${label}: y out of box ${JSON.stringify(p)}`);
    }
}
const bottomOf = p => p.y - extent(p).hy, topOf = p => p.y + extent(p).hy;
const onFloor = parts => parts.filter(p => Math.abs(bottomOf(p)) < 1e-6);

// ---- every kind stays inside its measured box, across the size ranges ----
const sizes = {
    chair: [[.40, .40, .70, .38], [.45, .48, .90, .46], [.70, .70, 1.10, .45], [.62, .60, .80, undefined]],
    couch: [[1.2, .70, .60, .40], [2.1, .92, .86, .44], [2.6, 1.1, 1.0, undefined], [1.5, .86, .82, .50]],
    table: [[.6, .5, .4], [1.4, .8, .74], [2.2, 1.2, .9], [1.0, .55, .45], [.81, .8, .46], [1.2, 1.15, .75]],
};
for (const [kind, list] of Object.entries(sizes)) for (const [L, D, H, seat] of list) {
    // reality-structure.js only marks a top round when D > .85 L; the disc
    // uses the mean diameter, so it can overhang D by (L - D) / 4 otherwise.
    const tables = D > .85 * L ? [{}, {round: true}, {low: true}] : [{}, {low: true}];
    for (const extra of kind === 'table' ? tables : [{}]) {
        const b = {L, D, H, seat, ...extra};
        const parts = F.layout(kind, b, {});
        const label = `${kind} ${JSON.stringify(b)}`;
        inside(parts, b, label, b.round ? (L - D) / 4 : 0);
        assert.ok(parts.length >= 5, `${label}: too few parts`);
        assert.ok(onFloor(parts).length >= 1, `${label}: nothing stands on the floor`);
        assert.ok(Math.abs(Math.max(...parts.map(topOf)) - H) < .015, `${label}: top is not at H`);
        assert.ok(F.triangles(parts) <= 400, `${label}: ${F.triangles(parts)} triangles`);
    }
}
console.log('V2 layouts stay inside the box, stand on the floor, reach H, <= 400 triangles');

// ---- tables: thin top, four legs at the corners, round = disc + pedestal ----
{
    const b = {L: 1.4, D: .8, H: .74};
    const parts = F.layout('table', b, {});
    const top = parts.find(p => Math.abs(topOf(p) - b.H) < 1e-6);
    assert.ok(top.sy <= .04 && Math.abs(top.sx - b.L) < 1e-6 && Math.abs(top.sz - b.D) < 1e-6, 'thin full-size top');
    const legs = onFloor(parts);
    assert.equal(legs.length, 4);
    assert.deepEqual(new Set(legs.map(p => Math.sign(p.x) + ',' + Math.sign(p.z))).size, 4, 'one leg per corner');
    for (const leg of legs) assert.ok(leg.sx < .06 && Math.abs(topOf(leg) - (b.H - top.sy)) < 1e-6, 'legs meet the top');
    const round = F.layout('table', {L: .9, D: .86, H: .72, round: true}, {});
    assert.ok(round.every(p => p.shape !== 'box' && p.x === 0 && p.z === 0), 'round table is a centred pedestal');
    assert.ok(Math.abs(round[0].sx - .88) < 1e-6 && round[0].sy <= .04, 'disc diameter = mean of L and D');
}

// ---- chairs: seat at the measured height, back on -Z, arms with hysteresis ----
{
    const b = {L: .45, D: .48, H: .9, seat: .46};
    const parts = F.layout('chair', b, {});
    const seat = parts.find(p => p.sx === b.L);
    assert.ok(Math.abs(topOf(seat) - .46) < 1e-6, 'seat top = measured seat height');
    const above = parts.filter(p => bottomOf(p) >= .46 - 1e-6);
    assert.ok(above.length >= 3 && above.every(p => p.z < -b.D / 4), 'everything above the seat is at the back (-Z)');
    assert.equal(onFloor(parts).length, 4, 'four legs');
    // Legacy seat rule: seat/H clamped to [.30, .72]; missing seat -> .46 H.
    assert.ok(Math.abs(F.chairSeat({seat: .2}, 1) - .3) < 1e-9 && Math.abs(F.chairSeat({}, 1) - .46) < 1e-9);
    const st = {};
    const n = L => F.layout('chair', {L, D: .6, H: .9, seat: .45}, st).length;
    const plain = n(.5), armed = n(.65);
    assert.ok(armed > plain && st.arms, 'wide chair gets arms');
    assert.equal(n(.58), armed, 'arms stay on inside the hysteresis band');
    assert.equal(n(.55), plain, 'arms drop below it');
    assert.equal(n(.60), plain, 'and stay off until 0.61');
}

// ---- couches: cushions split by length (hysteresis), back cushions on -Z ----
{
    const count = (L, st = {}) => (F.layout('couch', {L, D: .9, H: .85, seat: .44}, st), st.cushions);
    assert.equal(count(1.2), 1); assert.equal(count(1.6), 2); assert.equal(count(2.3), 3);
    const st = {};
    count(1.6, st);                       // inner width ~1.33 -> 2
    assert.equal(count(2.0, st), 2, 'inner width 1.66 sits in the band: keep 2');
    assert.equal(count(2.2, st), 3, 'clearly past it: 3');
    assert.equal(count(2.0, st), 3, 'and back inside the band keeps 3');
    const b = {L: 2.1, D: .92, H: .86, seat: .44};
    const parts = F.layout('couch', b, {});
    const backs = parts.filter(p => p.rx);
    assert.equal(backs.length, 3, 'one back cushion per seat cushion');
    assert.ok(backs.every(p => p.z < 0), 'back cushions on -Z');
    assert.equal(onFloor(parts).length, 4, 'four feet');
    assert.ok(onFloor(parts).every(p => topOf(p) <= .08), 'short feet');
    const seats = parts.filter(p => !p.rx && Math.abs(topOf(p) - .44) < 1e-6);
    assert.equal(seats.length, 3, 'seat cushions top out at the seat height');
}

// ---- other kinds reuse the legacy unit parts, scaled to metres ----
{
    const legacy = {tv: [[0, .62, 0, 1, .72, .12]], box: [[0, .5, 0, 1, 1, 1]]};
    const [p] = F.layout('tv', {L: 1.1, D: .1, H: .65}, {}, legacy);
    assert.ok(Math.abs(p.sx - 1.1) < 1e-9 && Math.abs(p.y - .403) < 1e-9 && Math.abs(p.sz - .012) < 1e-9);
    assert.equal(F.layout('plant', {L: .5, D: .5, H: 1}, {}, legacy).length, 1, 'unknown kind -> legacy box');
}

// ---- facing convention: local -Z (the back) is base yaw + 90 deg ----
{
    // reality-mode.js: group rotation.y = yaw + pi/2; scene x = -base y, z = -base x.
    for (const yaw of [0, .7, 2, -2.5]) {
        const th = yaw + Math.PI / 2, lx = 0, lz = -1;
        const sx = lx * Math.cos(th) + lz * Math.sin(th), sz = -lx * Math.sin(th) + lz * Math.cos(th);
        const bx = -sz, by = -sx;
        assert.ok(Math.abs(bx - Math.cos(yaw + Math.PI / 2)) < 1e-9 && Math.abs(by - Math.sin(yaw + Math.PI / 2)) < 1e-9);
    }
    // And the real fitter puts a chair's backrest there: raised returns on the
    // -Y edge of a seat (fixture from reality_furniture_check.cjs).
    const {furnitureSurfaces} = require('../../src/web/reality-structure');
    const cells = [];
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
        const x = 3.6 + i * .12 + .01, y = j * .12 + .01;
        cells.push({x, y, z: .45});
        if ((i === 0 || i === 3) && (j === 0 || j === 3)) cells.push({x, y, z: .2});
        if (j === 0) cells.push({x, y, z: .62}, {x, y, z: .82});
    }
    const chair = furnitureSurfaces(cells).find(o => o.kind === 'chair');
    const back = [Math.cos(chair.yaw + Math.PI / 2), Math.sin(chair.yaw + Math.PI / 2)];
    assert.ok(back[1] < -.95, `fitted chair back should point to -Y, got ${back}`);
}
console.log('Tables, chairs, couches, legacy fallback and the back-side convention passed');

// ---- flag: URL wins and is remembered; default off ----
{
    const store = {};
    globalThis.localStorage = {getItem: k => (k in store ? store[k] : null), setItem: (k, v) => { store[k] = String(v); }};
    globalThis.location = {search: ''};
    assert.equal(F.flagEnabled(), false, 'default off');
    globalThis.location = {search: '?furnitureLookV2=1'};
    assert.equal(F.flagEnabled(), true);
    globalThis.location = {search: ''};
    assert.equal(F.flagEnabled(), true, 'remembered');
    globalThis.location = {search: '?furnitureLookV2=0'};
    assert.equal(F.flagEnabled(), false);
    delete globalThis.localStorage; delete globalThis.location;
}

// ---- renderer: shared geometry, reuse on refit, fade, dispose on removal ----
{
    const scene = new THREE.Scene();
    const R = F.createRenderer(THREE);
    const ob = R.make('couch', scene, 0);
    let disposed = 0;
    ob.onDispose = () => { disposed++; };
    const mats = Object.values(ob.mats);
    let matDisposed = 0;
    for (const m of mats) m.addEventListener('dispose', () => matDisposed++);
    R.fit(ob, {L: 2.1, D: .9, H: .85, seat: .44});
    const meshes = ob.meshes.length, geos = new Set(ob.meshes.map(m => m.geometry));
    assert.ok(geos.size <= 3, 'parts share unit geometries');
    R.fit(ob, {L: 2.12, D: .91, H: .86, seat: .44});
    assert.equal(ob.meshes.length, meshes, 'refit reuses meshes');
    R.fit(ob, {L: 1.3, D: .9, H: .85, seat: .44});
    assert.ok(ob.meshes.filter(m => m.visible).length < meshes, 'fewer cushions hide spare meshes');
    assert.ok(ob.mats.box.transparent && ob.fade === 0, 'starts transparent');
    R.tick(scene, F.FADE_IN_MS / 2);
    assert.ok(ob.fade > .2 && ob.fade < .8 && ob.body.position.y < 0, 'fading in and rising');
    R.tick(scene, F.FADE_IN_MS + 1);
    assert.ok(ob.fade === 1 && !ob.mats.box.transparent && ob.body.position.y === 0, 'solid when done');
    assert.equal(R.fadingCount(), 0);
    R.retire(ob, scene, 1000);
    R.tick(scene, 1000 + F.FADE_OUT_MS / 2);
    assert.ok(scene.children.includes(ob.group) && ob.fade < 1, 'fading out');
    R.tick(scene, 1000 + F.FADE_OUT_MS + 1);
    assert.ok(!scene.children.includes(ob.group), 'removed after the fade');
    assert.equal(disposed, 1); assert.equal(matDisposed, mats.length, 'per-object materials disposed');
    const gone = R.make('chair', scene, 0);
    R.retire(gone, scene, 5, true);
    assert.ok(!scene.children.includes(gone.group), 'instant removal');
}
console.log('Flag and renderer lifecycle passed');
