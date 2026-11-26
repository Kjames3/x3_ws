const assert = require('node:assert/strict');
const {WallHistory} = require('../../src/web/reality-structure');
const h = new WallHistory();
const angle = .24, cos = Math.cos(angle), sin = Math.sin(angle);
function scan(i) {
    const xyz = [], pose = {x:.02*i, y:.1, yaw:.025*i};
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    function point(x,y,z) {
        const dx=x-pose.x, dy=y-pose.y;
        xyz.push(c*dx+s*dy,-s*dx+c*dy,z);
    }
    for (let j=-60;j<=60;j++) {
        const along=j*.055;
        if (Math.abs(along)<.58) continue; // doorway
        const off=2+.012*Math.sin(j+i);
        for (const z of [.4,.85,1.35,1.75]) point(cos*along-sin*off,sin*along+cos*off,z);
    }
    for (let j=0;j<80;j++) point(j*.05,-2,1.6); // high, flat clutter
    return {pose,xyz};
}
for (let i=0;i<25;i++) {
    const {pose,xyz}=scan(i);
    const walls=h.update(xyz,pose,i*.2);
    assert.equal(h.update(xyz,pose,i*.2).length,walls.length);
    if (i<2) assert.equal(walls.length,0);
    if (i<8) continue;
    assert.equal(walls.length,2);
    for (const w of walls) {
        const along = [];
        for (const k of [0,1]) {
            const x=pose.x+Math.cos(pose.yaw)*w['x'+k]-Math.sin(pose.yaw)*w['y'+k];
            const y=pose.y+Math.sin(pose.yaw)*w['x'+k]+Math.cos(pose.yaw)*w['y'+k];
            assert.ok(Math.abs(-sin*x+cos*y-2)<.045);
            along.push(cos*x+sin*y);
        }
        assert.ok(along.every(v=>v<-.38) || along.every(v=>v>.38));
    }
}
assert.equal(h.update([],scan(24).pose,12).length,0);
console.log('Angled walls, doorway, moving observer, clutter and stale history passed');
const doorwayWalls = [
    {id:1,x0:-2,y0:2,x1:-.5,y1:2,top:1.91},
    {id:2,x0:2,y0:2,x1:.5,y1:2,top:1.96}
];
const openScan = [-.48,3,.8,.48,3,.8];
assert.equal(h.doorways(doorwayWalls,[],20).length,0);
assert.equal(h.doorways(doorwayWalls,openScan,20.2).length,0);
assert.equal(h.doorways(doorwayWalls,openScan,20.3).length,0);
assert.equal(h.doorways(doorwayWalls,[...openScan],20.4)[0].top,1.91);
assert.equal(h.doorways(doorwayWalls,[],21).length,1);
const closed = [-.2,2,1,-.1,2,1,.1,2,1,.2,2,1];
assert.equal(h.doorways(doorwayWalls,closed,21.2).length,0);
console.log('Doorway confirmation, frame height, occlusion and blocked gap passed');
