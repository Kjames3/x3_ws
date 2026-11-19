const assert = require('node:assert/strict');
const {Memory, world, local, LIMIT} = require('../../src/web/surroundings-memory.js');
const pose = (stamp,x=0,y=0,yaw=0) => ({stamp,x,y,yaw});
let m = new Memory();
// Translation and rotation must preserve the SAME occupied world cell.
for(let i=0;i<4;i++) {
    const p=pose(i*.2, i*.03, 0, i*.1), [x,y]=local(2.03,.03,p);
    m.ingest([x,y,.25],p);
}
assert.equal(m.cells.size,1);assert.equal(m.snapshot(pose(.6)).cells[0].confidence,1);
// Single-frame speckle is removed; established wall survives missed scans.
m.ingest([1,1,.8],pose(.8));m.ingest([],pose(1.4));
assert.equal(m.cells.size,1);
// Observed floor only: a high ray does not mark the intervening floor free.
m=new Memory();m.ingest([3,0,0, 2,2,1],pose(0));assert.equal(m.floor.size,1);
assert.equal(m.snapshot(pose(0)).floor.length,1);
// Three distinct measured rays through a previously occupied cell clear it.
m=new Memory();for(let i=0;i<3;i++)m.ingest([1.05,0,.31],pose(i*.2));
for(let i=0;i<3;i++)m.ingest([2.05,0,.31],pose(.6+i*.2));
assert(![...m.cells.values()].some(c=>Math.abs(c.x-1.05)<.01));
// Person masks never gain persistent geometry or leave a stale cell behind.
m=new Memory();for(let i=0;i<4;i++)m.ingest([1,0,1],pose(i*.2),[{x:1,y:0}]);
assert.equal(m.cells.size,0);assert.equal(m.snapshot(pose(.6)).cells.length,1);
// Pose discontinuity/reconnect clears old geometry.
m=new Memory();for(let i=0;i<3;i++)m.ingest([1,0,1],pose(i*.2));
m.ingest([],pose(.6,5));assert.equal(m.cells.size,0);
// Hard bound and expired geometry.
m=new Memory();const pts=[];for(let i=0;i<15000;i++)pts.push((i%100)*.11,Math.floor(i/100)*.11,.2);
m.ingest(pts,pose(0));assert(m.cells.size<=LIMIT);
m.ingest([],pose(10));assert.equal(m.cells.size,0);
console.log('Surroundings memory: alignment, persistence, floor evidence, ray clearing, people masks, resets and limits passed');
// Fine display preserves samples that share one coarse cell; clearance stays independent.
const {voxelize} = require('../../src/web/surroundings-memory.js');
const geometry = require('../../src/web/surroundings-geometry.js');
const pair = [1.01,.01,.21, 1.06,.01,.21];
assert.equal(voxelize(pair,.1).length,1);
assert.equal(voxelize(pair,.05).length,2);
assert.equal(geometry.CELL,.1);
for (const size of [.05,.1]) {
    m=new Memory(size);
    for(let i=0;i<3;i++)m.ingest([1.025,0,.31],pose(i*.2));
    for(let i=0;i<3;i++)m.ingest([2.05,0,.31],pose(.6+i*.2));
    assert.equal(m.cells.size,1,'measured rays clear old cells at either size');
    m.ingest(pts,pose(1.2));assert(m.snapshot(pose(1.2)).cells.length<=LIMIT);
    assert(m.setSize(size===.05?.1:.05));assert.equal(m.cells.size,0);
    assert.equal(m.previous,null);
}
assert.equal(voxelize([NaN,0,0,0,0,-1],.05).length,0);
console.log('Fine/coarse grids, size reset, ray clearing and shared capacity passed');
