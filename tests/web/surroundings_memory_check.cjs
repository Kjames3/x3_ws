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
// 2.5 cm near mode: mixed cells, memory near-only, far floor still observed.
{
    const {voxelizeMixed, NEAR, FAR, NEAR_RADIUS} = require('../../src/web/surroundings-memory.js');
    const near2 = [1.005,.005,.2, 1.035,.005,.2];          // two 2.5 cm cells, one 5 cm cell
    const far2 = [4.005,.005,.2, 4.035,.005,.2];           // one 5 cm cell beyond 3 m
    const cells = voxelizeMixed([...near2, ...far2]);
    assert.equal(cells.filter(c => c.s === NEAR).length, 2);
    assert.equal(cells.filter(c => c.s === FAR).length, 1);
    assert.equal(voxelizeMixed([...near2, ...far2], {far: false}).length, 2);
    m = new Memory(NEAR); assert.equal(m.size, NEAR);
    for (let i = 0; i < 3; i++) m.ingest([...near2, ...far2, 4.5,0,0], pose(i*.2));
    assert.equal(m.cells.size, 2, 'memory keeps only near cells in 2.5 cm mode');
    assert(m.floor.size >= 1, 'far floor returns still mark coverage');
    // A near cell that ends up beyond the radius after the robot moves is dropped.
    let p; for (let i = 1; i <= 4; i++) { p = pose(.4 + i*.2, -.6*i, 0, 0); m.ingest([], p); }
    assert.equal(m.cells.size, 0, 'near cells now 3.4 m away are pruned');
    assert(m.setSize(.05) && m.size === .05 && m.cells.size === 0);
    console.log('2.5 cm near mode: mixed voxelize, near-only memory and far floor coverage passed');
}
