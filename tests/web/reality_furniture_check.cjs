const assert = require('node:assert/strict');
const {Tracker, analyze} = require('../../src/web/reality-structure');
const cells=[];
function surface(x0,l,d,h) {
    for(let i=0;i<l;i++) for(let j=0;j<d;j++) {
        const x=x0+i*.12+.01, y=j*.12+.01;
        cells.push({x,y,z:h});
        if((i===0 || i===l-1) && (j===0 || j===d-1)) cells.push({x,y,z:.2});
        if(h<.5 && j===0) cells.push({x,y,z:.82});
    }
}
surface(1.2,12,6,.74);
surface(3.6,4,4,.45);
const tracker=new Tracker(), pose={x:0,y:0,theta:0};
const step=(points,frame,t)=>analyze(points,[],pose,tracker,t,{xyz:frame}).objects;
const same=[];
for(let i=0;i<6;i++) assert.equal(step(cells,same,i*.01).length,0);
let found;
for(let i=1;i<=5;i++) found=step(cells,[],i*.2);
assert.deepEqual(found.map(o=>o.kind).sort(),['chair','table']);
assert.ok(Math.abs(found.find(o=>o.kind==='chair').seat-.45)<.04);
assert.ok(Math.abs(found.find(o=>o.kind==='table').H-.74)<.04);
assert.equal(step(cells.map(p=>({...p,age:4})),[],4).length,0);
console.log('Furniture geometry, confirmation, stale evidence and expiry passed');
const {furnitureOverlap,furnitureSurfaces} = require('../../src/web/reality-structure');
const desk = {kind:'table',x:0,y:0,L:1.7,D:.65,yaw:Math.PI/4};
assert.ok(furnitureOverlap(desk,{...desk,x:.65,y:.65}));
assert.ok(!furnitureOverlap(desk,{...desk,x:-.5,y:.5}));
const seat = furnitureSurfaces(cells).find(o=>o.kind==='chair');
// The raised returns lie along the -Y edge. Local back is +Y in fit coordinates.
assert.ok(Math.cos(seat.yaw)<-.95);
