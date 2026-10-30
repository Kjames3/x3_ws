const assert=require('node:assert/strict');
const {Tracker,clusters}=require('../../src/web/person-cloud-tracker.js');
const pose=(stamp,x=0,y=0,yaw=0)=>({stamp,x,y,yaw});
const camera=(stamp,x=2,y=0,id=1)=>({id,odom_x:x,odom_y:y,odom_vx:0,odom_vy:0,state_stamp_s:stamp,obs_age_s:0,measured:true});
function person(x=2,y=0,p=pose(0)) {
    const out=[];
    for(let dx=-.12;dx<=.12;dx+=.12)for(let dy=-.15;dy<=.15;dy+=.15)for(let z=.3;z<=1.5;z+=.3){
        const wx=x+dx-p.x,wy=y+dy-p.y;
        out.push(Math.cos(p.yaw)*wx+Math.sin(p.yaw)*wy,-Math.sin(p.yaw)*wx+Math.cos(p.yaw)*wy,z);
    }
    return out;
}
let t=new Tracker(),r;
for(let i=0;i<4;i++)r=t.update(person(),pose(i*.2),[camera(i*.2)],0);
assert.equal(r.length,1);assert.equal(r[0].mode,'camera + lidar');assert(r[0].points>=8);assert(Math.abs(r[0].x-2)<.1);
// No camera seed: arbitrary person-shaped furniture is never classified.
t=new Tracker();assert.equal(t.update(person(),pose(0),[],0).length,0);
// Do not refresh stale semantic evidence through repeated telemetry.
t=new Tracker();for(let i=0;i<9;i++)r=t.update(person(),pose(i*.2),[camera(0)],0);assert.equal(r.length,0);
// Camera dropout may continue on fresh lidar for a short bounded interval.
t=new Tracker();for(let i=0;i<3;i++)t.update(person(),pose(i*.2),[camera(i*.2)],0);
r=t.update(person(),pose(.9),[],0);assert.equal(r[0].mode,'lidar only');
assert(r[0].sigma>.2);r=t.update(person(),pose(1.7),[],0);assert.equal(r.length,0);
// Repeated cloud frames cannot count toward confirmation.
t=new Tracker();for(let i=0;i<5;i++)r=t.update(person(),pose(0),[camera(0)],0);assert.equal(r[0].mode,'lidar tentative');
// Turning/translating robot sees the same world person.
t=new Tracker();for(let i=0;i<5;i++){const p=pose(i*.2,i*.04,0,i*.15);r=t.update(person(2,0,p),p,[camera(p.stamp)],0);}
assert.equal(r[0].mode,'camera + lidar');
const p=pose(.8,.16,0,.6),wx=p.x+Math.cos(p.yaw)*r[0].x-Math.sin(p.yaw)*r[0].y,wy=p.y+Math.sin(p.yaw)*r[0].x+Math.cos(p.yaw)*r[0].y;
assert(Math.hypot(wx-2,wy)<.1);
// A complete wall is rejected before camera gating can crop it to person size.
const wall=[];for(let y=-2;y<=2;y+=.1)for(let z=.3;z<=1.8;z+=.2)wall.push(2,y,z);
assert.equal(clusters(wall).length,0);t=new Tracker();r=t.update(wall,pose(0),[camera(0)],0);assert.equal(r[0].points,0);
// Competing camera identities may not both claim a shared cluster.
t=new Tracker();r=t.update(person(),pose(0),[camera(0,2,-.1,1),camera(0,2,.1,2)],0);assert(r.every(x=>x.points===0));
// Unequal distances do not make a merged body cluster safe to assign.
t=new Tracker();r=t.update(person(),pose(0),[camera(0,2,0,1),camera(0,2,.3,2)],0);assert(r.every(x=>x.points===0));
// Separate people get distinct exclusive point sets.
t=new Tracker();r=t.update([...person(2,-.7),...person(2,.7)],pose(0),[camera(0,2,-.7,1),camera(0,2,.7,2)],0);
assert.equal(r.length,2);assert(r.every(x=>x.points>0));assert.equal(new Set(t.labels.values()).size,2);
// Sparse/invalid returns don't establish support; missing pose clears state.
t=new Tracker();r=t.update([2,0,.5,NaN,0,1],pose(0),[camera(0)],0);assert.equal(r[0].points,0);
assert.equal(t.update(person(),null,[camera(0)],0).length,0);
// An old camera packet cannot create a new identity even if points happen to match.
t=new Tracker();assert.equal(t.update(person(),pose(0),[camera(0)],2).length,0);
// Camera-only positions follow fresh camera evidence when lidar disappears.
t=new Tracker();t.update(person(),pose(0),[camera(0)],0);r=t.update([],pose(.2),[camera(.2,2.3)],0);assert.equal(r[0].x,2.3);
console.log('Person/cloud tracking: semantic seeds, exclusive matching, walls, ambiguity, motion alignment, dropout deadlines, duplicate frames and stale input passed');
