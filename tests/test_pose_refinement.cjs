// NODE_PATH=/tmp/x3-gui-check/node_modules node tests/test_pose_refinement.cjs
const assert = require('assert'), fs = require('fs'), path = require('path'), vm = require('vm');
global.THREE = require('three'); global.window = {};
for (const f of ['models/humanoid-walk.js', 'pose-refinement.js', 'humanoid.js'])
    vm.runInThisContext(fs.readFileSync(path.join(__dirname, '../src/web', f), 'utf8'));
const R=window.X3PoseRefinement, V=(x,y,z)=>new THREE.Vector3(x,y,z), forward=V(0,0,-1);
let leg=R.constrainLeg(V(0,-1,0), V(0,1,0),forward);
assert(leg.angle<=150*Math.PI/180+1e-9,'knee cannot fold through itself');
leg=R.constrainLeg(V(1,0,0),V(1,0,0),forward);
assert(leg.upper.distanceTo(V(1,0,0))<1e-8 && leg.lower.distanceTo(V(1,0,0))<1e-8,'straight sideways lift preserved');
leg=R.constrainLeg(V(0,-1,0),V(0,0,-1),forward);
assert(leg.lower.z>=0,'backward knee must not hyperextend into the forward half plane');
leg=R.constrainLeg(V(0,0,-1),V(0,-1,0),forward);
assert(leg.lower.distanceTo(V(0,-1,0))<1e-8,'seated forward thigh with shin down preserved');
for(let i=0;i<1000;i++) {
 const u=V(Math.sin(i),Math.cos(i*0.31),Math.sin(i*.17)).normalize();
 const v=V(Math.cos(i*.71),Math.sin(i*.41),Math.cos(i)).normalize();
 const out=R.constrainLeg(u,v,forward);
 assert(out.upper.toArray().concat(out.lower.toArray()).every(Number.isFinite));
 assert(out.upper.angleTo(out.lower)<=150*Math.PI/180+1e-8);
}
const hip=V(0,0.8,0),knee=V(0,0.4,-0.2),ankle=V(0,0,0),target=V(.02,0,0);
const solved=R.solveLeg(hip,knee,ankle,target);
assert(solved);
const solvedKnee=hip.clone().addScaledVector(solved.upper,hip.distanceTo(knee));
const solvedAnkle=solvedKnee.clone().addScaledVector(solved.lower,knee.distanceTo(ankle));
assert(solvedAnkle.distanceTo(target)<1e-8,'contact IK meets target with fixed bone lengths');
assert(!R.solveLeg(hip,knee,ankle,V(0,-2,0)),'unreachable contact releases');
const c=R.createContact();
for(let i=0;i<30;i++) R.contactTarget(c,ankle,0,.95,1/60,true);
assert(c.anchor,'stable supported foot plants');
assert(R.contactTarget(c,V(.003,0,0),0,.95,1/60,true).distanceTo(ankle)<1e-8,'small noise keeps original contact');
assert(!R.contactTarget(c,V(.003,.08,0),.08,.95,1/60,true),'lift releases immediately');
for(let i=0;i<30;i++) R.contactTarget(c,ankle,0,.95,1/60,true);
assert(!R.contactTarget(c,ankle,0,.3,1/60,true),'low confidence releases');
for(let i=0;i<30;i++) R.contactTarget(c,ankle,0,.95,1/60,true);
assert(!R.contactTarget(c,ankle,0,.95,1/60,false),'robot motion disables contact');
const k=Array(17).fill(null), scores=Array.from({length:17},()=>[0,0,.95]);
k[5]=[.2,-.5,2]; k[6]=[-.2,-.5,2]; k[11]=[.12,0,2]; k[12]=[-.12,0,2];
assert(R.facingCue(k,scores).yaw===Math.PI,'left/right frontal ordering faces camera');
for(const i of [5,6,11,12])k[i][0]*=-1;
assert(R.facingCue(k,scores).yaw===0,'back ordering faces away');
for(const i of [5,6,11,12])k[i][0]*=.1;
assert(!R.facingCue(k,scores),'side-on width is ambiguous');
k[5]=[.2,-.5,2]; k[6]=[-.2,-.5,2];k[11]=[-.12,0,2];k[12]=[.12,0,2];
assert(!R.facingCue(k,scores),'disagreeing hips and shoulders rejected');
console.log('PASS: knee/hip constraints, sideways lift, sitting, contact IK/release and conservative facing cues.');
const obs=(x,y,z)=>({direction:V(x,y,z).normalize(),confidence:.95});
const stance={left_thigh:obs(0,-1,-.3),left_shin:obs(0,-1,.3),right_thigh:obs(0,-1,-.3),right_shin:obs(0,-1,.3)};
const a=window.X3Humanoid.create();
function tick(pose,n=1,options={}){for(let i=0;i<n;i++){a.animate(0,1/60);a.pose(pose,1/60,options);}a.group.updateMatrixWorld(true);}
const point=n=>a.joints[n].getWorldPosition(V(0,0,0));
tick(stance,120);
assert(a.group.userData.footContacts.length===2,'supported bent stance acquires both contacts');
const anchor=point('left_foot');
const originalLengths=[point('left_thigh').distanceTo(point('left_shin')),point('left_shin').distanceTo(point('left_foot'))];
let maxDrift=0,minFloor=Infinity;
for(let i=0;i<120;i++) {
 const jitter={...stance,left_thigh:obs(.006*Math.sin(i/10),-1,-.3)};
 tick(jitter);
 maxDrift=Math.max(maxDrift,point('left_foot').distanceTo(anchor));
 minFloor=Math.min(minFloor,new THREE.Box3().setFromObject(a.joints.left_foot).min.y);
}
assert(maxDrift<.002,`contact should suppress small pose jitter: ${maxDrift}`);
assert(minFloor>-.006,`contact must not sink foot: ${minFloor}`);
assert(Math.abs(point('left_thigh').distanceTo(point('left_shin'))-originalLengths[0])<1e-8);
assert(Math.abs(point('left_shin').distanceTo(point('left_foot'))-originalLengths[1])<1e-8);
tick({...stance,left_thigh:obs(0,0,-1)},60);
assert(!a.group.userData.footContacts.includes('left'),'lifted avatar foot releases contact');
tick(stance,120); tick(stance,1,{contacts:false});
assert(a.group.userData.footContacts.length===0,'moving robot releases avatar contacts');
tick(null,120);
assert(a.group.userData.poseConfidence===0 && a.group.userData.footContacts.length===0);
assert.deepStrictEqual(a.group.position.toArray(),[0,0,0]);
console.log('PASS: rendered skeleton contacts, jitter suppression, bone lengths, floor, lift and dropout.',{maxDrift,minFloor});
