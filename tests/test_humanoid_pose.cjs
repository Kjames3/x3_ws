// NODE_PATH=/tmp/x3-gui-check/node_modules node tests/test_humanoid_pose.cjs
const assert = require('assert'), fs = require('fs'), vm = require('vm'), path = require('path');
global.THREE = require('three'); global.window = {};
for (const f of ['models/humanoid-walk.js', 'pose-refinement.js', 'humanoid.js'])
    vm.runInThisContext(fs.readFileSync(path.join(__dirname, '../src/web', f), 'utf8'));
const obs = (x,y,z,c=0.95) => ({direction:new THREE.Vector3(x,y,z).normalize(), confidence:c});
const standing = {left_thigh:obs(0,-1,0), left_shin:obs(0,-1,0), right_thigh:obs(0,-1,0), right_shin:obs(0,-1,0)};
function tick(a, pose, n=180, speed=0) { for(let i=0;i<n;i++){a.animate(speed,1/60); a.pose(pose,1/60);} a.group.updateMatrixWorld(true); }
function pos(a,n){return a.joints[n].getWorldPosition(new THREE.Vector3());}
function floor(a){return Math.min(...['left_foot','right_foot'].map(n=>new THREE.Box3().setFromObject(a.joints[n]).min.y));}
const a=window.X3Humanoid.create(); a.group.position.set(2,0,-3);
tick(a,standing); const hip=pos(a,'pelvis').y;
tick(a,{...standing,left_thigh:obs(1,0,0)});
assert(pos(a,'left_foot').y > pos(a,'right_foot').y+0.2, 'one leg lifts independently while stationary');
assert(Math.abs(floor(a))<0.006, 'support foot stays grounded');
tick(a,{...standing,left_thigh:obs(1,0,0),right_thigh:obs(1,0,0)});
assert(pos(a,'pelvis').y < hip-0.25, 'seated bend lowers pelvis');
assert(Math.abs(floor(a))<0.006, 'seated feet stay grounded');
assert.deepStrictEqual(a.group.position.toArray(),[2,0,-3]);
const bent=a.joints.left_thigh.quaternion.clone(); tick(a,null,6);
assert(a.joints.left_thigh.quaternion.angleTo(bent)<0.05, 'brief dropout holds pose');
tick(a,null); assert(a.group.userData.poseConfidence===0, 'stale pose fades');
const neutral=a.joints.left_thigh.quaternion.clone();
tick(a,{left_thigh:obs(1,0,0,0.2)});
assert(a.joints.left_thigh.quaternion.angleTo(neutral)<1e-6, 'low confidence cannot pose leg');
tick(a,{left_thigh:obs(NaN,0,0)});
assert(a.joints.left_thigh.quaternion.toArray().every(Number.isFinite));
a.group.rotation.y=Math.PI/2; tick(a,standing);
const down=pos(a,'left_shin').sub(pos(a,'left_thigh')).normalize();
assert(down.dot(new THREE.Vector3(0,-1,0))>0.99, 'world-space fitting survives heading changes');
tick(a,standing,180,0.6); assert(Math.abs(floor(a))<0.006);
console.log('PASS: independent leg, seated pelvis, support floor, fixed track root, dropout, confidence, invalid data, heading and walking.');
// Exercise the actual scene keypoint adapter without creating a WebGL renderer.
const sceneSource=fs.readFileSync(path.join(__dirname,'../src/web/scene-view.js'),'utf8');
const adapter=sceneSource.slice(sceneSource.indexOf('    const BODY_KPTS'),sceneSource.indexOf('    // Avatars follow C3'));
const context={THREE, window}; vm.createContext(context); vm.runInContext(adapter+'\nthis.adapt = poseDirs;',context);
const k=Array.from({length:17},(_,i)=>[i%2,Math.floor(i/2)*0.1,2]);
const scores=Array.from({length:17},()=>[100,100,0.95]);
const det={label:'person',xyz_m:{x:0,z:2},keypoints_xyz:k,keypoints:scores};
let dirs=context.adapt({x:0,z:2},[det],new Set());
assert(dirs.left_thigh && dirs.right_shin && dirs.torso);
scores[13][2]=0.1; dirs=context.adapt({x:0,z:2},[det],new Set());
assert(!dirs.left_thigh && !dirs.left_shin && dirs.right_thigh,'weak knee rejects both connected bones');
assert(context.adapt({x:5,z:2},[det],new Set())===null,'distant track is not matched');
assert(context.adapt({x:0,z:2},[det],new Set([0]))===null,'one detection cannot pose two people');
console.log('PASS: scene adapter endpoint confidence, torso, track distance and unique association.');
const headingCode=sceneSource.slice(sceneSource.indexOf('    function updateHeading'),sceneSource.indexOf('    function updatePeople'));
vm.runInContext(headingCode+'\nthis.heading = updateHeading;',context);
const person={group:new THREE.Group()}; person.group.rotation.y=1.2;
for(let i=0;i<120;i++) context.heading(person,0.03*Math.sin(i),2,0.4,1/60);
assert(Math.abs(person.group.rotation.y-1.2)<1e-8,'posture position/velocity noise must preserve heading');
for(let i=0;i<180;i++) context.heading(person,i/120,2,0.5,1/60);
assert(Math.abs(person.group.rotation.y+Math.PI/2)<0.05,'sustained travel updates heading');
const stopped=person.group.rotation.y;
for(let i=0;i<120;i++) context.heading(person,1.5,2,0,1/60);
assert(Math.abs(person.group.rotation.y-stopped)<0.05,'stopping preserves travel heading');
console.log('PASS: heading rejects stationary jitter, follows sustained travel and persists at rest.');
const facingPerson={group:new THREE.Group()};
const cue={yaw:Math.PI,confidence:.95};
for(let i=0;i<20;i++) context.heading(facingPerson,0,2,0,1/60,cue);
assert(facingPerson.group.rotation.y===0,'brief facing evidence must not flip heading');
for(let i=0;i<160;i++) context.heading(facingPerson,0,2,0,1/60,cue);
assert(Math.abs(facingPerson.group.rotation.y-Math.PI)<.01,'sustained frontal evidence updates heading at rest');
for(let i=0;i<120;i++) context.heading(facingPerson,0,2,0,1/60,null);
assert(Math.abs(facingPerson.group.rotation.y-Math.PI)<.01,'ambiguous side-on view holds last heading');
console.log('PASS: body-facing dwell, stationary turn and ambiguous-view hold.');
