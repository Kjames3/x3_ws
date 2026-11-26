// Deterministic mapped-room replay. Coordinates are metres in base_footprint.
const assert = require('node:assert/strict');
const {analyze, Tracker} = require('../../src/web/reality-structure');
const tracker = new Tracker();
const pose = {x:0, y:0, theta:0};
const furniture = ['chair', 'couch', 'tv', 'dining table'].map((label,i) =>
    ({label, conf:.9, xyz_base_m:{x:2, y:i, z:1.5}}));
function step(t, detections=[], xyz=[]) {
    return analyze([], detections, pose, tracker, t, {mapped:true, xyz, detectionStamp:t}).objects;
}
step(0,furniture);
const before = step(.2,furniture);
assert.equal(before.length,4);
assert.ok(before.find(o => o.kind === 'tv').bottom > 1);
pose.theta = Math.PI;
const away = step(12);
assert.deepEqual(away.map(o => o.id),before.map(o => o.id));
assert.ok(Math.abs(away[0].yaw + pose.theta - before[0].yaw) < 1e-8);
pose.theta = 0;
const moved = {...furniture[0], xyz_base_m:{x:3,y:1,z:.4}};
const rays = [4,0,.36, 4,.02,.38, 4,-.03,.40, 4,.04,.42];
for (const t of [13,14,15]) step(t,[moved],[...rays]);
const after = step(16);
assert.equal(after.length,4);
assert.ok(!after.some(o => o.id === before[0].id));
assert.equal(after.find(o => o.kind === 'chair').x,3);
console.log('Mapped-room replay passed');

// Exercise production scene placement with Three.js matrices and a doorway map.
const THREE = require('../../src/web/vendor/three/three.min.js');
global.window = global;
global.RealityStructure = require('../../src/web/reality-structure');
const button = {};
global.document = {getElementById: () => button};
const storage = new Map();
global.localStorage = {getItem:k => storage.get(k), setItem:(k,v) => storage.set(k,v)};
require('../../src/web/reality-mode');
const scene = new THREE.Scene();
scene.background = new THREE.Color();
scene.fog = new THREE.Fog(0,1,10);
const view = RealityMode.create(THREE,scene,{setClearColor(){}},
    {floor:new THREE.Mesh(), rings:[], tinted:[]});
const pixels = new Uint8Array(20*20).fill(255);
for (let i=2;i<18;i++) if (i<8 || i>11) pixels[9*20+i]=0;
const nav = {roomMap:'replay', rawPixels:pixels, rawPixelW:20, rawPixelH:20,
    mapMeta:{resolution:.1,origin:[0,0]}};
const data = {robotPoseMap:{x:0,y:0,theta:0}};
view.update(true,data,nav,0,[]);
assert.ok(view.isKnownWall(.4,1.05));
assert.ok(!view.isKnownWall(1.0,1.05));
view.analyzeStructure([],[],data,'replay',[],0);
global.prompt = () => '1,2,90,.8,.4,1.1';
button.onclick();
const cabinet = () => view.analyzeStructure([],[],data,'replay',[],100).objects[0];
assert.equal(cabinet().kind,'cabinet');
assert.equal(cabinet().x,1);
data.robotPoseMap = {x:100,y:100,theta:Math.PI/2};
view.update(true,data,nav,100,[]);
assert.ok(view.isKnownWall(.05,.6));
assert.ok(Math.abs(cabinet().x-1) < 1e-8);
assert.ok(Math.abs(cabinet().y) < 1e-8);
assert.equal(cabinet().yaw,0);
console.log('Mapped walls, doorway and cabinet placement passed');
