// Use the production scene and tracker. Map coordinates are translated and
// rotated 90 degrees from odometry, and the camera moves between captures.
const assert = require('node:assert/strict');
const THREE = require('../../src/web/vendor/three/three.min.js');
global.window = global;
global.RealityStructure = require('../../src/web/reality-structure');
const {Lookalike, Tracker} = RealityStructure;
global.document = {getElementById: () => ({})};
global.localStorage = {getItem: () => null};
let receive, saved;
global.addEventListener = (_,fn) => { receive=fn; };
global.sendMessage = msg => { if (msg.objects) saved=msg.objects; };
require('../../src/web/reality-mode');
const scene=new THREE.Scene();
scene.background=new THREE.Color();
scene.fog=new THREE.Fog(0,1,10);
const view=RealityMode.create(THREE,scene,{setClearColor(){}},{floor:new THREE.Mesh(),rings:[],tinted:[]});
const box={kind:'table', X:4,Y:6,yaw:Math.PI/2,L:1.6,D:.8,H:.9,bottom:.1};
const d={};
function pose(x,y,theta) {
    d.robotPose={x:x*100,y:y*100,theta};
    d.robotPoseMap={x:(4-y)*100,y:(5+x)*100,theta:theta+Math.PI/2};
}
pose(0,0,0);
view.analyzeStructure([],[],d,'room',[],0);
receive({detail:{key:'map:room',objects:[box]}});
const pts=[];
for (let i=0;i<10;i++) for (let j=0;j<10;j++) pts.push(1+i*.025,j*.025,.55);
// Below the bottom, outside the footprint, and at the detected person's feet.
pts.push(1.1,0,.11, 4,0,.55, .3,0,.55);
function cloud(stamp, red) {
    const a=new Float32Array(pts);
    a.colors=new Float32Array(a.length);
    for(let i=0;i<a.length;i+=3) { a.colors[i]=red; a.colors[i+2]=1-red; }
    a.stamp=stamp;
    return a;
}
function feed(time,stamp,red) {
    d.oakCloud=cloud(stamp,red);
    const x=d.robotPose.x/100,y=d.robotPose.y/100,theta=d.robotPose.theta;
    const dx=.3-x,dy=-y;
    const people=[{x:Math.cos(theta)*dx+Math.sin(theta)*dy,y:-Math.sin(theta)*dx+Math.cos(theta)*dy}];
    return view.analyzeStructure([],[],d,'room',[],time,null,{x,y,yaw:theta},people).objects[0].model;
}
feed(10,1,1);
assert.equal(feed(20,1,1).voxels.size,0,'repeat delivery cannot confirm points');
let m=feed(100,2,0);
assert.equal(m.voxels.size,100);
assert.ok(!Lookalike.ready(m),'one view retains stand-in');
pose(1,1,-Math.PI/2);
m=feed(2200,3,1);
assert.equal(m.voxels.size,100,'robot motion preserves voxel identity');
assert.ok(Lookalike.ready(m));
const sample=[...m.voxels.values()].find(v=>v.x===2 && v.y===3);
assert.equal(sample.z,18);
assert.equal(sample.hits,3);
assert.equal(sample.r,170);
assert.equal(sample.b,85);
const points=scene.getObjectByProperty('type','Points');
assert.equal(points.geometry.drawRange.count,100);
const parts = points.parent.children[0].children;
assert.ok(parts.slice(0,-1).every(mesh => !mesh.visible));
assert.ok(parts.at(-1).visible, 'measured footprint remains visible');
scene.updateMatrixWorld(true);
const vertex = [...m.voxels.values()].indexOf(sample);
const rendered = points.localToWorld(new THREE.Vector3().fromBufferAttribute(points.geometry.attributes.position,vertex));
// Base axes after rotating the robot by -90 degrees: forward = -odom Y, left = odom X.
assert.ok(rendered.distanceTo(new THREE.Vector3(-.05,.55,-.925))<1e-6);
const tracker=new Tracker();
tracker.importLandmarks(JSON.parse(JSON.stringify(saved)),3);
assert.deepEqual(tracker.exportLandmarks(),saved);
assert.ok(Lookalike.ready(tracker.tracks[0].model));
const proposals=Lookalike.viewpoints(box,m.sectors);
assert.equal(proposals.length,3);
for(const p of proposals) {
    const dx=box.X-p.x,dy=box.Y-p.y;
    assert.ok(Math.hypot(dx,dy)>=1 && Math.hypot(dx,dy)<=1.5);
    assert.ok(Math.abs(Math.atan2(dy,dx)-p.yaw)<1e-8);
}
const full=Lookalike.create(), xyz=[];
for(let x=-25;x<25;x++) for(let y=-15;y<15;y++) for(let z=1;z<=5;z++)
    xyz.push(x*.025,y*.025,z*.025);
const target={x:0,y:0,yaw:0,L:2,D:1,H:1};
for(let stamp=1;stamp<=2;stamp++) {
    const a=new Float32Array(xyz);
    a.colors=new Float32Array(a.length).fill(.5);
    a.stamp=stamp;
    Lookalike.ingest(full,a,target,{x:2,y:0},[]);
    assert.ok(full.pending.size+full.voxels.size<=Lookalike.CAP);
}
assert.equal(full.voxels.size,4000);
const encoded=Lookalike.pack(full);
assert.equal(Lookalike.unpack(encoded).voxels.size,4000);
assert.ok(encoded.length<65536);
assert.equal(Lookalike.unpack(encoded+encoded),undefined);
console.log('Object-frame fusion, moving robot, person/floor masks, render threshold, session round trip, cap and viewpoints passed');
