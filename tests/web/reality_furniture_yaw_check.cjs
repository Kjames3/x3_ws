// Synthetic paired replay through the production fit, match and confirmation paths.
const assert = require('node:assert/strict');
const {analyze, furnitureSurfaces, Tracker} = require('../../src/web/reality-structure');
const {voxelize} = require('../../src/web/surroundings-memory');

const rad = deg => deg*Math.PI/180;
const deg = radians => radians*180/Math.PI;
const delta = (a,b,k) => Math.atan2(Math.sin(k*(a-b)),Math.cos(k*(a-b)))/k;
const rng = seed => () => ((seed = (Math.imul(1664525,seed)+1013904223)>>>0)/2**32);
const origin = {x:0,y:0,theta:0};

function sample(kind, square, partial, view, random) {
    const points=[];
    const add=(u,v,z)=> {
        if (random()<.18) return;
        points.push([u+(random()-.5)*.024,v+(random()-.5)*.024,z+(random()-.5)*.02]);
    };
    const table=kind==='table', couch=kind==='couch';
    const L=table ? (square ? .82 : 1.27) : (couch ? 1.82 : .54);
    const D=table ? (square ? .80 : .74) : (couch ? .86 : .52);
    const seat=table ? .745 : .445;
    for(let u=-L/2;u<=L/2;u+=.028) for(let v=-D/2;v<=D/2;v+=.028) {
        // A dominant face plus a portion of the adjacent top/seat, or a diagonal
        // occluder. Face changes as the observer goes around the piece.
        if (partial===1 && (view%2 ? Math.sign(Math.cos(view))*u : Math.sign(Math.sin(view))*v) >
                (view%2 ? L : D)*.13) continue;
        if (partial===2 && u/L+v/D>.37) continue;
        add(u,v,seat);
    }
    if (!table) {
        for(let v=D/2-.035;v<=D/2;v+=.025) for(let u=-L/2;u<=L/2;u+=.025) {
            if (partial===1 && Math.sign(Math.cos(view))*u>L*.10) continue;
            if (partial===3) continue; // single side/arm, with no view of the back
            for(let z=.49;z<=.87;z+=.035) add(u,v,z);
        }
        if (couch) for (const u of [-L/2+.025,L/2-.025]) {
            const top=partial===3 && u>0 ? .73 : .63;
            for(let v=-D/2+.04;v<D/2;v+=.033) for(let z=.22;z<=top;z+=.04) add(u,v,z);
        }
    }
    for (const u of [-L/2+.06,L/2-.06]) for (const v of [-D/2+.045,D/2-.045]) {
        for (let z=.10;z<seat;z+=.04) add(u,v,z);
    }
    return points;
}

function frame(kind,square,partial,heading,centre,pose,seed,view) {
    const c=Math.cos(heading),s=Math.sin(heading),pc=Math.cos(pose.theta),ps=Math.sin(pose.theta);
    const xyz=[];
    for (const [u,v,z] of sample(kind,square,partial,view,rng(seed))) {
        const dx=centre.x+c*u-s*v-pose.x,dy=centre.y+s*u+c*v-pose.y;
        xyz.push(pc*dx+ps*dy,-ps*dx+pc*dy,z);
    }
    return {xyz,cells:voxelize(xyz,seed%3===0 ? .05 : .025)};
}

function detections(kind,centre,pose) {
    const dx=centre.x-pose.x,dy=centre.y-pose.y,c=Math.cos(pose.theta),s=Math.sin(pose.theta);
    return [{label:kind,conf:.9,xyz_base_m:{x:c*dx+s*dy,y:-s*dx+c*dy,z:.55}}];
}

const count=16, frames=48, start=32;
function run(kind,v2) {
    const errors=[];
    let shown=0, flips=0;
    for(let j=0;j<count;j++) {
        const square=kind==='table' && j%2===0, symmetry=kind==='table' ? (square ? 4 : 2) : 1;
        const centre={x:1.8,y:-.2}, yaw=rad(-171+37*j);
        const tracker=new Tracker({furnitureYawV2:v2});
        let last;
        for (let i=0;i<frames;i++) {
            const a=rad(j*29+i*2.5);
            const observed={x:centre.x+2*Math.cos(a),y:centre.y+2*Math.sin(a),theta:a+Math.PI};
            // Vary readout lag in half the replays. Identical stacks for A and B.
            const lag=j%2 ? rad(12+4*Math.sin(i/3)) : 0;
            const pose={x:observed.x-.06*Math.sin(a),y:observed.y+.06*Math.cos(a),theta:observed.theta+lag};
            const view=Math.floor((a-yaw)/rad(45));
            let partial=j%3;
            if (kind!=='table' && i<12 && j%4===0) partial=3;
            const {xyz,cells}=frame(kind,square,partial,yaw,centre,observed,2000*j+i+1,view);
            // Include semantic-only couch, a mix of labeled and lidar-only chairs/tables.
            const det=kind==='couch' || j%3!==2 ? detections(kind,centre,pose) : [];
            const out=analyze(cells,det,pose,tracker,i*.15,{xyz,mapped:true,observationPose:observed,detectionStamp:Math.floor(i/3)});
            if (i<start) continue;
            const object=out.objects.filter(o=>o.kind===kind).sort((a,b)=>a.id-b.id)[0];
            if (!object) continue;
            shown++;
            const actual=object.yaw+pose.theta;
            errors.push(Math.abs(deg(delta(actual,yaw,symmetry))));
            if (last!==undefined && Math.abs(deg(delta(actual,last,symmetry)))>(square ? 30 : 45)) flips++;
            last=actual;
        }
    }
    errors.sort((a,b)=>a-b);
    return {median:+errors[Math.floor(errors.length/2)].toFixed(1),
        within:Math.round(1000*errors.filter(e=>e<=10).length/errors.length)/10,
        flips, shown, total:count*(frames-start)};
}

for (const kind of ['table','chair','couch']) {
    const before=run(kind,false), after=run(kind,true);
    console.log(kind,JSON.stringify({before,after}));
    assert.ok(after.shown>=before.shown-3,kind+' replay coverage');
    assert.ok(after.median<before.median && after.within>before.within,kind+' paired replay improves yaw');
    assert.ok(after.flips<=before.flips,kind+' angular discontinuities');
}

// A restored couch turned by 180 degrees requires new geometry. Repainting
// the same cloud and temporary arm-only observations carry no turn votes.
{
    const tracker=new Tracker({furnitureYawV2:true}), centre={x:1.8,y:0}, yaw=rad(175);
    tracker.importLandmarks([{kind:'couch',X:centre.x,Y:centre.y,yaw:yaw-Math.PI,L:1.82,D:.86,H:.87}],0);
    const det=detections('couch',centre,origin);
    const tick=(input,i,observationPose=origin)=>analyze(input.cells,det,origin,tracker,i*.1,
        {mapped:true,xyz:input.xyz,detectionStamp:5,observationPose}).objects[0];
    const held=frame('couch',false,0,yaw,centre,origin,12,0);
    const before=tick(held,0,null);
    assert.equal(tracker.tracks[0].headingSamples,undefined,'unpaired cloud contributes no yaw');
    for (let i=1;i<10;i++) assert.equal(tick(held,i).yaw,before.yaw);
    assert.equal(tracker.tracks[0].headingSamples.length,1,'late pose enables exactly one vote');
    for (let i=10;i<20;i++) {
        const input=frame('couch',false,3,yaw,centre,origin,i,0);
        assert.equal(tick(input,i).yaw,before.yaw);
    }
    let after;
    tracker.tracks[0].model={old:true};
    for (let i=20;i<38;i++) after=tick(frame('couch',false,0,yaw,centre,origin,i,0),i);
    assert.equal(after.id,before.id);
    assert.ok(Math.abs(deg(delta(after.yaw,yaw,1)))<5);
    assert.equal(tracker.tracks[0].model,null);
}

// A resolved rectangular table can correct a stable error smaller than the
// legacy turn threshold without creating a replacement track.
{
    const tracker=new Tracker({furnitureYawV2:true}), centre={x:1.8,y:0}, yaw=rad(18);
    tracker.importLandmarks([{kind:'table',X:centre.x,Y:centre.y,yaw:yaw-rad(15),L:1.27,D:.74,H:.745}],0);
    const id=tracker.tracks[0].id;
    let table;
    for (let i=0;i<18;i++) {
        const {xyz,cells}=frame('table',false,0,yaw,centre,origin,i+1,0);
        const out=analyze(cells,[],origin,tracker,i*.15,{mapped:true,xyz,observationPose:origin});
        table=out.objects.find(o=>o.kind==='table');
    }
    assert.equal(table.id,id);
    assert.ok(Math.abs(deg(delta(table.yaw,yaw,2)))<5);
}

// A compatible measured wall helps with small quantization error. The same
// prior leaves a tabletop deliberately angled 33 degrees to that wall alone.
{
    const centre={x:2,y:0};
    for (const heading of [.8,33]) {
        const yaw=rad(heading), c=Math.cos(yaw), s=Math.sin(yaw), xyz=[];
        const add=(u,v,z)=>xyz.push(centre.x+c*u-s*v,s*u+c*v,z);
        for (let u=-.65;u<.65;u+=.013) for (let v=-.4;v<.4;v+=.013) add(u,v,.745);
        for (const u of [-.55,.55]) for (const v of [-.32,.32]) for (let z=.1;z<.64;z+=.04) add(u,v,z);
        const a=rad(.8), wx=Math.cos(a), wy=Math.sin(a);
        const wall={x0:2-wx+.59*wy,y0:-wy-.59*wx,x1:2+wx+.59*wy,y1:wy-.59*wx};
        const cells=voxelize(xyz,.025);
        const error=walls=>Math.abs(deg(delta(furnitureSurfaces(cells,walls).find(o=>o.kind==='table').yawFit.yaw,yaw,4)));
        if (heading < 1) assert.ok(error([wall]) < error([]));
        assert.ok(error([wall]) < .1);
    }
}

// Exercise the GUI flag and the production map <- odom <- cloud composition
// with a non-identity map transform and a newer readout pose.
{
    const THREE=require('../../src/web/vendor/three/three.min.js');
    global.window=global;
    global.RealityStructure=require('../../src/web/reality-structure');
    const button={};
    global.document={getElementById:()=>button};
    const storage=new Map();
    global.localStorage={getItem:k=>storage.get(k),setItem:(k,v)=>storage.set(k,v)};
    require('../../src/web/reality-mode');
    function create(search) {
        global.location={search};
        const scene=new THREE.Scene();
        scene.background=new THREE.Color();
        scene.fog=new THREE.Fog(0,1,10);
        return RealityMode.create(THREE,scene,{setClearColor(){}},
            {floor:new THREE.Mesh(),rings:[],tinted:[]});
    }
    const mapYaw=rad(50), mc=Math.cos(mapYaw), ms=Math.sin(mapYaw);
    const toMap=({x,y,theta})=>({x:3+mc*x-ms*y,y:-2+ms*x+mc*y,theta:mapYaw+theta});
    const toCm=({x,y,theta})=>({x:x*100,y:y*100,theta});
    const cloudOdom={x:.4,y:-.3,theta:rad(13)}, readoutOdom={x:.55,y:-.35,theta:rad(39)};
    const capture=toMap(cloudOdom), readout=toMap(readoutOdom);
    const centre={x:capture.x+1.8*Math.cos(capture.theta),y:capture.y+1.8*Math.sin(capture.theta)};
    const yaw=rad(-123);
    function render(view) {
        let st;
        for (let i=0;i<15;i++) {
            const {xyz,cells}=frame('couch',false,0,yaw,centre,capture,i+1,0);
            const det=detections('couch',centre,readout);
            st=view.analyzeStructure(cells,det,{
                robotPose:toCm(readoutOdom),robotPoseMap:toCm(readout),oakDetectionsAt:i
            },'yaw-test',xyz,i*150,{...cloudOdom,yaw:cloudOdom.theta});
        }
        const o=st.objects.find(o=>o.kind==='couch');
        return Math.abs(deg(delta(o.yaw+readout.theta,yaw,1)));
    }
    assert.ok(render(create(''))>15,'flag off by default');
    assert.ok(render(create('?furnitureYawV2=1'))<5,'paired cloud yaw in map frame');
    assert.equal(storage.get('furnitureYawV2'),'1');
    assert.ok(render(create(''))<5,'flag persists after reload');
    assert.ok(render(create('?furnitureYawV2=0'))>15,'URL disables the stored flag');
}
