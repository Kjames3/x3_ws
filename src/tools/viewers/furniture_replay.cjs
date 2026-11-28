// Replay exported walk frames (export_surroundings_frames.py) through the GUI furniture
// tracker the way scene-view.js feeds it (5 cm memory, unmapped).
// Usage: node src/tools/viewers/furniture_replay.cjs FRAMES.ndjson [REPO=.] [OUT=stdout]
// YAWV2=0 replays with ?furnitureYawV2 off. Output: per frame, objects in the odom frame.
const fs=require('fs'), path=require('path');
const repo=process.argv[3]||'.';
const SM=require(path.resolve(repo,'src/web/surroundings-memory.js'));
const RS=require(path.resolve(repo,'src/web/reality-structure.js'));
const lines=fs.readFileSync(process.argv[2],'utf8').trim().split('\n');
const mem=new SM.Memory(.05), tracker=new RS.Tracker({furnitureYawV2:process.env.YAWV2!=='0'});
const deg=r=>Math.round(r*180/Math.PI);
const log=[]; let lastDetT=-1e9, lastDets=[];
for (const line of lines) {
  const f=JSON.parse(line); const cp={...f.pose};
  const xyz=new Float32Array(f.xyz);
  mem.ingest(xyz,cp,f.movers||[]);
  const cells=mem.snapshot(cp).cells;
  const rp=f.robot_pose; const pose={x:rp.x/100,y:rp.y/100,theta:rp.theta};
  const obs={x:cp.x,y:cp.y,theta:cp.yaw};
  const st=RS.analyze(cells,f.dets,pose,tracker,f.t,{mapped:false,xyz,detectionStamp:f.det_t,observationPose:obs,wallPose:cp});
  // world (odom) frame objects
  const c=Math.cos(pose.theta), s=Math.sin(pose.theta);
  log.push({t:f.t, robot:[+pose.x.toFixed(2),+pose.y.toFixed(2),deg(pose.theta)], objs:st.objects.map(o=>({id:o.id,k:o.kind,
     X:+(pose.x+c*o.x-s*o.y).toFixed(2),Y:+(pose.y+s*o.x+c*o.y).toFixed(2),yaw:deg(o.yaw+pose.theta),
     L:+o.L.toFixed(2),D:+o.D.toFixed(2),H:+(o.H||0).toFixed(2),round:o.round?1:0,inf:o.inferred?1:0}))});
}
fs.writeFileSync(process.argv[4]||'/dev/stdout',log.map(x=>JSON.stringify(x)).join('\n')+'\n');
