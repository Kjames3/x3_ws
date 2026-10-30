// GUI-only camera-seeded lidar tracker. No person classification from lidar,
// no navigation output. Coordinates inside the tracker are odometry metres.
(function(root) {
    const MAX_CAMERA_AGE = .65, MAX_WITHOUT_CAMERA = 1.2, MIN_POINTS = 8;
    const world = (x,y,p) => [p.x + Math.cos(p.yaw)*x - Math.sin(p.yaw)*y, p.y + Math.sin(p.yaw)*x + Math.cos(p.yaw)*y];
    const local = (x,y,p) => [Math.cos(p.yaw)*(x-p.x)+Math.sin(p.yaw)*(y-p.y), -Math.sin(p.yaw)*(x-p.x)+Math.cos(p.yaw)*(y-p.y)];
    // XY connectivity allows gaps along the vertical non-repetitive scan pattern.
    // Build whole-scene components before gating: cropping a wall around a person
    // first could turn a piece of that wall into a person-sized cluster.
    function clusters(xyz) {
        const bins = new Map(), cell=.18;
        for(let i=0;i<xyz.length;i+=3) {
            const x=xyz[i],y=xyz[i+1],z=xyz[i+2];
            if(![x,y,z].every(Number.isFinite)||z<.15||z>2.2) continue;
            const ix=Math.floor(x/cell),iy=Math.floor(y/cell),k=`${ix},${iy}`;
            if(!bins.has(k))bins.set(k,{ix,iy,points:[]});
            bins.get(k).points.push(i/3);
        }
        const visited=new Set(), out=[];
        for(const [k,bin] of bins) {
            if(visited.has(k))continue;
            visited.add(k);const queue=[bin], indices=[];
            for(let n=0;n<queue.length;n++) {
                const b=queue[n]; indices.push(...b.points);
                for(let dx=-1;dx<=1;dx++)for(let dy=-1;dy<=1;dy++) {
                    const next=`${b.ix+dx},${b.iy+dy}`;
                    if(bins.has(next)&&!visited.has(next)){visited.add(next);queue.push(bins.get(next));}
                }
            }
            if(indices.length<MIN_POINTS)continue;
            const xs=indices.map(i=>xyz[i*3]).sort((a,b)=>a-b), ys=indices.map(i=>xyz[i*3+1]).sort((a,b)=>a-b);
            const zs=indices.map(i=>xyz[i*3+2]), lo=[xs[0],ys[0],Math.min(...zs)], hi=[xs.at(-1),ys.at(-1),Math.max(...zs)];
            const width=hi[0]-lo[0],depth=hi[1]-lo[1],height=hi[2]-lo[2];
            if(Math.max(width,depth)>.90 || Math.max(width,depth)<.12 || height<.35)continue;
            out.push({x:xs[Math.floor(xs.length/2)],y:ys[Math.floor(ys.length/2)],lo,hi,indices});
        }
        return out;
    }
    class Tracker {
        constructor(){this.clear();}
        clear(){this.tracks=new Map();this.previous=null;this.results=[];this.labels=new Map();}
        update(xyz, pose, camera, packetAge=Infinity) {
            const now=pose?.stamp;
            if(!pose||![pose.x,pose.y,pose.yaw,now].every(Number.isFinite)) {this.clear();return [];}
            if(this.previous) {
                const dt=now-this.previous.stamp;
                if(dt===0)return this.results;
                const da=Math.abs(Math.atan2(Math.sin(pose.yaw-this.previous.yaw),Math.cos(pose.yaw-this.previous.yaw)));
                if(dt<0||dt>1||Math.hypot(pose.x-this.previous.x,pose.y-this.previous.y)>.5+.8*dt||da>.4+2*dt)this.clear();
            }
            this.previous=pose;
            // Repeated telemetry does not refresh a camera observation. The
            // semantic confirmation deadline comes from the source state stamp.
            const anchors=new Map();
            if(packetAge<.5)for(const c of camera||[]) {
                const dt=now-c.state_stamp_s;
                if(![c.odom_x,c.odom_y,c.odom_vx,c.odom_vy,c.state_stamp_s,c.obs_age_s].every(Number.isFinite)||dt<-.15||dt>MAX_CAMERA_AGE)continue;
                const lastSeen=c.state_stamp_s-c.obs_age_s;
                if(c.measured===false || now-lastSeen>MAX_CAMERA_AGE)continue;
                const x=c.odom_x+c.odom_vx*Math.max(0,dt),y=c.odom_y+c.odom_vy*Math.max(0,dt);
                anchors.set(String(c.id),{x,y,vx:c.odom_vx,vy:c.odom_vy,lastSeen});
            }
            for(const [id,a]of anchors) {
                if(!this.tracks.has(id))this.tracks.set(id,{id,x:a.x,y:a.y,vx:a.vx,vy:a.vy,at:now,lastCamera:a.lastSeen,lastLidar:-Infinity,hits:0,sigma:.3});
                const t=this.tracks.get(id);
                // A discontinuous camera ID/position must not drag an existing
                // track through the room or transfer its lidar confirmation.
                if(Math.hypot(a.x-t.x,a.y-t.y)>.8){t.x=a.x;t.y=a.y;t.vx=a.vx;t.vy=a.vy;t.hits=0;t.lastLidar=-Infinity;}
                t.lastCamera=Math.max(t.lastCamera,a.lastSeen);
            }
            if(!this.tracks.size){this.labels=new Map();this.results=[];return [];}
            const candidates=clusters(xyz).map(c=>{const [x,y]=world(c.x,c.y,pose);return {...c,wx:x,wy:y};});
            const predicted=[];
            for(const [id,t]of this.tracks) {
                if(now-t.lastCamera>MAX_WITHOUT_CAMERA){this.tracks.delete(id);continue;}
                const dt=Math.min(.4,Math.max(0,now-t.at));
                const a=anchors.get(id);
                let x=t.x+t.vx*dt,y=t.y+t.vy*dt;
                if(t.hits<2&&a){x=a.x;y=a.y;}
                predicted.push({id,t,x,y,dt,a});
            }
            const edges=[];
            for(let i=0;i<predicted.length;i++)for(let j=0;j<candidates.length;j++) {
                const p=predicted[i],c=candidates[j],distance=Math.hypot(c.wx-p.x,c.wy-p.y);
                if(distance<.55)edges.push({i,j,distance});
            }
            // Require a distinct nearest match in BOTH directions. Crossings
            // and two people merged into one cluster remain unassigned.
            const matches=new Map();
            for(let i=0;i<predicted.length;i++) {
                const options=edges.filter(e=>e.i===i).sort((a,b)=>a.distance-b.distance);
                if(!options.length || options[1]&&options[1].distance-options[0].distance<.15)continue;
                const best=options[0], rivals=edges.filter(e=>e.j===best.j).sort((a,b)=>a.distance-b.distance);
                const component=candidates[best.j];
                const inside=predicted.filter(p=>{const [x,y]=local(p.x,p.y,pose);return x>=component.lo[0]-.2&&x<=component.hi[0]+.2&&y>=component.lo[1]-.2&&y<=component.hi[1]+.2;});
                if(inside.length>1)continue;
                if(rivals[0].i!==i || rivals[1]&&rivals[1].distance-rivals[0].distance<.15)continue;
                matches.set(i,best.j);
            }
            this.labels=new Map();const results=[];
            for(let i=0;i<predicted.length;i++) {
                const p=predicted[i],t=p.t,c=candidates[matches.get(i)];
                let mode='camera only';
                if(c) {
                    const rx=c.wx-p.x,ry=c.wy-p.y;
                    t.x=p.x+.6*rx;t.y=p.y+.6*ry;
                    if(p.dt>.05&&t.hits) {
                        t.vx=Math.max(-2,Math.min(2,t.vx+.12*rx/p.dt));
                        t.vy=Math.max(-2,Math.min(2,t.vy+.12*ry/p.dt));
                    }
                    t.hits++;t.lastLidar=now;t.sigma=Math.max(.12,t.sigma*.8);
                    mode=t.hits>=3?(now-t.lastCamera>.4?'lidar only':'camera + lidar'):'lidar tentative';
                    for(const idx of c.indices)this.labels.set(idx,t.id);
                } else {
                    t.x=p.a?p.a.x:p.x;t.y=p.a?p.a.y:p.y;t.sigma=Math.min(.8,t.sigma+.12);
                    if(p.a){t.vx=p.a.vx;t.vy=p.a.vy;}
                    if(now-t.lastLidar>.4)t.hits=0;
                    // No lidar hit means no camera-dropout extension. Keep only
                    // a short, explicitly uncertain prediction of confirmed tracks.
                    if(!p.a && (t.hits<3 || now-t.lastLidar>.4)) {this.tracks.delete(t.id);continue;}
                    mode=p.a?'camera only':'uncertain';
                }
                t.at=now;
                const [x,y]=local(t.x,t.y,pose);
                const dropout=Math.max(0,now-t.lastCamera);
                const sigma=Math.max(t.sigma,.12+dropout*.3);
                results.push({id:t.id,x,y,sigma,mode,points:c?.indices.length||0,age:now-t.lastLidar,
                    cameraAge:dropout,bounds:c?{lo:c.lo,hi:c.hi}:null});
            }
            this.results=results;return results;
        }
    }
    const api={Tracker,clusters,MAX_WITHOUT_CAMERA};
    if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.PersonCloudTracker=api;
})(typeof globalThis!=='undefined'?globalThis:this);
