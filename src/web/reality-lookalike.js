// Coloured OAK points arrive in odom. The caller supplies the object's box
// in that frame and a base pose for view direction and person masking.
(function (root) {
    'use strict';
    const STEP = .025, CAP = 4000, MIN_VOXELS = 64;
    const keyOf = (x,y,z) => ((x+128)*256+y+128)*256+z;
    function create() {
        return {voxels:new Map(), pending:new Map(), sectors:0, revision:0, frame:0, clouds:new WeakSet()};
    }

    function ingest(model, cloud, box, robot, people) {
        if (model.clouds.has(cloud) || (cloud.stamp != null && cloud.stamp === model.stamp)) return;
        model.clouds.add(cloud);
        model.stamp = cloud.stamp;
        const frame = ++model.frame;
        // Keep first sightings briefly. They consume the same budget as
        // confirmed voxels and never enter the renderer or session data.
        for (const [key,v] of model.pending) {
            if (frame-v.frame > 4) model.pending.delete(key);
        }
        const c=Math.cos(box.yaw), s=Math.sin(box.yaw), bottom=box.bottom || 0;
        let supported=0;
        for (let i=0; i<cloud.length; i+=3) {
            const z=cloud[i+2]-bottom;
            if (!(z>=.02 && z<=box.H+.03)) continue;
            const dx=cloud[i]-box.x, dy=cloud[i+1]-box.y;
            const along=c*dx+s*dy, left=c*dy-s*dx;
            if (!(Math.abs(along)<=box.L/2+.03 && Math.abs(left)<=box.D/2+.03)) continue;
            if (people.some(p=>(cloud[i]-p.x)**2+(cloud[i+1]-p.y)**2 < .5**2)) continue;
            const x=Math.round(along/STEP), y=Math.round(left/STEP), h=Math.round(z/STEP);
            if (x < -128 || x>127 || y < -128 || y>127 || h>255) continue;
            const key=keyOf(x,y,h);
            let v=model.voxels.get(key) || model.pending.get(key);
            if (v && v.frame===frame) continue;
            const r=cloud.colors[i]*255, g=cloud.colors[i+1]*255, b=cloud.colors[i+2]*255;
            if (!v) {
                if (model.pending.size+model.voxels.size>=CAP) continue;
                model.pending.set(key,{x,y,z:h,r,g,b,hits:1,frame});
                continue;
            }
            if (v.hits===1) {
                model.pending.delete(key);
                model.voxels.set(key,v);
            }
            v.hits=Math.min(255,v.hits+1);
            v.r+=(r-v.r)/v.hits; v.g+=(g-v.g)/v.hits; v.b+=(b-v.b)/v.hits;
            v.frame=frame;
            supported++;
        }
        if (supported) {
            if (supported>=8) {
                const angle=Math.atan2(robot.y-box.y,robot.x-box.x)-box.yaw;
                model.sectors |= 1 << (Math.round(angle/(Math.PI/4)) & 7);
            }
            model.revision++;
        }
    }
    function ready(m) {
        return m && m.voxels.size>=MIN_VOXELS && (m.sectors & (m.sectors-1))!==0;
    }
    // v1: version, sector mask, then 7 bytes per confirmed voxel.
    function pack(m, limit=CAP) {
        if (!m || !m.voxels.size) return undefined;
        const bytes=[1,m.sectors];
        for (const v of m.voxels.values()) {
            bytes.push(v.x+128,v.y+128,v.z,Math.round(v.r),Math.round(v.g),Math.round(v.b),v.hits);
            if ((bytes.length-2)/7>=limit) break;
        }
        return btoa(String.fromCharCode(...bytes));
    }
    function unpack(encoded) {
        if (typeof encoded!=='string' || encoded.length>37336) return;
        let raw;
        try { raw=atob(encoded); } catch { return; }
        if (raw.length<9 || (raw.length-2)%7 || raw.charCodeAt(0)!==1) return;
        const m=create();
        m.sectors=raw.charCodeAt(1);
        for (let i=2;i<raw.length;i+=7) {
            const [a,b,z,r,g,blue,hits]=Array.from(raw.slice(i,i+7),ch=>ch.charCodeAt(0));
            if (hits<2) return;
            const x=a-128,y=b-128;
            m.voxels.set(keyOf(x,y,z),{x,y,z,r,g,b:blue,hits,frame:0});
        }
        return m;
    }
    // Geometric candidates in the landmark frame. Callers need a map and
    // path validation before treating any candidate as reachable.
    function viewpoints(object, sectors=object.model?.sectors || 0) {
        const poses=[];
        for (const sector of [0,3,6,1,4,7,2,5]) {
            if (sectors & (1<<sector)) continue;
            const local=sector*Math.PI/4;
            const edge=Math.min(object.L/2/Math.abs(Math.cos(local)),object.D/2/Math.abs(Math.sin(local)));
            const range=Math.max(1.25,edge+.12);
            if (range>1.5) continue;
            const angle=object.yaw+local;
            poses.push({x:object.X+range*Math.cos(angle),y:object.Y+range*Math.sin(angle),
                        yaw:Math.atan2(-Math.sin(angle),-Math.cos(angle))});
            if (poses.length===3) break;
        }
        return poses;
    }
    const api={create,ingest,ready,pack,unpack,viewpoints,STEP,CAP};
    if (typeof module!=='undefined' && module.exports) module.exports=api;
    else root.RealityLookalike=api;
})(typeof window!=='undefined'?window:globalThis);
