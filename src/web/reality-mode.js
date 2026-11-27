// =================================================================
// Reality mode for the Surroundings view: a dark, FSD-style restyle of
// the same scene plus two intent overlays.
//   theme : dark background/fog/floor, dim range rings, walls and voxels
//           tinted cool blue (instance colours are multiplied, so height
//           colouring still reads).
//   path  : the Nav2 global plan (state.nav.path, map frame, metres) moved
//           into the robot frame through robotPoseMap (cm, rad) and drawn as
//           a glowing ribbon with chevrons flowing toward the goal, plus a
//           light pillar at the goal.
//   shield: a translucent shell around the robot coloured from readout.cbf:
//           cyan = clear, amber pulse = CBF limiting the command,
//           red = blind (stale obstacles), grey = 3D-sweep hold.
//   intent: per moving person, the predicted path for TRAIL_S as a floor
//           ribbon that widens with uncertainty and fades, with a ring at the
//           predicted spot. Constant velocity for now; an entry carrying
//           `path` (scene [x, z] points, one per TRAIL_S/(N-1) s) is drawn
//           as given, so GRU predictions can drop in. Amber normally, red
//           when the predicted path passes within CONFLICT_M of the robot.
//   struct: RealityStructure (reality-structure.js) output: fitted wall
//           segments as flat panels with a lit top edge, and furniture
//           stand-ins (low-poly, built here, scaled to the lidar box).
//   people: soft halo under every person; avatars restyled to mannequins
//           through X3Humanoid.setStyle (shared materials).
// Display only: nothing here feeds control.
// Scene frame (see scene-view.js): forward = -Z, right = +X, up = +Y.
// =================================================================
window.RealityMode = (() => {
    const MAX_PATH = 512;
    const RIBBON_W = 0.30;        // ~ robot width
    const RIBBON_Y = 0.012;
    const SHIELD_R = 0.36, SHIELD_H = 0.32;
    const COLORS = { ok: 0x22d3ee, limited: 0xf59e0b, blind: 0xef4444, hold: 0x9ca3af };
    const TRAIL_S = 2.0, TRAIL_N = 16, MAX_TRAILS = 12;
    const TRAIL_W0 = 0.22, TRAIL_W1 = 0.55;       // width now / at TRAIL_S (m)
    const MOVING_MPS = 0.15, CONFLICT_M = 0.6;
    const TRAIL_COLORS = { normal: 0xfbbf24, conflict: 0xef4444 };
    const DARK = { bg: 0x0b0f14, floor: 0x111820, ring: 0x1e2a36,
                   tint: 0x8fb4d9, emissive: 0x0a1622 };

    function create(THREE, scene, renderer, refs) {
        const time = { value: 0 };

        // ---- path ribbon -------------------------------------------------
        const pos = new Float32Array(MAX_PATH * 2 * 3);
        const along = new Float32Array(MAX_PATH * 2);   // metres from robot
        const side = new Float32Array(MAX_PATH * 2);    // -1 / +1 across
        const idx = [];
        for (let i = 0; i < MAX_PATH - 1; i++) {
            const a = 2 * i;
            idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
        }
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
        geom.setAttribute('along', new THREE.BufferAttribute(along, 1));
        geom.setAttribute('side', new THREE.BufferAttribute(side, 1));
        geom.setIndex(idx);
        geom.setDrawRange(0, 0);
        const ribbon = new THREE.Mesh(geom, new THREE.ShaderMaterial({
            uniforms: { time, total: { value: 1 }, color: { value: new THREE.Color(0x38bdf8) } },
            vertexShader: `
                attribute float along; attribute float side;
                varying float vAlong; varying float vSide;
                void main() { vAlong = along; vSide = side;
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
            fragmentShader: `
                uniform float time; uniform float total; uniform vec3 color;
                varying float vAlong; varying float vSide;
                void main() {
                    float edge = 1.0 - smoothstep(0.55, 1.0, abs(vSide));
                    float rim = smoothstep(0.75, 0.95, abs(vSide)) * (1.0 - smoothstep(0.95, 1.0, abs(vSide)));
                    float chevron = fract((vAlong + abs(vSide) * 0.12) * 2.5 - time * 1.2);
                    chevron = smoothstep(0.0, 0.15, chevron) * (1.0 - smoothstep(0.15, 0.35, chevron));
                    float fadeIn = smoothstep(0.0, 0.35, vAlong);
                    float fadeOut = 1.0 - smoothstep(total - 0.4, total, vAlong) * 0.6;
                    float far = 1.0 - smoothstep(4.0, 8.0, vAlong);
                    float a = (0.22 * edge + 0.55 * rim + 0.45 * chevron * edge) * fadeIn * fadeOut * far;
                    gl_FragColor = vec4(color * (1.0 + chevron), a);
                }`,
            transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide,
        }));
        ribbon.frustumCulled = false;
        ribbon.visible = false;
        scene.add(ribbon);

        const beacon = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 1.6, 24, 1, true),
            new THREE.ShaderMaterial({
                uniforms: { time, color: { value: new THREE.Color(0x38bdf8) } },
                vertexShader: `varying float vY; void main(){ vY = uv.y;
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`,
                fragmentShader: `uniform float time; uniform vec3 color; varying float vY;
                    void main(){ float a = (1.0 - vY) * (0.45 + 0.15 * sin(time * 3.0));
                    gl_FragColor = vec4(color, a); }`,
                transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide,
            }));
        beacon.visible = false;
        scene.add(beacon);

        // ---- CBF shield ----------------------------------------------------
        const shieldColor = { value: new THREE.Color(COLORS.ok) };
        const shieldPulse = { value: 0 };
        const shield = new THREE.Mesh(new THREE.CylinderGeometry(SHIELD_R, SHIELD_R, SHIELD_H, 48, 1, true),
            new THREE.ShaderMaterial({
                uniforms: { time, color: shieldColor, pulse: shieldPulse },
                vertexShader: `varying vec2 vUv; void main(){ vUv = uv;
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`,
                fragmentShader: `uniform float time; uniform vec3 color; uniform float pulse; varying vec2 vUv;
                    void main(){
                        float base = pow(1.0 - vUv.y, 2.2);
                        float scan = smoothstep(0.96, 1.0, fract(vUv.y * 1.0 - time * 0.6)) * 0.4;
                        float beat = 1.0 + pulse * (0.8 + 0.8 * sin(time * 9.0));
                        gl_FragColor = vec4(color, (0.32 * base + scan * (1.0 - vUv.y)) * beat);
                    }`,
                transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide,
            }));
        shield.position.y = SHIELD_H / 2;
        const shieldRingMat = new THREE.MeshBasicMaterial({ color: COLORS.ok, transparent: true, opacity: 0.7,
            depthWrite: false, blending: THREE.AdditiveBlending });
        const shieldRing = new THREE.Mesh(new THREE.RingGeometry(SHIELD_R - 0.012, SHIELD_R + 0.012, 64), shieldRingMat);
        shieldRing.rotation.x = -Math.PI / 2;
        shieldRing.position.y = 0.006;
        const shieldGroup = new THREE.Group();
        shieldGroup.add(shield, shieldRing);
        shieldGroup.visible = false;
        scene.add(shieldGroup);

        // ---- intent trails + person halos ----------------------------------
        const trailIdx = [];
        for (let i = 0; i < TRAIL_N - 1; i++) { const a = 2 * i; trailIdx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2); }
        const trailVert = `
            attribute float u; attribute float side; varying float vU; varying float vSide;
            void main(){ vU = u; vSide = side;
                gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`;
        const trailFrag = `
            uniform float time; uniform vec3 color; varying float vU; varying float vSide;
            void main(){
                float edge = 1.0 - smoothstep(0.35, 1.0, abs(vSide));
                float dash = smoothstep(0.0, 0.2, fract(vU * 6.0 - time * 1.5)) * 0.5 + 0.5;
                float a = pow(1.0 - vU, 1.3) * edge * dash * 0.8;
                gl_FragColor = vec4(color, a);
            }`;
        const markerFrag = `
            uniform vec3 color; uniform float strength; varying vec2 vUv;
            void main(){ float r = length(vUv - 0.5) * 2.0;
                float ring = smoothstep(0.70, 0.86, r) * (1.0 - smoothstep(0.86, 1.0, r));
                float fill = (1.0 - smoothstep(0.0, 1.0, r)) * 0.35;
                gl_FragColor = vec4(color, (ring + fill) * strength); }`;
        const uvVert = `varying vec2 vUv; void main(){ vUv = uv;
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`;
        function marker(color, strength, size) {
            const m = new THREE.Mesh(new THREE.PlaneGeometry(size, size), new THREE.ShaderMaterial({
                uniforms: { color: { value: new THREE.Color(color) }, strength: { value: strength } },
                vertexShader: uvVert, fragmentShader: markerFrag,
                transparent: true, depthWrite: false, blending: THREE.AdditiveBlending }));
            m.rotation.x = -Math.PI / 2; m.visible = false; m.frustumCulled = false;
            scene.add(m);
            return m;
        }
        const trails = [];
        for (let k = 0; k < MAX_TRAILS; k++) {
            const g = new THREE.BufferGeometry();
            g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_N * 6), 3));
            const u = new Float32Array(TRAIL_N * 2), sd = new Float32Array(TRAIL_N * 2);
            for (let i = 0; i < TRAIL_N; i++) { u[2*i] = u[2*i+1] = i / (TRAIL_N - 1); sd[2*i] = 1; sd[2*i+1] = -1; }
            g.setAttribute('u', new THREE.BufferAttribute(u, 1));
            g.setAttribute('side', new THREE.BufferAttribute(sd, 1));
            g.setIndex(trailIdx);
            const color = { value: new THREE.Color(TRAIL_COLORS.normal) };
            const mesh = new THREE.Mesh(g, new THREE.ShaderMaterial({
                uniforms: { time, color }, vertexShader: trailVert, fragmentShader: trailFrag,
                transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide }));
            mesh.frustumCulled = false; mesh.visible = false;
            scene.add(mesh);
            trails.push({ mesh, color, end: marker(TRAIL_COLORS.normal, 0.9, 0.42),
                          halo: marker(0x67e8f9, 0.55, 0.9) });
        }
        function hideTrails(from) {
            for (let k = from; k < MAX_TRAILS; k++) trails[k].mesh.visible = trails[k].end.visible = trails[k].halo.visible = false;
        }
        function updateIntent(persons) {
            let k = 0;
            for (const p of persons || []) {
                if (k >= MAX_TRAILS) break;
                const t = trails[k++];
                t.halo.position.set(p.x, 0.008, p.z); t.halo.visible = true;
                const speed = Math.hypot(p.sx, p.sz);
                const path = p.path || (speed > MOVING_MPS ? Array.from({ length: TRAIL_N }, (_, i) => {
                    const dt = TRAIL_S * i / (TRAIL_N - 1);
                    return [p.x + p.sx * dt, p.z + p.sz * dt];
                }) : null);
                if (!path || path.length !== TRAIL_N) { t.mesh.visible = t.end.visible = false; continue; }
                const pos = t.mesh.geometry.attributes.position.array;
                let minD = Infinity;
                for (let i = 0; i < TRAIL_N; i++) {
                    const a = path[Math.max(0, i - 1)], b = path[Math.min(TRAIL_N - 1, i + 1)];
                    let tx = b[0] - a[0], tz = b[1] - a[1];
                    const tl = Math.hypot(tx, tz) || 1; tx /= tl; tz /= tl;
                    const w = (TRAIL_W0 + (TRAIL_W1 - TRAIL_W0) * i / (TRAIL_N - 1)) / 2;
                    const o = 6 * i;
                    pos[o] = path[i][0] - tz * w; pos[o+1] = 0.014; pos[o+2] = path[i][1] + tx * w;
                    pos[o+3] = path[i][0] + tz * w; pos[o+4] = 0.014; pos[o+5] = path[i][1] - tx * w;
                    minD = Math.min(minD, Math.hypot(path[i][0], path[i][1]));
                }
                t.mesh.geometry.attributes.position.needsUpdate = true;
                const hex = minD < CONFLICT_M ? TRAIL_COLORS.conflict : TRAIL_COLORS.normal;
                t.color.value.setHex(hex);
                t.end.material.uniforms.color.value.setHex(hex);
                const e = path[TRAIL_N - 1];
                t.end.position.set(e[0], 0.016, e[1]);
                t.mesh.visible = t.end.visible = true;
            }
            hideTrails(k);
        }

        const unitBox = new THREE.BoxGeometry(1, 1, 1);

        // ---- fitted wall panels + furniture stand-ins ------------------------
        // Base (x fwd, y left, z up) -> scene (x = -y, y = z, z = -x).
        const panelVert = `varying vec2 vUv; void main(){ vUv = uv;
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`;
        const panelFrag = `uniform float h; varying vec2 vUv;
            void main(){
                float y = vUv.y * h;
                vec3 body = mix(vec3(0.07,0.10,0.14), vec3(0.16,0.22,0.30), vUv.y);
                float edge = smoothstep(h - 0.05, h - 0.01, y);
                float base = 1.0 - smoothstep(0.0, 0.04, y);
                vec3 col = mix(body, vec3(0.45,0.85,1.0), edge) + vec3(0.1,0.25,0.35) * base;
                gl_FragColor = vec4(col, mix(0.85, 0.45, vUv.y) + 0.4 * edge);
            }`;
        const panels = [];
        function panel(i) {
            if (panels[i]) return panels[i];
            const m = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.ShaderMaterial({
                uniforms: { h: { value: 1 } }, vertexShader: panelVert, fragmentShader: panelFrag,
                transparent: true, side: THREE.DoubleSide }));
            m.frustumCulled = false;
            scene.add(m);
            return (panels[i] = m);
        }
        const furnMat = new THREE.MeshLambertMaterial({ color: 0xc4cfdb, emissive: 0x141c26, transparent: true });
        const furnEdgeMat = new THREE.MeshBasicMaterial({ color: 0x67d4f5, transparent: true, opacity: 0.6,
            depthWrite: false, blending: THREE.AdditiveBlending });
        // Parts in a unit box: x = length, y = height, z = depth, origin at floor centre.
        const PARTS = {
            chair: [[0, .46, .05, .9, .06, .9], [0, .75, -.42, .9, .5, .08],
                    [-.4, .22, -.4, .07, .45, .07], [.4, .22, -.4, .07, .45, .07], [-.4, .22, .4, .07, .45, .07], [.4, .22, .4, .07, .45, .07]],
            couch: [[0, .25, .08, 1, .3, .84], [0, .62, -.42, 1, .55, .16], [-.46, .45, .08, .08, .3, .84], [.46, .45, .08, .08, .3, .84]],
            table: [[0, .96, 0, 1, .06, 1], [-.45, .47, -.42, .05, .94, .06], [.45, .47, -.42, .05, .94, .06],
                    [-.45, .47, .42, .05, .94, .06], [.45, .47, .42, .05, .94, .06]],
            tv:    [[0, .62, 0, 1, .72, .12], [0, .13, 0, .08, .26, .08], [0, .02, 0, .4, .04, .6]],
            bed:   [[0, .3, 0, 1, .55, 1], [-.42, .62, 0, .12, .12, .8], [.48, .55, 0, .04, .9, 1]],
            plant: [[0, .2, 0, .5, .4, .5], [0, .7, 0, .95, .6, .95]],
            cabinet: [[0,.5,0,1,1,.94],[-.25,.5,.48,.48,.96,.04],[.25,.5,.48,.48,.96,.04]],
            box:   [[0, .5, 0, 1, 1, 1]],
        };
        const unit = new THREE.BoxGeometry(1, 1, 1);
        const objects = new Map();   // id -> {group}
        const splatMaterial = new THREE.PointsMaterial({size:.037, color:0xcbd5df, vertexColors:true});
        function removeObject(ob) {
            scene.remove(ob.group);
            ob.ring.geometry.dispose();
            if (ob.splats) ob.splats.geometry.dispose();
        }
        function updateModel(ob, model) {
            const show = !!RealityStructure.Lookalike.ready(model);
            for (const part of ob.inner.children) part.visible = part === ob.ring || !show;
            if (ob.splats) ob.splats.visible = show;
            if (!show || (ob.model === model && ob.revision === model.revision)) return;
            const {STEP, CAP} = RealityStructure.Lookalike;
            if (!ob.splats) {
                const geom = new THREE.BufferGeometry();
                for (const name of ['position','color'])
                    geom.setAttribute(name,new THREE.BufferAttribute(new Float32Array(3*CAP),3));
                ob.splats = new THREE.Points(geom,splatMaterial);
                ob.splats.frustumCulled = false;
                ob.group.add(ob.splats);
            }
            const geom = ob.splats.geometry;
            let i=0;
            for (const v of model.voxels.values()) {
                geom.attributes.position.setXYZ(i,v.x*STEP,v.z*STEP,-v.y*STEP);
                geom.attributes.color.setXYZ(i,v.r/255,v.g/255,v.b/255);
                i++;
            }
            geom.setDrawRange(0,i);
            for (const attr of Object.values(geom.attributes)) {
                attr.updateRange.count = 3*i;
                attr.needsUpdate = true;
            }
            ob.model = model;
            ob.revision = model.revision;
        }
        const roundTop = new THREE.CylinderGeometry(.5, .5, 1, 32);
        function makeObject(kind) {
            const g = new THREE.Group(), inner = new THREE.Group();
            if (kind === 'table_round') {
                // Round top (unit diameter) on a pedestal and foot, scaled to L x H x D.
                for (const [y, r, h] of [[.97, 1, .06], [.47, .12, .94], [.02, .5, .04]]) {
                    const m = new THREE.Mesh(roundTop, furnMat);
                    m.position.set(0, y, 0); m.scale.set(r, h, r);
                    inner.add(m);
                }
            }
            for (const [x, y, z, sx, sy, sz] of kind === 'table_round' ? [] : PARTS[kind] || PARTS.box) {
                const m = new THREE.Mesh(unit, furnMat);
                m.position.set(x, y, z); m.scale.set(sx, sy, sz);
                inner.add(m);
            }
            const ring = new THREE.Mesh(new THREE.PlaneGeometry(1.08, 1.08), furnEdgeMat);
            ring.rotation.x = -Math.PI / 2; ring.position.y = 0.004; ring.scale.set(1, 1, 1);
            inner.add(ring);
            g.add(inner); scene.add(g);
            return { group: g, inner, ring };
        }
        const doorFrames = [];
        const doorMaterial = new THREE.MeshBasicMaterial({color:0xffce86});
        function updateDoorFrames(gaps) {
            gaps.forEach((g,i) => {
                let frame = doorFrames[i];
                if (!frame) {
                    frame = new THREE.Group();
                    for (let part=0;part<3;part++) frame.add(new THREE.Mesh(unitBox,doorMaterial));
                    scene.add(frame);
                    doorFrames.push(frame);
                }
                const width = Math.hypot(g.x1-g.x0,g.y1-g.y0);
                frame.position.set(-(g.y0+g.y1)/2,0,-(g.x0+g.x1)/2);
                frame.rotation.y = Math.atan2(g.x1-g.x0,g.y0-g.y1);
                const [left,right,header] = frame.children;
                for (const post of [left,right]) post.scale.set(.028,g.top,.028);
                left.position.set(-width/2,g.top/2,0);
                right.position.set(width/2,g.top/2,0);
                header.scale.set(width+.028,.028,.028);
                header.position.set(0,g.top,0);
                frame.visible = true;
            });
            for (let i=gaps.length;i<doorFrames.length;i++) doorFrames[i].visible=false;
        }
        function updateStructure(st) {
            const walls = st ? st.walls : [];
            updateDoorFrames(st ? st.doorways || [] : []);
            walls.forEach((w, i) => {
                const m = panel(i);
                const x0 = -w.y0, z0 = -w.x0, x1 = -w.y1, z1 = -w.x1, len = Math.hypot(x1 - x0, z1 - z0);
                const h = w.top;
                m.scale.set(len, h, 1);
                m.position.set((x0 + x1) / 2, h / 2, (z0 + z1) / 2);
                m.rotation.set(0, Math.atan2(-(z1 - z0), x1 - x0), 0);
                m.material.uniforms.h.value = h;
                m.visible = true;
            });
            for (let i = walls.length; i < panels.length; i++) panels[i].visible = false;
            const seen = new Set();
            for (const o of st ? st.objects : []) {
                seen.add(o.id);
                let ob = objects.get(o.id);
                const shape = o.kind === 'table' && o.round ? 'table_round' : o.kind;
                if (!ob || ob.kind !== shape) {
                    if (ob) removeObject(ob);
                    ob = makeObject(shape); ob.kind = shape; objects.set(o.id, ob);
                }
                ob.group.position.set(-o.y, o.bottom || 0, -o.x);
                ob.group.rotation.set(0, o.yaw + Math.PI / 2, 0);
                ob.inner.scale.set(o.L, o.H, o.D);
                if (o.kind === 'chair') {
                    const seat = Number.isFinite(o.seat) ? Math.max(.30,Math.min(.72,o.seat/o.H)) : .46;
                    const [pad,back,...rest] = ob.inner.children;
                    pad.position.y = seat;
                    back.position.y = (1+seat)/2;
                    back.scale.y = 1-seat;
                    for (const leg of rest.slice(0,4)) {
                        leg.position.y = seat/2;
                        leg.scale.y = seat;
                    }
                }
                updateModel(ob,o.model);
                ob.group.visible = true;
            }
            for (const [id, ob] of objects) if (!seen.has(id)) { removeObject(ob); objects.delete(id); }
        }
        const cabinets = new Map();
        try {
            for (const entry of JSON.parse(localStorage.getItem('x3-reality-cabinets') || '[]')) cabinets.set(...entry);
        } catch { /* Browser storage can be disabled. */ }
        const cabinetButton = document.getElementById('reality-cabinet');
        cabinetButton.onclick = () => {
            if (!roomKey) return;
            const input = window.prompt('Cabinet in map coordinates: center X, center Y, heading (degrees), width, depth, height (metres). Leave empty to remove.');
            if (input === null) return;
            if (input.trim()) {
                const fields = input.split(',').map(v => v.trim() ? Number(v) : NaN);
                const [x,y,heading,L,D,H] = fields;
                if (fields.length !== 6 || !fields.every(Number.isFinite) || Math.min(L,D,H) <= 0) {
                    window.alert('Enter six numbers with positive dimensions.'); return;
                }
                cabinets.set(roomKey, {x,y,yaw:heading*Math.PI/180,L,D,H});
            } else cabinets.delete(roomKey);
            try { localStorage.setItem('x3-reality-cabinets', JSON.stringify([...cabinets], (key,value) => key==='model' ? undefined : value)); } catch { /* Session only. */ }
        };
        let roomKey;
        // Session furniture memory: the server keeps confirmed landmarks per map
        // until it stops, so a refresh or a second browser gets the same room.
        const LANDMARK_PUT_MS = 2000;
        let landmarksPutAt = -1e9, lastPose = null;
        if (window.addEventListener) window.addEventListener('x3-reality-landmarks', ev => {
            const msg = ev.detail || {};
            if (structTracker && roomKey && msg.key === 'map:' + roomKey) {
                structTracker.importLandmarks(msg.objects, performance.now() / 1000);
                const cabinet = cabinets.get(roomKey);
                const saved = msg.objects?.find(o => o.kind === 'cabinet' && cabinet &&
                    o.X===cabinet.x && o.Y===cabinet.y && ['yaw','L','D','H'].every(k=>o[k]===cabinet[k]));
                if (saved) cabinet.model = RealityStructure.Lookalike.unpack(saved.lookalike);
            }
        });
        // Measured furniture in map coordinates (cm, deg) for tape-measure checks.
        const measureButton = document.getElementById('reality-measurements');
        if (measureButton && measureButton !== cabinetButton) measureButton.onclick = () => {
            const rows = structTracker && roomKey ? structTracker.exportLandmarks().map(o => ({
                kind: o.kind, x_cm: Math.round(o.X * 100), y_cm: Math.round(o.Y * 100),
                heading_deg: Math.round(o.yaw * 180 / Math.PI),
                length_cm: Math.round(o.L * 100), depth_cm: Math.round(o.D * 100), height_cm: Math.round(o.H * 100),
                seat_cm: Number.isFinite(o.seat) ? Math.round(o.seat * 100) : null,
                range_cm: lastPose ? Math.round(Math.hypot(o.X - lastPose.x, o.Y - lastPose.y) * 100) : null,
            })) : [];
            const text = JSON.stringify({map: roomKey || null, objects: rows}, null, 1);
            console.log('[reality] furniture measurements', text);
            // navigator.clipboard needs https; the GUI is served over http, so
            // fall back to a selected textarea + execCommand, and report honestly.
            let copied = false;
            try {
                const ta = document.createElement('textarea');
                ta.value = text; ta.style.position = 'fixed'; ta.style.opacity = '0';
                document.body.appendChild(ta); ta.select();
                copied = document.execCommand('copy');
                ta.remove();
            } catch { copied = false; }
            if (!copied && navigator.clipboard && window.isSecureContext)
                navigator.clipboard.writeText(text).then(() => {}, () => {});
            window.alert(rows.length
                ? `${rows.length} objects${copied ? ' (copied to clipboard)' : ' (copy failed: select the text below, or see the browser console)'}:\n` + text
                : 'No confirmed furniture yet (needs a loaded map and Reality view on).');
        };
        function furnitureYawEnabled() {
            const value = new URLSearchParams(window.location?.search).get('furnitureYawV2');
            if (value === '1' || value === '0') {
                try { localStorage.setItem('furnitureYawV2',value); } catch { /* Storage can be disabled. */ }
                return value === '1';
            }
            try { return localStorage.getItem('furnitureYawV2') === '1'; } catch { return false; }
        }
        const furnitureYawV2 = furnitureYawEnabled();
        let structTracker = window.RealityStructure ? new RealityStructure.Tracker({furnitureYawV2}) : null;
        let wallHistory = window.RealityStructure && new RealityStructure.WallHistory();
        let structOn = false;
        function analyzeStructure(cells, dets, d, mapName, xyz, nowMs, cloudPose, oakPose, people=[]) {
            const mapped = !!(mapName && d.robotPoseMap);
            const key = mapName || null;
            if (key !== roomKey && structTracker) {
                structTracker = new RealityStructure.Tracker({furnitureYawV2});
                roomKey = key;
                wallHistory = new RealityStructure.WallHistory();
                landmarksPutAt = nowMs;      // let the server copy arrive before overwriting it
                if (key && window.sendMessage) window.sendMessage({type: 'reality_landmarks_get', key: 'map:' + key});
            }
            const odomPoseCm = mapName ? d.robotPoseMap : d.robotPose;
            if (!structTracker) return null;
            const pose = odomPoseCm && [odomPoseCm.x, odomPoseCm.y, odomPoseCm.theta].every(Number.isFinite)
                ? { x: odomPoseCm.x / 100, y: odomPoseCm.y / 100, theta: odomPoseCm.theta } : null;
            // A cloud can arrive before its pose. Hold yaw evidence until the
            // pair is available, then the same cloud can contribute once.
            let observationPose = null;
            if (furnitureYawV2 && pose && cloudPose && [cloudPose.x,cloudPose.y,cloudPose.yaw].every(Number.isFinite)) {
                if (!mapped) observationPose = {x:cloudPose.x,y:cloudPose.y,theta:cloudPose.yaw};
                else if (d.robotPose && [d.robotPose.x,d.robotPose.y,d.robotPose.theta].every(Number.isFinite)) {
                    // The cloud pose arrives in odom. Apply the readout's map <- odom
                    // transform before fitting it into the map-frame furniture tracks.
                    const yaw = pose.theta - d.robotPose.theta;
                    const c=Math.cos(yaw), s=Math.sin(yaw);
                    const x=cloudPose.x-d.robotPose.x/100, y=cloudPose.y-d.robotPose.y/100;
                    observationPose = {x:pose.x+c*x-s*y,y:pose.y+s*x+c*y,theta:cloudPose.yaw+yaw};
                }
            }
            const st = RealityStructure.analyze(cells, dets, pose, structTracker, nowMs / 1000,
                {mapped, wallHistory,
                 wallPose: cloudPose || (d.robotPose && {x:d.robotPose.x/100, y:d.robotPose.y/100, yaw:d.robotPose.theta}),
                 xyz, detectionStamp: d.oakDetectionsAt, observationPose});
            if (mapped && pose) {
                const cabinet = cabinets.get(mapName);
                if (cabinet) {
                    const dx = cabinet.x - pose.x, dy = cabinet.y - pose.y;
                    const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
                    st.objects.push({...cabinet, id: 'manual-cabinet', kind: 'cabinet',
                        x: c*dx+s*dy, y: -s*dx+c*dy, yaw: cabinet.yaw-pose.theta});
                }
            }
            // Rendered boxes and detections are base-relative to the readout pose.
            // Use its paired odometry pose to place them in the OAK cloud's odom
            // frame. The OAK stream pose gives the observation bearing only.
            const odom = d.robotPose;
            const oak = oakPose && d.oakCloud;
            const detectedPeople = (dets || []).filter(det => det.label === 'person').map(det => det.xyz_base_m);
            // An unlocalized person cannot be masked in 3D. Wait for the next cloud.
            const canFuse = oak?.colors && odom && [odom.x,odom.y,odom.theta].every(Number.isFinite)
                && detectedPeople.every(p => p && Number.isFinite(p.x) && Number.isFinite(p.y));
            const toOdom = p => ({
                x:odom.x/100+Math.cos(odom.theta)*p.x-Math.sin(odom.theta)*p.y,
                y:odom.y/100+Math.sin(odom.theta)*p.x+Math.cos(odom.theta)*p.y
            });
            const masks = canFuse ? people.concat(detectedPeople).map(toOdom) : [];
            for (const o of st.objects) {
                const t = o.kind==='cabinet' ? cabinets.get(mapName) : structTracker.tracks.find(t=>t.id===o.id);
                if (!t || t.inferred || t.misses>0) continue;
                if (o.kind!=='cabinet' && t.hits < RealityStructure.LANDMARK_HITS) continue;
                if (canFuse) {
                    t.model ||= RealityStructure.Lookalike.create();
                    RealityStructure.Lookalike.ingest(t.model, oak,
                        {...o,...toOdom(o),yaw:o.yaw+odom.theta},oakPose,masks);
                }
                o.model=t.model;
            }
            if (mapped && pose && window.sendMessage && nowMs - landmarksPutAt > LANDMARK_PUT_MS) {
                landmarksPutAt = nowMs;
                const objects = structTracker.exportLandmarks();
                const cabinet = cabinets.get(mapName);
                if (cabinet?.model?.voxels.size) {
                    const {x:X,y:Y,yaw,L,D,H,model} = cabinet;
                    objects.push({kind:'cabinet',X,Y,yaw,L,D,H,lookalike:RealityStructure.Lookalike.pack(model)});
                }
                window.sendMessage({type: 'reality_landmarks_put', key: 'map:' + key,
                                    objects});
            }
            lastPose = mapped ? pose : null;
            updateStructure(st);
            structOn = true;
            return st;
        }

        // ---- theme swap -----------------------------------------------------
        const saved = {
            bg: scene.background.clone(), fog: scene.fog.color.clone(), fogNear: scene.fog.near, fogFar: scene.fog.far,
            floor: refs.floor.material.color.clone(),
            rings: refs.rings.map(m => m.color.clone()),
            tinted: refs.tinted.map(m => ({ color: m.color.clone(), emissive: m.emissive ? m.emissive.clone() : null })),
        };
        let themed = false;
        function applyTheme(on) {
            if (on === themed) return;
            themed = on;
            const bg = on ? new THREE.Color(DARK.bg) : saved.bg;
            scene.background.copy(bg);
            scene.fog.color.copy(bg);
            scene.fog.near = on ? 3.5 : saved.fogNear; scene.fog.far = on ? 10 : saved.fogFar;
            renderer.setClearColor(bg, 1);
            refs.floor.material.color.set(on ? DARK.floor : saved.floor);
            refs.rings.forEach((m, i) => m.color.copy(on ? new THREE.Color(DARK.ring) : saved.rings[i]));
            if (window.X3Humanoid && X3Humanoid.setStyle) X3Humanoid.setStyle(on ? 'mannequin' : 'default');
            refs.tinted.forEach((m, i) => {
                m.color.copy(on ? new THREE.Color(DARK.tint) : saved.tinted[i].color);
                if (m.emissive) m.emissive.copy(on ? new THREE.Color(DARK.emissive) : saved.tinted[i].emissive);
            });
        }

        // map (m) -> scene, relative to the robot pose (cm, rad)
        function updatePath(path, pose) {
            if (!path || path.length < 2 || !pose) { ribbon.visible = beacon.visible = false; return; }
            const px = pose.x / 100, py = pose.y / 100, c = Math.cos(pose.theta), s = Math.sin(pose.theta);
            const pts = [];
            for (const p of path) {
                const dx = p[0] - px, dy = p[1] - py;
                const fwd = c * dx + s * dy, left = -s * dx + c * dy;
                pts.push([-left, -fwd]);                          // scene x, z
            }
            // Start where the robot is: drop plan points already behind us.
            let start = 0, best = Infinity;
            for (let i = 0; i < Math.min(pts.length, 60); i++) {
                const d = Math.hypot(pts[i][0], pts[i][1]);
                if (d < best) { best = d; start = i; }
            }
            const use = pts.slice(start, start + MAX_PATH);
            if (use.length < 2) { ribbon.visible = beacon.visible = false; return; }
            let acc = 0;
            for (let i = 0; i < use.length; i++) {
                const a = use[Math.max(0, i - 1)], b = use[Math.min(use.length - 1, i + 1)];
                let tx = b[0] - a[0], tz = b[1] - a[1];
                const tl = Math.hypot(tx, tz) || 1; tx /= tl; tz /= tl;
                const nx = -tz * RIBBON_W / 2, nz = tx * RIBBON_W / 2;
                if (i) acc += Math.hypot(use[i][0] - use[i - 1][0], use[i][1] - use[i - 1][1]);
                const o = 6 * i;
                pos[o] = use[i][0] + nx; pos[o + 1] = RIBBON_Y; pos[o + 2] = use[i][1] + nz;
                pos[o + 3] = use[i][0] - nx; pos[o + 4] = RIBBON_Y; pos[o + 5] = use[i][1] - nz;
                along[2 * i] = along[2 * i + 1] = acc;
                side[2 * i] = 1; side[2 * i + 1] = -1;
            }
            geom.attributes.position.needsUpdate = true;
            geom.attributes.along.needsUpdate = true;
            geom.attributes.side.needsUpdate = true;
            geom.setDrawRange(0, (use.length - 1) * 6);
            ribbon.material.uniforms.total.value = acc;
            ribbon.visible = true;
            const g = use[use.length - 1];
            beacon.position.set(g[0], 0.8, g[1]);
            beacon.visible = true;
        }

        function updateShield(cbf) {
            if (!cbf) { shieldGroup.visible = false; return; }
            const key = cbf.blind ? 'blind' : cbf.hold ? 'hold' : cbf.limited ? 'limited' : 'ok';
            shieldColor.value.set(COLORS[key]);
            shieldRingMat.color.set(COLORS[key]);
            shieldPulse.value = key === 'limited' || key === 'blind' ? 1 : 0;
            shieldGroup.visible = true;
        }

        return {
            analyzeStructure,
            furnitureYawV2,
            // persons: [{x, z, sx, sz, path?}] in scene coordinates (m, m/s)
            update(enabled, d, nav, now, persons, structure = true) {
                applyTheme(enabled);
                cabinetButton.disabled = !(enabled && structure && roomKey && roomKey === nav?.roomMap && d.robotPoseMap);
                if ((!enabled || !structure) && structOn) { updateStructure(null); structOn = false; }
                if (!enabled) { ribbon.visible = beacon.visible = shieldGroup.visible = false; hideTrails(0); return; }
                updateIntent(persons);
                time.value = now / 1000;
                updatePath(nav && nav.path, d.robotPoseMap);
                updateShield(d.readout && d.readout.cbf);
            },
        };
    }
    return { create };
})();
