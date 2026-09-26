// =================================================================
// Surroundings view (Drive tab): a Tesla-style chase-cam scene built only from
// data the server already streams -- no extra work on the Jetson.
//   people : readout.velocity_estimates (camera frame: x=right, z=forward)
//   walls  : /scan via the Foxglove bridge. foxgloveScanToXY leaves points in
//            laser_link as (x, -y); laser_link is yawed 180 deg and 0.044 m
//            ahead of base_link (tf2_echo on the robot, 2026-09-14).
// Scene frame: robot at origin, forward = -Z, right = +X, up = +Y.
// Velocities use the same convention the CBF uses (vx=forward, vy=left).
// =================================================================
(() => {
    const MAX_WALL_PTS = 1500;
    const WALL_H = 0.45;
    const RANGE_M = 6.0;
    const GHOST_S = 1.0;          // prediction horizon for the faded ghost
    const LASER_X = 0.044;        // base_link -> laser_link x offset (m)
    const SCAN_STALE_MS = 1000;   // hide walls when /scan stops (e.g. 3D sweep gate)

    let lastPts = null, lastPtsAt = 0;
    let renderer, scene, camera, container, hint, _m, _dir, _org;
    const people = new Map();     // track id -> {group, arrow, ghost}

    function init() {
        container = document.getElementById('scene-view');
        hint = document.getElementById('scene-view-hint');
        if (!container) return;
        if (typeof THREE === 'undefined') return fail('Three.js did not load (CDN blocked?)');
        try {
            renderer = new THREE.WebGLRenderer({ antialias: true });
        } catch (e) {
            return fail('WebGL unavailable: ' + e.message);
        }
        _m = new THREE.Matrix4(); _dir = new THREE.Vector3(); _org = new THREE.Vector3();
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        container.appendChild(renderer.domElement);

        scene = new THREE.Scene();
        scene.background = new THREE.Color(0xe9ecef);
        scene.fog = new THREE.Fog(0xe9ecef, 4, 9);

        camera = new THREE.PerspectiveCamera(50, 1, 0.05, 50);
        camera.position.set(0, 2.4, 2.2);
        camera.lookAt(0, 0, -1.6);

        scene.add(new THREE.HemisphereLight(0xffffff, 0x9aa3ad, 0.9));
        const sun = new THREE.DirectionalLight(0xffffff, 0.5);
        sun.position.set(2, 6, 3);
        sun.castShadow = true;
        sun.shadow.mapSize.set(1024, 1024);
        sun.shadow.radius = 4;
        Object.assign(sun.shadow.camera, { left: -7, right: 7, top: 7, bottom: -7, near: 0.5, far: 20 });
        scene.add(sun);

        const floor = new THREE.Mesh(new THREE.PlaneGeometry(30, 30),
            new THREE.MeshLambertMaterial({ color: 0xf8f9fa }));
        floor.rotation.x = -Math.PI / 2;
        floor.receiveShadow = true;
        scene.add(floor);

        // 1 m range rings
        for (let r = 1; r <= RANGE_M; r++) {
            const ring = new THREE.Mesh(new THREE.RingGeometry(r - 0.008, r + 0.008, 96),
                new THREE.MeshBasicMaterial({ color: 0xc5ccd3 }));
            ring.rotation.x = -Math.PI / 2;
            ring.position.y = 0.002;
            scene.add(ring);
        }

        // Robot: URDF model baked by src/build_web_robot_model.py, with a
        // box + blue nose standing in until (or if) it loads.
        const robot = new THREE.Group();
        const body = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.12, 0.30),
            new THREE.MeshLambertMaterial({ color: 0x2b2f36 }));
        body.position.y = 0.08;
        body.castShadow = true;
        const nose = new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.02, 0.05),
            new THREE.MeshLambertMaterial({ color: 0x3b82f6 }));
        nose.position.set(0, 0.15, -0.12);
        robot.add(body, nose);
        scene.add(robot);
        loadRobotModel(robot);

        initWalls();

        new ResizeObserver(resize).observe(container);
        resize();
        requestAnimationFrame(loop);
    }

    // URDF frame (x fwd, y left, z up) -> scene (x right, y up, z back)
    const BASE_Z = 0.0815;        // base_joint: base_link above base_footprint
    let tiltNode = null, tiltRest = null, _tq = null;

    function loadRobotModel(placeholder) {
        if (!THREE.GLTFLoader) return;
        new THREE.GLTFLoader().load('models/x3_robot.glb?v=1', (gltf) => {
            const wrap = new THREE.Group();
            wrap.matrixAutoUpdate = false;
            wrap.matrix.set(0, -1, 0, 0,
                            0, 0, 1, 0,
                            -1, 0, 0, 0,
                            0, 0, 0, 1);
            const model = gltf.scene;
            model.position.z = BASE_Z;
            model.traverse((o) => {
                if (o.isMesh) {
                    o.castShadow = true;
                    // trimesh exports a metallic PBR material, which renders
                    // near-black without an environment map
                    o.material.metalness = 0;
                    o.material.roughness = 0.8;
                    o.material.side = THREE.DoubleSide;
                }
                if (o.name === 'tilt') tiltNode = o;
            });
            if (tiltNode) {
                tiltRest = tiltNode.quaternion.clone();
                _tq = new THREE.Quaternion();
            }
            wrap.add(model);
            scene.remove(placeholder);
            scene.add(wrap);
        }, undefined, (e) => console.warn('[scene-view] robot model failed, keeping box:', e));
    }

    // lidar_tilt_joint = radians(readout.tilt.deg), about the joint's +Y
    const _yAxis = { x: 0, y: 1, z: 0 };
    function updateTilt(readout) {
        if (!tiltNode || !readout || !readout.tilt || !isFinite(readout.tilt.deg)) return;
        _tq.setFromAxisAngle(_yAxis, readout.tilt.deg * Math.PI / 180);
        tiltNode.quaternion.copy(tiltRest).multiply(_tq);
    }

    function fail(msg) {
        console.error('[scene-view]', msg);
        container.innerHTML = '<div style="padding:1rem;color:var(--accent-red)">' + msg + '</div>';
    }

    function resize() {
        const w = container.clientWidth, h = container.clientHeight;
        if (!w || !h) return;
        renderer.setSize(w, h, false);
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
    }

    // ---- Wall post-processing (all browser-side) ---------------------------
    // 1. Per-beam smoothing: scan points are binned by laser angle and each bin's
    //    range is low-passed, except on jumps > SMOOTH_JUMP_M so movers stay live.
    //    A bin missed by a scan survives BIN_HOLD_SCANS scans before it drops.
    // 2. Line fitting: angular runs of nearby bins are split (end-point fit) into
    //    straight pieces; long, well-supported pieces become wall panels and the
    //    rest stay as posts (chair legs, clutter).
    const BIN_DEG = 0.5;
    const N_BINS = 360 / BIN_DEG;
    const SMOOTH_ALPHA = 0.35;     // weight of the newest range
    const SMOOTH_JUMP_M = 0.15;
    const BIN_HOLD_SCANS = 2;
    const SPLIT_TOL_M = 0.04;      // max point-to-line deviation inside a panel
    const PANEL_MIN_PTS = 8;
    const PANEL_MIN_LEN_M = 0.35;
    const MAX_PANELS = 400;
    const PANEL_T = 0.04;

    const binRange = new Float32Array(N_BINS);    // 0 = empty
    const binMiss = new Uint8Array(N_BINS);
    const binHit = new Uint8Array(N_BINS);
    let panels, posts, NEAR_COLOR, FAR_COLOR, _c, _q, _s, _p, _up;

    function initWalls() {
        NEAR_COLOR = new THREE.Color(0x8b95a1);
        FAR_COLOR = new THREE.Color(0xd9dee3);
        _c = new THREE.Color(); _q = new THREE.Quaternion(); _s = new THREE.Vector3();
        _p = new THREE.Vector3(); _up = new THREE.Vector3(0, 1, 0);
        const mat = () => new THREE.MeshLambertMaterial({ color: 0xffffff });
        panels = new THREE.InstancedMesh(new THREE.BoxGeometry(1, WALL_H, PANEL_T), mat(), MAX_PANELS);
        posts = new THREE.InstancedMesh(new THREE.BoxGeometry(0.05, WALL_H, 0.05), mat(), MAX_WALL_PTS);
        for (const m of [panels, posts]) {
            m.castShadow = true;
            m.setColorAt(0, NEAR_COLOR);   // allocates instanceColor
            m.count = 0;
            scene.add(m);
        }
    }

    function ingestScan(pts) {
        binHit.fill(0);
        for (let i = 0; i + 1 < pts.length; i += 2) {
            const lx = pts[i], ly = -pts[i + 1];
            if (!isFinite(lx) || !isFinite(ly)) continue;
            const r = Math.hypot(lx, ly);
            if (r > RANGE_M) continue;
            let a = Math.atan2(ly, lx) * 180 / Math.PI;
            if (a < 0) a += 360;
            const b = Math.min(N_BINS - 1, Math.floor(a / BIN_DEG));
            if (binHit[b]) { binRange[b] = Math.min(binRange[b], r); continue; }
            const old = binRange[b];
            binRange[b] = (old === 0 || Math.abs(r - old) > SMOOTH_JUMP_M)
                ? r : old + SMOOTH_ALPHA * (r - old);
            binHit[b] = 1;
        }
        for (let b = 0; b < N_BINS; b++) {
            if (binHit[b]) binMiss[b] = 0;
            else if (binRange[b] && ++binMiss[b] > BIN_HOLD_SCANS) binRange[b] = 0;
        }
    }

    function clearBins() { binRange.fill(0); binMiss.fill(0); }

    function fadeColor(dist) {
        return _c.copy(NEAR_COLOR).lerp(FAR_COLOR, Math.min(1, dist / RANGE_M));
    }

    // Scene-frame (x=right, z=-forward) point list from the smoothed bins.
    function binsToPoints() {
        const xs = [], zs = [];
        for (let b = 0; b < N_BINS; b++) {
            const r = binRange[b];
            if (!r) { xs.push(NaN); zs.push(NaN); continue; }
            const a = (b + 0.5) * BIN_DEG * Math.PI / 180;
            const lx = r * Math.cos(a), ly = r * Math.sin(a);
            // laser_link yawed 180 deg: fwd = LASER_X - lx, right = ly
            xs.push(ly); zs.push(-(LASER_X - lx));
        }
        return { xs, zs };
    }

    function rebuildWalls() {
        const { xs, zs } = binsToPoints();
        // Angular runs of spatially-close points
        const runs = [];
        let cur = [];
        for (let i = 0; i < N_BINS; i++) {
            if (isNaN(xs[i])) { if (cur.length) runs.push(cur); cur = []; continue; }
            if (cur.length) {
                const j = cur[cur.length - 1];
                const gap = Math.hypot(xs[i] - xs[j], zs[i] - zs[j]);
                const range = Math.hypot(xs[i], zs[i]);
                if (gap > 0.08 + 0.03 * range) { runs.push(cur); cur = []; }
            }
            cur.push(i);
        }
        if (cur.length) runs.push(cur);

        const pieces = [];
        const split = (idx) => {
            if (idx.length < 3) { pieces.push(idx); return; }
            const a = idx[0], b = idx[idx.length - 1];
            const dx = xs[b] - xs[a], dz = zs[b] - zs[a];
            const len = Math.hypot(dx, dz) || 1e-6;
            let worst = 0, wi = -1;
            for (let k = 1; k < idx.length - 1; k++) {
                const i = idx[k];
                const d = Math.abs((xs[i] - xs[a]) * dz - (zs[i] - zs[a]) * dx) / len;
                if (d > worst) { worst = d; wi = k; }
            }
            if (worst > SPLIT_TOL_M) { split(idx.slice(0, wi + 1)); split(idx.slice(wi)); }
            else pieces.push(idx);
        };
        runs.forEach(split);

        let np = 0, nq = 0;
        for (const idx of pieces) {
            const fit = idx.length >= PANEL_MIN_PTS && fitLine(idx, xs, zs);
            if (fit && fit.len >= PANEL_MIN_LEN_M && np < MAX_PANELS) {
                _p.set(fit.cx, WALL_H / 2, fit.cz);
                _q.setFromAxisAngle(_up, -Math.atan2(fit.dz, fit.dx));
                _s.set(fit.len, 1, 1);
                _m.compose(_p, _q, _s);
                panels.setMatrixAt(np, _m);
                panels.setColorAt(np, fadeColor(Math.hypot(fit.cx, fit.cz)));
                np++;
            } else {
                for (const i of idx) {
                    if (nq >= MAX_WALL_PTS) break;
                    _m.makeTranslation(xs[i], WALL_H / 2, zs[i]);
                    posts.setMatrixAt(nq, _m);
                    posts.setColorAt(nq, fadeColor(Math.hypot(xs[i], zs[i])));
                    nq++;
                }
            }
        }
        panels.count = np; posts.count = nq;
        for (const m of [panels, posts]) {
            m.instanceMatrix.needsUpdate = true;
            if (m.instanceColor) m.instanceColor.needsUpdate = true;
        }
    }

    // Least-squares (PCA) line through the points; the panel spans the
    // projections of the extreme points onto it.
    function fitLine(idx, xs, zs) {
        let mx = 0, mz = 0;
        for (const i of idx) { mx += xs[i]; mz += zs[i]; }
        mx /= idx.length; mz /= idx.length;
        let sxx = 0, szz = 0, sxz = 0;
        for (const i of idx) {
            const x = xs[i] - mx, z = zs[i] - mz;
            sxx += x * x; szz += z * z; sxz += x * z;
        }
        const th = 0.5 * Math.atan2(2 * sxz, sxx - szz);
        const dx = Math.cos(th), dz = Math.sin(th);
        let tmin = Infinity, tmax = -Infinity;
        for (const i of idx) {
            const t = (xs[i] - mx) * dx + (zs[i] - mz) * dz;
            if (t < tmin) tmin = t;
            if (t > tmax) tmax = t;
        }
        const mid = (tmin + tmax) / 2;
        return { cx: mx + dx * mid, cz: mz + dz * mid, dx, dz, len: tmax - tmin };
    }

    function makePerson() {
        const group = new THREE.Group();
        const bodyMat = new THREE.MeshLambertMaterial({ color: 0xf59e0b });
        const cyl = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 1.2, 24), bodyMat);
        cyl.position.y = 0.6;
        cyl.castShadow = true;
        group.add(cyl);
        const ghost = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 1.2, 24),
            new THREE.MeshLambertMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.25 }));
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(),
            1, 0xdc2626, 0.15, 0.1);
        scene.add(group, ghost, arrow);
        return { group, ghost, arrow };
    }

    function removeTrack(p) {
        scene.remove(p.group, p.ghost, p.arrow);
    }

    let moverGeom = null, moverMat = null, moverGhostMat = null;
    function makeMover() {
        if (!moverGeom) {
            moverGeom = new THREE.BoxGeometry(0.34, 0.12, 0.34);
            moverGeom.translate(0, 0.06, 0);
            moverMat = new THREE.MeshLambertMaterial({ color: 0xf59e0b });
            moverGhostMat = new THREE.MeshLambertMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.3 });
        }
        const group = new THREE.Mesh(moverGeom, moverMat);
        const ghost = new THREE.Mesh(moverGeom, moverGhostMat);
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(),
            1, 0xdc2626, 0.15, 0.1);
        scene.add(group, ghost, arrow);
        return { group, ghost, arrow };
    }

    function updatePeople(estimates) {
        const seen = new Set();
        for (const est of estimates || []) {
            const fwd = est.z, right = est.x;
            if (!isFinite(fwd) || !isFinite(right)) continue;
            // Static depth blobs (furniture, door frames) are already drawn by
            // the /scan walls. People get an avatar, confirmed non-person
            // movers ("dynamic", e.g. the Roomba) an amber box. A null
            // category means no live detector, so draw as a person as before.
            if (est.category === 'static') continue;
            const kind = est.category === 'dynamic' ? 'dynamic' : 'person';
            seen.add(est.id);
            let p = people.get(est.id);
            if (p && p.kind !== kind) { removeTrack(p); p = null; }
            if (!p) {
                p = kind === 'dynamic' ? makeMover() : makePerson();
                p.kind = kind;
                people.set(est.id, p);
            }
            p.group.position.set(right, 0, -fwd);

            // vx=forward, vy=left (CBF convention) -> scene (x=-vy, z=-vx)
            const sx = -(est.vy || 0), sz = -(est.vx || 0);
            const speed = Math.hypot(sx, sz);
            const moving = speed > 0.15;
            p.ghost.visible = moving;
            p.arrow.visible = moving;
            if (moving) {
                p.ghost.position.set(right + sx * GHOST_S, kind === 'dynamic' ? 0 : 0.6,
                    -fwd + sz * GHOST_S);
                _org.set(right, kind === 'dynamic' ? 0.3 : 1.3, -fwd);
                _dir.set(sx, 0, sz).normalize();
                p.arrow.position.copy(_org);
                p.arrow.setDirection(_dir);
                p.arrow.setLength(Math.max(0.3, speed * GHOST_S), 0.15, 0.1);
            }
        }
        for (const [id, p] of people) {
            if (seen.has(id)) continue;
            removeTrack(p);
            people.delete(id);
        }
    }

    // Keep /scan subscribed while this view is on screen, independent of the
    // Sweep tab's Lidar toggle; release it again when hidden unless that toggle wants it.
    function syncScanSubscription(visible) {
        if (typeof foxgloveResubscribe !== 'function') return;
        if (visible) foxgloveResubscribe('/scan');
        else if (!state.lidarEnabled) foxgloveUnsubscribe('/scan');
    }

    let wasVisible = null;
    function loop() {
        requestAnimationFrame(loop);
        const visible = !!container.offsetParent;
        if (visible !== wasVisible || visible) syncScanSubscription(visible);
        wasVisible = visible;
        if (!visible) return;   // Drive tab hidden: skip rendering
        const d = state.latestData;
        const now = performance.now();
        try {
        if (d.lidarPoints !== lastPts) {
            lastPts = d.lidarPoints; lastPtsAt = now;
            if (lastPts) { ingestScan(lastPts); rebuildWalls(); }
        }
        const scanFresh = lastPts && now - lastPtsAt < SCAN_STALE_MS;
        if (!scanFresh && (panels.count || posts.count)) {
            clearBins(); panels.count = 0; posts.count = 0;
        }
        updatePeople(d.velocityEstimates);
        updateTilt(d.readout);
        if (hint) {
            const msgs = [];
            if (!scanFresh) msgs.push('No lidar scan');
            if (!d.velocityEstimates || !d.velocityEstimates.length) msgs.push('No people tracked');
            hint.textContent = msgs.join(' · ');
        }
        renderer.render(scene, camera);
        } catch (e) {
            if (hint) hint.textContent = 'Render error: ' + e.message;
        }
    }

    function safeInit() {
        try { init(); } catch (e) { fail('Init failed: ' + e.message); }
    }
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', safeInit);
    else safeInit();
})();
