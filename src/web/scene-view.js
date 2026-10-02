// =================================================================
// Surroundings view (Drive tab): a Tesla-style chase-cam scene built only from
// data the server already streams -- no extra work on the Jetson.
//   people : c3_tracks when the C3 tracker runs (robot frame: fwd, left;
//            capture-time corrected, 0.5 s prediction), else
//            readout.velocity_estimates (camera frame: x=right, z=forward).
//            v3 still supplies non-person movers (amber boxes).
//   c3     : c3_tracks, the diagnostic C3 Kalman tracker (robot frame:
//            fwd, left). Cyan floor rings: now, and PREDICT_S ahead at 2 sigma.
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
    const c3 = new Map();         // C3 track id -> {now, pred, line}
    let c3RingGeom = null, c3Mat = null, c3PredMat = null;

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
        const avatar = X3Humanoid.create();
        const prediction = X3Humanoid.create({ ghost: true });
        const group = avatar.group, ghost = prediction.group;
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(),
            1, 0xdc2626, 0.15, 0.1);
        scene.add(group, ghost, arrow);
        return { group, ghost, arrow, avatar, prediction, joints: avatar.joints, predictionJoints: prediction.joints };
    }

    function removeTrack(p) {
        scene.remove(p.group, p.ghost, p.arrow);
        // Avatar/mover meshes share geometry/materials; only the per-track
        // ArrowHelper materials are owned here (its geometry is shared too).
        p.arrow.line.material.dispose();
        p.arrow.cone.material.dispose();
    }

    let moverGeom = null, moverMat = null, moverGhostMat = null;
    function makeMover() {
        if (!moverGeom) {
            moverGeom = new THREE.BoxGeometry(0.34, 0.12, 0.34);
            moverGeom.translate(0, 0.06, 0);
            moverMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.6 });
            moverGhostMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.3 });
        }
        const group = new THREE.Mesh(moverGeom, moverMat);
        const ghost = new THREE.Mesh(moverGeom, moverGhostMat);
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(),
            1, 0xdc2626, 0.15, 0.1);
        scene.add(group, ghost, arrow);
        const still = { animate() {} };
        return { group, ghost, arrow, avatar: still, prediction: still };
    }

    // COCO-17 limb endpoints, with confidence checked at both joints.
    const BODY_KPTS = {
        left_upper_arm: [5, 7], left_lower_arm: [7, 9],
        right_upper_arm: [6, 8], right_lower_arm: [8, 10],
        left_thigh: [11, 13], left_shin: [13, 15],
        right_thigh: [12, 14], right_shin: [14, 16],
    };
    const POSE_MATCH_M = 0.8;     // max est <-> OAK person distance (x/z, camera frame)
    const POSE_STALE_MS = 500;

    // Limb directions for one tracked person from the nearest unused OAK pose
    // detection. keypoints_xyz is CAM_A optical (x right, y down, z fwd) ->
    // scene (x, -y, -z). Only directions are used, so the camera height and
    // the shared torso-plane depth drop out.
    function poseDirs(est, dets, used) {
        if (!dets) return null;
        let best = -1, bestD = POSE_MATCH_M;
        dets.forEach((d, i) => {
            if (used.has(i) || d.label !== 'person' || !d.keypoints_xyz || !d.xyz_m) return;
            const dist = Math.hypot(d.xyz_m.x - est.x, d.xyz_m.z - est.z);
            if (dist < bestD) { bestD = dist; best = i; }
        });
        if (best < 0) return null;
        used.add(best);
        const k = dets[best].keypoints_xyz, confidence = dets[best].keypoints, dirs = {};
        function score(i) {
            const c = confidence && confidence[i] && confidence[i][2];
            return k[i] && k[i].every(Number.isFinite) && Number.isFinite(c) ? c : 0;
        }
        for (const [bone, [a, b]] of Object.entries(BODY_KPTS)) {
            const c = Math.min(score(a), score(b));
            if (c < 0.5) continue;
            const v = new THREE.Vector3(k[b][0] - k[a][0], -(k[b][1] - k[a][1]), -(k[b][2] - k[a][2]));
            if (v.lengthSq() > 1e-4) dirs[bone] = { direction: v.normalize(), confidence: c };
        }
        // Torso points down from shoulder midpoint to hip midpoint. The current
        // observations share one depth plane: do not invent forward knee bends.
        const c = Math.min(...[5, 6, 11, 12].map(score));
        if (c >= 0.5) {
            const v = new THREE.Vector3(
                (k[11][0] + k[12][0] - k[5][0] - k[6][0]) / 2,
                -(k[11][1] + k[12][1] - k[5][1] - k[6][1]) / 2,
                -(k[11][2] + k[12][2] - k[5][2] - k[6][2]) / 2);
            if (v.lengthSq() > 0.01) dirs.torso = { direction: v.normalize(), confidence: c };
        }
        if (window.X3PoseRefinement) dirs.facing = window.X3PoseRefinement.facingCue(k, confidence || []);
        return dirs;
    }

    // Avatars follow C3 when it runs: it is corrected for camera latency and
    // uses the fixed preview geometry, one track per person. v3 (the MLP)
    // stays the CBF's input and here only contributes non-person movers.
    // Entries are normalised to v3's shape: x=right, z=forward in the CAMERA
    // frame (poseDirs matches keypoints on that), vx=forward, vy=left.
    const OAK_MOUNT_X = 0.107815;   // oakd_driver.OAK_MOUNT_X, camera ahead of base
    function avatarEntries(d) {
        if (!d.c3Stats) return d.velocityEstimates || [];   // C3 off: v3 as before
        const out = [];
        for (const t of d.c3Tracks || []) {
            out.push({ id: 'c3-' + t.id, x: -t.left, z: t.fwd - OAK_MOUNT_X, fwdBase: t.fwd,
                       vx: t.vx, vy: t.vy, category: 'person',
                       horizon: t.predict_s, measured: t.measured });
        }
        for (const e of d.velocityEstimates || []) {
            if (e.category === 'dynamic') out.push(Object.assign({}, e, { id: 'v3-' + e.id }));
        }
        return out;
    }

    // Travel is only a heading cue after sustained displacement. Instantaneous
    // tracker velocity can spike when someone raises a knee or changes posture.
    function updateHeading(p, right, fwd, speed, dt, facing = null) {
        const step = Number.isFinite(dt) ? Math.max(0, Math.min(dt, 0.1)) : 0;
        if (!p.heading) p.heading = { x: right, z: fwd, age: 0, target: p.group.rotation.y };
        const h = p.heading;
        h.bodyHold = Math.max(0, (h.bodyHold || 0) - step);
        if (facing && Number.isFinite(facing.yaw) && facing.confidence >= 0.75) {
            h.bodyAge = h.bodyCandidate === facing.yaw ? (h.bodyAge || 0) + step : 0;
            h.bodyCandidate = facing.yaw;
            if (h.bodyAge >= 0.6) {
                h.target = facing.yaw; h.bodyHold = 0.8;
            }
        } else { h.bodyAge = 0; h.bodyCandidate = null; }
        if (speed < 0.25) {
            h.x = right; h.z = fwd; h.age = 0;
        } else {
            h.age += step;
            const dx = right - h.x, dz = fwd - h.z;
            if (h.age >= 0.35 && Math.hypot(dx, dz) >= 0.18) {
                if (!h.bodyHold) h.target = Math.atan2(-dx, dz);
                h.x = right; h.z = fwd; h.age = 0;
            }
        }
        const delta = Math.atan2(Math.sin(h.target - p.group.rotation.y),
            Math.cos(h.target - p.group.rotation.y));
        p.group.rotation.y += delta * (1 - Math.exp(-step / 0.25));
    }

    let contactRobotPose = null, robotContactHold = 0.3;
    function updatePeople(estimates, dt) {
        const seen = new Set();
        const d = state.latestData;
        const dets = d.oakDetectionsAt && performance.now() - d.oakDetectionsAt < POSE_STALE_MS
            ? d.oakDetections : null;
        const used = new Set();
        const rp = d.robotPose;
        const validRobotPose = rp && [rp.x, rp.y, rp.theta].every(Number.isFinite);
        robotContactHold = Math.max(0, robotContactHold - Math.max(0, Math.min(dt || 0, 0.1)));
        if (validRobotPose) {
            // robotPose x/y arrive in cm. Do not pin feet in the moving robot frame.
            if (!contactRobotPose || Math.hypot(rp.x-contactRobotPose.x, rp.y-contactRobotPose.y) > 0.1 ||
                Math.abs(Math.atan2(Math.sin(rp.theta-contactRobotPose.theta), Math.cos(rp.theta-contactRobotPose.theta))) > 0.002) {
                robotContactHold = 0.3;
                contactRobotPose = { x: rp.x, y: rp.y, theta: rp.theta };
            }
        } else { robotContactHold = 0.3; contactRobotPose = null; }
        const facingToggle = document.getElementById('body-facing-toggle');
        const bodyFacing = facingToggle && facingToggle.checked;
        for (const est of estimates || []) {
            // Scene origin is the robot base; C3 gives base-frame forward directly.
            const fwd = est.fwdBase !== undefined ? est.fwdBase : est.z, right = est.x;
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
            p.missingFor = 0;
            p.group.visible = true;
            p.group.position.set(right, 0, -fwd);

            // vx=forward, vy=left (CBF convention) -> scene (x=-vy, z=-vx)
            const sx = -(est.vy || 0), sz = -(est.vx || 0);
            const speed = Math.hypot(sx, sz);
            const moving = speed > 0.15;
            const bodyPose = kind === 'person' ? poseDirs(est, dets, used) : null;
            // Set heading before world-space pose fitting, including on turns.
            if (!bodyFacing && p.heading) p.heading.bodyHold = 0;
            updateHeading(p, right, fwd, speed, dt, bodyFacing && bodyPose ? bodyPose.facing : null);
            p.avatar.animate(speed, dt);
            if (p.avatar.pose) p.avatar.pose(bodyPose, dt, { contacts: robotContactHold === 0 && est.measured !== false });
            p.ghost.rotation.y = p.group.rotation.y;
            const horizon = est.horizon || GHOST_S;
            p.ghost.position.set(right + sx * horizon, 0, -fwd + sz * horizon);
            p.prediction.animate(speed, dt);
            if (p.prediction.pose) p.prediction.pose(bodyPose, dt);
            p.ghost.visible = moving;
            p.arrow.visible = moving;
            if (moving) {
                // Arrow follows velocity; body retains its filtered travel heading.
                _org.set(right, kind === 'dynamic' ? 0.3 : 1.3, -fwd);
                _dir.set(sx, 0, sz).normalize();
                p.arrow.position.copy(_org);
                p.arrow.setDirection(_dir);
                p.arrow.setLength(Math.max(0.3, speed * horizon), 0.15, 0.1);
            }
        }
        for (const [id, p] of people) {
            if (seen.has(id)) continue;
            // Brief same-ID dropouts must not erase facing/pose history.
            p.missingFor = (p.missingFor || 0) + Math.max(0, Math.min(dt || 0, 0.1));
            p.group.visible = p.ghost.visible = p.arrow.visible = false;
            if (p.missingFor < 0.75) continue;
            removeTrack(p);
            people.delete(id);
        }
    }

    const footMarkers = new Map();
    let footGeom, footMat, footArrow;
    function updateFootDiagnostic() {
        const d = state.latestData;
        const toggle = document.getElementById('foot-diagnostic-toggle');
        const status = document.getElementById('foot-diagnostic-status');
        const reportedAge = d.footDiagnostic ? (d.footDiagnostic.capture_age_s || 0) + (d.footDiagnostic.update_age_s || 0) : 0;
        const fresh = d.footDiagnosticAt && performance.now()-d.footDiagnosticAt + reportedAge*1000 < 650;
        const diag = fresh ? d.footDiagnostic : null;
        const show = toggle && toggle.checked;
        const seen = new Set();
        if (!footGeom) {
            footGeom = new THREE.RingGeometry(0.85, 1, 32);
            footGeom.rotateX(-Math.PI/2);
            footMat = new THREE.MeshBasicMaterial({color:0xd946ef,transparent:true,opacity:0.8,side:THREE.DoubleSide});
            footArrow = new THREE.ArrowHelper(new THREE.Vector3(0,0,-1),new THREE.Vector3(0,.42,0),1,0xd946ef,.12,.08);
            scene.add(footArrow);
        }
        for (const f of show && diag ? diag.feet || [] : []) {
            if (![f.fwd,f.left,f.radius_m,f.sigma_m].every(Number.isFinite)) continue;
            seen.add(f.id);
            let marker = footMarkers.get(f.id);
            if (!marker) { marker = new THREE.Mesh(footGeom,footMat); scene.add(marker); footMarkers.set(f.id,marker); }
            // Foot extent + uncertainty, excluding robot radius. Camera/base -> scene.
            marker.position.set(-f.left,.018,-f.fwd);
            marker.scale.setScalar(f.radius_m+2*f.sigma_m);
        }
        for (const [id,m] of footMarkers) if (!seen.has(id)) { scene.remove(m); footMarkers.delete(id); }
        const shadow = diag && diag.shadow;
        const v = shadow && shadow.velocity;
        const speed = v && Math.hypot(v.fwd,v.left);
        footArrow.visible = !!(show && Number.isFinite(speed) && speed > .005);
        if (footArrow.visible) {
            footArrow.setDirection(new THREE.Vector3(-v.left,0,-v.fwd).normalize());
            // Two seconds of suggested travel, with a small visible minimum.
            footArrow.setLength(Math.max(.35,2*speed),.10,.06);
        }
        if (status) {
            status.style.display = show ? '' : 'none';
            if (!diag) status.textContent = 'Foot diagnostic: waiting/stale · motors unchanged';
            else if (diag.status !== 'ok') status.textContent = `Foot diagnostic: ${diag.status} · motors unchanged`;
            else {
                const n = (diag.feet || []).length;
                const reason = Object.entries(diag.rejected || {}).map(([k,v]) => `${k}: ${v}`).join(', ');
                const suggestion = v ? `would move ${speed.toFixed(2)} m/s` : (shadow ? shadow.status : 'unavailable');
                const visibility = (diag.rejected || {}).ankle_not_visible;
                status.textContent = n
                    ? `${n} MAGENTA foot regions · ${suggestion} · feet only, motors unchanged`
                    : (visibility ? 'No foot markers: ankle confidence too low or ankles missing'
                                  : 'No foot markers: no usable foot depth');
                if (!n && reason) status.textContent += ` · ${reason}`;
                if (!n) status.textContent += ' · cyan rings belong to person tracking';
            }
        }
    }

    function makeC3() {
        if (!c3RingGeom) {
            c3RingGeom = new THREE.RingGeometry(0.9, 1.0, 40);
            c3RingGeom.rotateX(-Math.PI / 2);
            c3Mat = new THREE.MeshBasicMaterial({ color: 0x0891b2, side: THREE.DoubleSide });
            c3PredMat = new THREE.MeshBasicMaterial({ color: 0x0891b2, transparent: true,
                opacity: 0.45, side: THREE.DoubleSide });
        }
        const now = new THREE.Mesh(c3RingGeom, c3Mat);
        const pred = new THREE.Mesh(c3RingGeom, c3PredMat);
        const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(
            [new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineBasicMaterial({ color: 0x0891b2 }));
        scene.add(now, pred, line);
        return { now, pred, line };
    }

    function updateC3(tracks) {
        const seen = new Set();
        for (const t of tracks || []) {
            if (!isFinite(t.fwd) || !isFinite(t.left)) continue;
            seen.add(t.id);
            let c = c3.get(t.id);
            if (!c) { c = makeC3(); c3.set(t.id, c); }
            // Robot frame (fwd, left) -> scene (x = -left, z = -fwd); 1 cm up to avoid z-fighting.
            const x = -t.left, z = -t.fwd;
            const px = x - t.vy * t.predict_s, pz = z - t.vx * t.predict_s;
            c.now.position.set(x, 0.01, z);
            c.now.scale.setScalar(Math.max(0.15, 2 * t.pos_sigma_m));
            c.now.material = t.measured ? c3Mat : c3PredMat;   // coasting = faded
            c.pred.position.set(px, 0.01, pz);
            c.pred.scale.setScalar(Math.max(0.15, 2 * t.pred_sigma_m));
            const pos = c.line.geometry.attributes.position;
            pos.setXYZ(0, x, 0.02, z); pos.setXYZ(1, px, 0.02, pz);
            pos.needsUpdate = true;
        }
        for (const [id, c] of c3) {
            if (seen.has(id)) continue;
            scene.remove(c.now, c.pred, c.line);
            c.line.geometry.dispose(); c.line.material.dispose();
            c3.delete(id);
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
    let previousFrameAt = performance.now();
    function loop() {
        requestAnimationFrame(loop);
        const now = performance.now();
        const dt = (now - previousFrameAt) / 1000;
        previousFrameAt = now;
        const visible = !!container.offsetParent;
        if (visible !== wasVisible || visible) syncScanSubscription(visible);
        wasVisible = visible;
        if (!visible) return;   // Drive tab hidden: skip rendering
        const d = state.latestData;
        try {
        if (d.lidarPoints !== lastPts) {
            lastPts = d.lidarPoints; lastPtsAt = now;
            if (lastPts) { ingestScan(lastPts); rebuildWalls(); }
        }
        const scanFresh = lastPts && now - lastPtsAt < SCAN_STALE_MS;
        if (!scanFresh && (panels.count || posts.count)) {
            clearBins(); panels.count = 0; posts.count = 0;
        }
        const avatars = avatarEntries(d);
        updatePeople(avatars, dt);
        const uncertaintyToggle = document.getElementById('person-uncertainty-toggle');
        updateC3(!uncertaintyToggle || uncertaintyToggle.checked ? d.c3Tracks : []);
        updateFootDiagnostic();
        updateTilt(d.readout);
        if (hint) {
            const msgs = [];
            if (!scanFresh) msgs.push('No lidar scan');
            if (!avatars.length) msgs.push('No people tracked');
            const persons = [...people.values()].filter(p => p.kind === 'person' && p.group.visible);
            const partial = persons.filter(p => !(p.group.userData.poseConfidence >= 0.75)).length;
            if (persons.length) msgs.push(partial
                ? `Pose uncertain: ${partial}/${persons.length} · hidden joints use animation`
                : 'Leg pose observed · depth approximated');
            if (document.getElementById('body-facing-toggle') && document.getElementById('body-facing-toggle').checked) msgs.push('Facing estimate enabled');
            if (d.c3Stats) msgs.push(`C3 ${d.c3Tracks.length} trk · ${d.c3Stats.hz} Hz · ${d.c3Stats.update_ms_avg} ms`);
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
