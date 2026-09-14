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
    let renderer, scene, camera, walls, container, hint, _m, _dir, _org;
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
        container.appendChild(renderer.domElement);

        scene = new THREE.Scene();
        scene.background = new THREE.Color(0xe9ecef);
        scene.fog = new THREE.Fog(0xe9ecef, 4, 9);

        camera = new THREE.PerspectiveCamera(50, 1, 0.05, 50);
        camera.position.set(0, 2.4, 2.2);
        camera.lookAt(0, 0, -1.6);

        scene.add(new THREE.HemisphereLight(0xffffff, 0x9aa3ad, 0.9));
        const sun = new THREE.DirectionalLight(0xffffff, 0.5);
        sun.position.set(2, 5, 3);
        scene.add(sun);

        const floor = new THREE.Mesh(new THREE.PlaneGeometry(30, 30),
            new THREE.MeshLambertMaterial({ color: 0xf8f9fa }));
        floor.rotation.x = -Math.PI / 2;
        scene.add(floor);

        // 1 m range rings
        for (let r = 1; r <= RANGE_M; r++) {
            const ring = new THREE.Mesh(new THREE.RingGeometry(r - 0.008, r + 0.008, 96),
                new THREE.MeshBasicMaterial({ color: 0xc5ccd3 }));
            ring.rotation.x = -Math.PI / 2;
            ring.position.y = 0.002;
            scene.add(ring);
        }

        // Robot: body + blue nose so heading is obvious
        const robot = new THREE.Group();
        const body = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.12, 0.30),
            new THREE.MeshLambertMaterial({ color: 0x2b2f36 }));
        body.position.y = 0.08;
        const nose = new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.02, 0.05),
            new THREE.MeshLambertMaterial({ color: 0x3b82f6 }));
        nose.position.set(0, 0.15, -0.12);
        robot.add(body, nose);
        scene.add(robot);

        walls = new THREE.InstancedMesh(new THREE.BoxGeometry(0.06, WALL_H, 0.06),
            new THREE.MeshLambertMaterial({ color: 0x9ca3af }), MAX_WALL_PTS);
        walls.count = 0;
        scene.add(walls);

        new ResizeObserver(resize).observe(container);
        resize();
        requestAnimationFrame(loop);
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

    function updateWalls(pts) {
        if (!pts || pts.length < 2) { walls.count = 0; return; }
        const n = pts.length / 2;
        const step = Math.max(1, Math.ceil(n / MAX_WALL_PTS));
        let k = 0;
        for (let i = 0; i < n && k < MAX_WALL_PTS; i += step) {
            const lx = pts[2 * i], p1 = pts[2 * i + 1];
            if (!isFinite(lx) || !isFinite(p1) || lx * lx + p1 * p1 > RANGE_M * RANGE_M) continue;
            // laser (x, y=-p1) -> base_link: fwd = LASER_X - x, right = y = -p1
            const fwd = LASER_X - lx, right = -p1;
            _m.makeTranslation(right, WALL_H / 2, -fwd);
            walls.setMatrixAt(k++, _m);
        }
        walls.count = k;
        walls.instanceMatrix.needsUpdate = true;
    }

    function makePerson() {
        const group = new THREE.Group();
        const bodyMat = new THREE.MeshLambertMaterial({ color: 0xf59e0b });
        const cyl = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 1.2, 24), bodyMat);
        cyl.position.y = 0.6;
        group.add(cyl);
        const ghost = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 1.2, 24),
            new THREE.MeshLambertMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.25 }));
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
            seen.add(est.id);
            let p = people.get(est.id);
            if (!p) { p = makePerson(); people.set(est.id, p); }
            p.group.position.set(right, 0, -fwd);

            // vx=forward, vy=left (CBF convention) -> scene (x=-vy, z=-vx)
            const sx = -(est.vy || 0), sz = -(est.vx || 0);
            const speed = Math.hypot(sx, sz);
            const moving = speed > 0.15;
            p.ghost.visible = moving;
            p.arrow.visible = moving;
            if (moving) {
                p.ghost.position.set(right + sx * GHOST_S, 0.6, -fwd + sz * GHOST_S);
                _org.set(right, 1.3, -fwd);
                _dir.set(sx, 0, sz).normalize();
                p.arrow.position.copy(_org);
                p.arrow.setDirection(_dir);
                p.arrow.setLength(Math.max(0.3, speed * GHOST_S), 0.15, 0.1);
            }
        }
        for (const [id, p] of people) {
            if (seen.has(id)) continue;
            scene.remove(p.group, p.ghost, p.arrow);
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
        if (d.lidarPoints !== lastPts) { lastPts = d.lidarPoints; lastPtsAt = now; }
        const scanFresh = lastPts && now - lastPtsAt < SCAN_STALE_MS;
        try {
        updateWalls(scanFresh ? lastPts : null);
        updatePeople(d.velocityEstimates);
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
