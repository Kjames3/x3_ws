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
// Display only: nothing here feeds control.
// Scene frame (see scene-view.js): forward = -Z, right = +X, up = +Y.
// =================================================================
window.RealityMode = (() => {
    const MAX_PATH = 512;
    const RIBBON_W = 0.30;        // ~ robot width
    const RIBBON_Y = 0.012;
    const SHIELD_R = 0.36, SHIELD_H = 0.32;
    const COLORS = { ok: 0x22d3ee, limited: 0xf59e0b, blind: 0xef4444, hold: 0x9ca3af };
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
            update(enabled, d, nav, now) {
                applyTheme(enabled);
                if (!enabled) { ribbon.visible = beacon.visible = shieldGroup.visible = false; return; }
                time.value = now / 1000;
                updatePath(nav && nav.path, d.robotPoseMap);
                updateShield(d.readout && d.readout.cbf);
            },
        };
    }
    return { create };
})();
