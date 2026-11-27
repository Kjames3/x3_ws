// =================================================================
// Furniture look V2 for the Reality view (opt-in: ?furnitureLookV2=1,
// remembered in localStorage 'furnitureLookV2'; default = the legacy
// unit-box stand-ins in reality-mode.js).
//
// layout(kind, box, state) is pure (no THREE) and returns parts in METRES in
// the stand-in's local frame, so leg and top thicknesses stay true whatever
// the box size (the legacy meshes stretch a unit box by L x H x D):
//   x along L (centred), y up from the box bottom, z along D (centred),
//   BACK at -Z, open front at +Z.
// That is the legacy convention: reality-mode.js turns the group by
// yaw + pi/2, which puts local -Z on the base-frame LEFT of the box yaw
// (yaw + 90 deg). reality-structure.js fits a chair heading as
// atan2(backrest offset) - pi/2, i.e. the backrest lies at yaw + 90 deg.
//
// createRenderer(THREE) builds the meshes from shared unit geometries and
// shared shader source; per object only the transforms and three small
// material clones (opacity, fade) change. Nothing is rebuilt when the
// smoothed box drifts. Display only: nothing here feeds control.
// =================================================================
(function (root) {
    'use strict';
    const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
    const fin = Number.isFinite;
    const FADE_IN_MS = 650, FADE_OUT_MS = 400, RISE_M = 0.04;
    const SHADOW_MARGIN = 0.30;     // shadow plane extends this far past the footprint (m)

    function box(parts, x, y, z, sx, sy, sz, rx) {
        if (sx > 0.002 && sy > 0.002 && sz > 0.002)
            parts.push(rx ? {shape: 'box', x, y, z, sx, sy, sz, rx} : {shape: 'box', x, y, z, sx, sy, sz});
    }
    // Narrow cylinders (columns, collars) use the 12-segment 'post' geometry.
    const POST_DIA = 0.25, SEGS = {cyl: 24, post: 12};
    function cyl(parts, x, y, z, dia, h) {
        if (dia > 0.002 && h > 0.002)
            parts.push({shape: dia < POST_DIA ? 'post' : 'cyl', x, y, z, sx: dia, sy: h, sz: dia});
    }

    // Rectangular table: thin top with a small overhang, square legs, an
    // apron under dining/desk tops, a low shelf under coffee tables.
    function tableRect(L, D, H, low) {
        const p = [];
        const top = clamp(0.024 + 0.010 * Math.min(1, L / 1.6), 0.024, 0.036);
        const over = Math.min(0.045, 0.06 * Math.min(L, D));
        const leg = low ? 0.045 : clamp(0.034 + 0.008 * Math.min(1, L / 1.6), 0.034, 0.045);
        const lx = L / 2 - over - leg / 2, lz = D / 2 - over - leg / 2;
        const legH = H - top;
        box(p, 0, H - top / 2, 0, L, top, D);
        for (const sx of [-1, 1]) for (const sz of [-1, 1]) box(p, sx * lx, legH / 2, sz * lz, leg, legH, leg);
        if (!low && legH > 0.3) {
            const ah = 0.07, at = 0.018, ay = legH - ah / 2;
            box(p, 0, ay, -lz, 2 * lx - leg, ah, at);
            box(p, 0, ay, lz, 2 * lx - leg, ah, at);
            box(p, -lx, ay, 0, at, ah, 2 * lz - leg);
            box(p, lx, ay, 0, at, ah, 2 * lz - leg);
        } else if (low && L >= 0.7 && legH > 0.2) {
            box(p, 0, 0.11, 0, 2 * lx - leg, 0.018, 2 * lz - leg);
        }
        return p;
    }

    // Round table: disc top, collar, column, foot plate. Diameter = mean of L, D.
    function tableRound(L, D, H) {
        const p = [];
        const dia = (L + D) / 2, top = clamp(0.022 + 0.012 * Math.min(1, dia / 1.2), 0.024, 0.034);
        const foot = clamp(0.5 * dia, 0.32, 0.6), plate = 0.022;
        const col = clamp(0.07 * dia, 0.055, 0.09);
        cyl(p, 0, H - top / 2, 0, dia, top);
        cyl(p, 0, H - top - 0.02, 0, clamp(0.3 * dia, 0.16, 0.3), 0.04);
        cyl(p, 0, (plate + H - top - 0.04) / 2, 0, col, H - top - 0.04 - plate);
        cyl(p, 0, plate / 2, 0, foot, plate);
        cyl(p, 0, plate + 0.02, 0, clamp(2.2 * col, 0.12, 0.2), 0.04);
        return p;
    }

    function chairSeat(box, H) {
        // Same rule as the legacy builder: seat fraction clamped to [.30, .72] of H.
        return fin(box.seat) && box.seat > 0 ? H * clamp(box.seat / H, 0.30, 0.72) : 0.46 * H;
    }

    // Chair: seat slab, four legs, back posts continuing the rear legs, a
    // backrest panel, optional arms (wide seats; no box field says "arms").
    function chair(L, D, H, s, arms) {
        const p = [];
        const seatT = 0.045, leg = clamp(0.06 * Math.min(L, D), 0.026, 0.036), inset = 0.02;
        const lx = L / 2 - inset - leg / 2, lz = D / 2 - inset - leg / 2;
        const legH = s - seatT;
        box(p, 0, s - seatT / 2, 0.01, L, seatT, D - 0.02);
        for (const sx of [-1, 1]) for (const sz of [-1, 1]) box(p, sx * lx, legH / 2, sz * lz, leg, legH, leg);
        // Back posts from the seat to the top, then the panel between them.
        const backTop = H, postH = backTop - s;
        for (const sx of [-1, 1]) box(p, sx * lx, s + postH / 2, -lz, leg, postH, leg);
        const panelBot = s + clamp(0.3 * postH, 0.08, 0.2), panelTop = backTop - 0.012;
        box(p, 0, (panelBot + panelTop) / 2, -lz, 2 * lx - leg, panelTop - panelBot, 0.026);
        if (arms) {
            const ay = Math.min(s + 0.2, H - 0.06), ah = 0.026, aw = 0.045;
            box(p, -lx, ay + ah / 2, 0, aw, ah, 2 * lz + leg);
            box(p, lx, ay + ah / 2, 0, aw, ah, 2 * lz + leg);
            for (const sx of [-1, 1]) box(p, sx * lx, (s + ay) / 2, lz, leg, ay - s, leg);
        }
        return p;
    }

    function couchSeat(box, H) {
        return fin(box.seat) && box.seat > 0 ? clamp(box.seat, 0.3, H - 0.2) : clamp(0.52 * H, 0.36, 0.48);
    }

    // Couch: short feet, plinth between two armrests, a back frame, seat
    // cushions split along the length and matching back cushions leaning on it.
    function couch(L, D, H, s, n) {
        const p = [];
        const foot = Math.min(0.07, 0.08 * H), footW = 0.05;
        const armW = clamp(0.085 * L, 0.10, 0.18), armH = Math.min(H - 0.08, s + 0.2);
        const backT = clamp(0.16 * D, 0.11, 0.2), cushT = Math.min(0.13, 0.6 * (s - foot));
        const baseTop = s - cushT, Wi = L - 2 * armW;
        for (const sx of [-1, 1]) for (const sz of [-1, 1])
            box(p, sx * (L / 2 - 0.05), foot / 2, sz * (D / 2 - 0.05), footW, foot, footW);
        box(p, 0, (foot + baseTop) / 2, 0, Wi, baseTop - foot, D);
        for (const sx of [-1, 1]) box(p, sx * (L / 2 - armW / 2), (foot + armH) / 2, 0, armW, armH - foot, D);
        const frameTop = H - 0.07;
        box(p, 0, (baseTop + frameTop) / 2, -D / 2 + backT / 2, Wi, frameTop - baseTop, backT);
        const gap = 0.012, cw = (Wi - (n + 1) * gap) / n;
        const bc = clamp(0.17 * D, 0.11, 0.17), lean = 0.12;
        const backH = H - 0.01 - s, bz = -D / 2 + backT + bc / 2 + Math.sin(lean) * backH / 2;
        const seatZ0 = -D / 2 + backT + 0.6 * bc, seatZ1 = D / 2 - 0.015;
        for (let i = 0; i < n; i++) {
            const cx = -Wi / 2 + gap + cw / 2 + i * (cw + gap);
            box(p, cx, (baseTop + s) / 2, (seatZ0 + seatZ1) / 2, cw, s - baseTop, seatZ1 - seatZ0);
            box(p, cx, s + backH / 2, bz, cw, backH * Math.cos(lean), bc, -lean);
        }
        return p;
    }

    // Choices that would flicker at a threshold keep the last answer (state).
    function couchCushions(Wi, prev) {
        const edges = [1.0, 1.65], hyst = 0.06;
        let n = Wi < edges[0] ? 1 : Wi < edges[1] ? 2 : 3;
        if (prev && prev !== n) {
            const e = edges[Math.min(prev, n) - 1];
            if (Math.abs(Wi - e) < hyst) n = prev;
        }
        return n;
    }
    function chairArms(L, prev) {
        if (prev === true) return L >= 0.56;
        return L >= 0.61;
    }

    // Legacy unit-box parts (x, y, z, sx, sy, sz as fractions of L, H, D) for
    // kinds without a dedicated V2 layout; scaled to metres here.
    function scaledLegacy(unitParts, L, D, H) {
        return (unitParts || [[0, .5, 0, 1, 1, 1]]).map(([x, y, z, sx, sy, sz]) =>
            ({shape: 'box', x: x * L, y: y * H, z: z * D, sx: sx * L, sy: sy * H, sz: sz * D}));
    }

    // kind: chair | couch | table (others fall back to `legacy[kind]`).
    // box: {L, D, H, seat?, round?, low?}. state: per-object memory, mutated.
    function layout(kind, b, state = {}, legacy = {}) {
        const L = Math.max(0.05, b.L), D = Math.max(0.05, b.D), H = Math.max(0.1, b.H);
        if (kind === 'table') {
            const low = !!b.low || H < 0.55;
            return b.round ? tableRound(L, D, H) : tableRect(L, D, H, low);
        }
        if (kind === 'chair') {
            state.arms = chairArms(L, state.arms);
            return chair(L, D, H, chairSeat(b, H), state.arms);
        }
        if (kind === 'couch') {
            const armW = clamp(0.085 * L, 0.10, 0.18);
            state.cushions = couchCushions(L - 2 * armW, state.cushions);
            return couch(L, D, H, couchSeat(b, H), state.cushions);
        }
        return scaledLegacy(legacy[kind] || legacy.box, L, D, H);
    }

    function triangles(parts) {
        // BoxGeometry: 12; CylinderGeometry(radial=n): 2n side + 2n caps.
        return parts.reduce((n, p) => n + (SEGS[p.shape] ? 4 * SEGS[p.shape] : 12), 0);
    }

    function flagEnabled() {
        try {
            const q = new URLSearchParams(root.location ? root.location.search : '').get('furnitureLookV2');
            if (q !== null) {
                const on = q === '1' || q === 'true';
                try { root.localStorage.setItem('furnitureLookV2', on ? '1' : '0'); } catch (_) { /* session only */ }
                return on;
            }
        } catch (_) { /* no URL */ }
        try { return root.localStorage.getItem('furnitureLookV2') === '1'; } catch (_) { return false; }
    }

    // ---- rendering ---------------------------------------------------------
    const BODY_VERT = `
        #include <fog_pars_vertex>
        varying vec3 vLocal; varying vec3 vSize; varying vec3 vN; varying vec3 vW;
        void main() {
            vLocal = position;
            vSize = vec3(length(modelMatrix[0].xyz), length(modelMatrix[1].xyz), length(modelMatrix[2].xyz));
            vec4 w = modelMatrix * vec4(position, 1.0);
            vW = w.xyz;
            vN = normalize(mat3(modelMatrix) * (normal / (vSize * vSize)));
            vec4 mvPosition = viewMatrix * w;
            gl_Position = projectionMatrix * mvPosition;
            #include <fog_vertex>
        }`;
    // Untextured light grey. Soft key + hemisphere fill, darker toward the
    // floor (cheap AO), a fresnel rim, and a thin metric edge highlight that
    // stands in for a bevel: distance to the nearest edge is computed in
    // metres from the part's scale, so every edge gets the same ~6 mm line.
    const BODY_FRAG = `
        uniform vec3 base; uniform float opacity;
        #include <fog_pars_fragment>
        varying vec3 vLocal; varying vec3 vSize; varying vec3 vN; varying vec3 vW;
        void main() {
            vec3 n = normalize(vN);
            vec3 v = normalize(cameraPosition - vW);
            vec3 d = (0.5 - abs(vLocal)) * vSize;
        #ifdef CYL
            float e = max((0.5 - length(vLocal.xz)) * vSize.x, d.y);
        #else
            float e = d.x + d.y + d.z - min(d.x, min(d.y, d.z)) - max(d.x, max(d.y, d.z));
        #endif
            float aa = fwidth(e) + 1e-4;
            float line = 1.0 - smoothstep(0.006 - aa, 0.006 + aa, e);
            float glow = 1.0 - smoothstep(0.0, 0.03, e);
            float hemi = 0.5 + 0.5 * n.y;
            float key = max(dot(n, normalize(vec3(-0.45, 0.85, 0.35))), 0.0);
            float fill = max(dot(n, normalize(vec3(0.6, 0.25, -0.55))), 0.0);
            float ao = mix(0.55, 1.0, smoothstep(0.0, 0.3, vW.y));
            float rim = pow(1.0 - max(dot(n, v), 0.0), 3.0);
            vec3 col = base * (0.36 + 0.26 * hemi + 0.38 * key + 0.14 * fill) * ao;
            col += vec3(0.78, 0.82, 0.88) * (0.20 * rim + 0.12 * glow + 0.30 * line);
            gl_FragColor = vec4(col, opacity);
            #include <fog_fragment>
        }`;
    // Soft contact shadow: rounded-rect (or disc) distance field in metres
    // from the plane's scale, a wide penumbra plus a darker core.
    const SHADOW_VERT = `
        varying vec2 vP; varying vec2 vHalf;
        void main() {
            vec2 size = vec2(length(modelMatrix[0].xyz), length(modelMatrix[1].xyz));
            vP = position.xy * size;
            vHalf = max(size * 0.5 - ${SHADOW_MARGIN.toFixed(3)}, vec2(0.02));
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }`;
    const SHADOW_FRAG = `
        uniform float opacity; uniform float round; uniform float strength;
        varying vec2 vP; varying vec2 vHalf;
        void main() {
            float dist;
            if (round > 0.5) dist = length(vP) - vHalf.x;
            else {
                vec2 q = abs(vP) - vHalf + 0.06;
                dist = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - 0.06;
            }
            // A dark blob alone vanishes on the near-black floor, so the
            // footprint also gets a faint light halo (bounce light) outside it.
            float core = (1.0 - smoothstep(-0.30, 0.04, dist)) * 0.55 * strength;
            float halo = (1.0 - smoothstep(0.0, ${SHADOW_MARGIN.toFixed(3)} - 0.02, dist))
                       * smoothstep(-0.10, 0.02, dist) * 0.10;
            float a = core + halo;
            gl_FragColor = vec4(vec3(0.82, 0.86, 0.91) * halo / max(a, 1e-4), a * opacity);
        }`;

    function createRenderer(THREE) {
        const unitBox = new THREE.BoxGeometry(1, 1, 1);
        const unitCyl = new THREE.CylinderGeometry(0.5, 0.5, 1, SEGS.cyl, 1);
        const unitPost = new THREE.CylinderGeometry(0.5, 0.5, 1, SEGS.post, 1);
        const GEOMETRY = {box: unitBox, cyl: unitCyl, post: unitPost};
        const unitPlane = new THREE.PlaneGeometry(1, 1);
        function bodyMaterial(cylinder) {
            return new THREE.ShaderMaterial({
                uniforms: THREE.UniformsUtils.merge([THREE.UniformsLib.fog,
                    {base: {value: new THREE.Color(0xdbe0e6)}, opacity: {value: 1}}]),
                vertexShader: BODY_VERT, fragmentShader: BODY_FRAG,
                defines: cylinder ? {CYL: 1} : {}, fog: true, extensions: {derivatives: true},
            });
        }
        const proto = {
            box: bodyMaterial(false), cyl: bodyMaterial(true),
            shadow: new THREE.ShaderMaterial({
                uniforms: {opacity: {value: 1}, round: {value: 0}, strength: {value: 1}},
                vertexShader: SHADOW_VERT, fragmentShader: SHADOW_FRAG,
                transparent: true, depthWrite: false,
            }),
        };
        const fading = new Set();

        // Group (yaw, position) > inner (lookalike toggles its children)
        //   > body (part meshes) + ring (contact shadow).
        function make(kind, scene, nowMs) {
            const group = new THREE.Group(), inner = new THREE.Group(), body = new THREE.Group();
            // Clones share the compiled programs; each carries its own opacity.
            const mats = {box: proto.box.clone(), cyl: proto.cyl.clone(), shadow: proto.shadow.clone()};
            const ring = new THREE.Mesh(unitPlane, mats.shadow);
            ring.rotation.x = -Math.PI / 2;
            ring.renderOrder = -1;
            inner.add(body, ring);
            group.add(inner);
            scene.add(group);
            const ob = {v2: true, kind, group, inner, body, ring, mats, meshes: [], state: {},
                        born: nowMs, fade: 0, dying: 0};
            setFade(ob, 0);
            fading.add(ob);
            return ob;
        }

        function fit(ob, b, legacy) {
            const parts = layout(ob.kind, b, ob.state, legacy);
            parts.forEach((p, i) => {
                let m = ob.meshes[i];
                if (!m) { m = new THREE.Mesh(unitBox, ob.mats.box); ob.body.add(m); ob.meshes.push(m); }
                m.geometry = GEOMETRY[p.shape];
                m.material = p.shape === 'box' ? ob.mats.box : ob.mats.cyl;
                m.position.set(p.x, p.y, p.z);
                m.scale.set(p.sx, p.sy, p.sz);
                m.rotation.set(p.rx || 0, 0, 0);
                m.visible = true;
            });
            for (let i = parts.length; i < ob.meshes.length; i++) ob.meshes[i].visible = false;
            const round = ob.kind === 'table' && !!b.round;
            const L = Math.max(0.05, b.L), D = Math.max(0.05, b.D);
            const w = round ? (L + D) / 2 : L, d = round ? (L + D) / 2 : D;
            ob.ring.scale.set(w + 2 * SHADOW_MARGIN, d + 2 * SHADOW_MARGIN, 1);
            ob.ring.position.set(0, 0.002 - (b.bottom || 0), 0);
            ob.mats.shadow.uniforms.round.value = round ? 1 : 0;
            // Open furniture (legs) casts a lighter blob than a solid couch.
            ob.mats.shadow.uniforms.strength.value = ob.kind === 'couch' || ob.kind === 'cabinet' ? 1 : 0.75;
            ob.parts = parts;
        }

        function setFade(ob, a) {
            ob.fade = a;
            const solid = a >= 1;
            for (const k of ['box', 'cyl']) {
                ob.mats[k].uniforms.opacity.value = a;
                ob.mats[k].transparent = !solid;
            }
            ob.mats.shadow.uniforms.opacity.value = a;
            ob.body.position.y = -RISE_M * (1 - a);
        }

        function dispose(ob, scene) {
            scene.remove(ob.group);
            for (const m of Object.values(ob.mats)) m.dispose();
            if (ob.onDispose) ob.onDispose();
            fading.delete(ob);
        }

        // Start a fade-out; `instant` removes now (view switched off).
        function retire(ob, scene, nowMs, instant) {
            if (instant) { dispose(ob, scene); return; }
            ob.dying = nowMs;
            ob.fadeFrom = ob.fade;
            fading.add(ob);
        }

        const ease = t => t * t * (3 - 2 * t);
        function tick(scene, nowMs) {
            for (const ob of fading) {
                if (ob.dying) {
                    const t = clamp((nowMs - ob.dying) / FADE_OUT_MS, 0, 1);
                    setFade(ob, ob.fadeFrom * (1 - ease(t)));
                    if (t >= 1) dispose(ob, scene);
                } else {
                    const t = clamp((nowMs - ob.born) / FADE_IN_MS, 0, 1);
                    setFade(ob, ease(t));
                    if (t >= 1) fading.delete(ob);
                }
            }
        }

        return {make, fit, retire, tick, dispose, fadingCount: () => fading.size};
    }

    const api = {layout, triangles, flagEnabled, createRenderer, chairSeat, couchSeat,
                 FADE_IN_MS, FADE_OUT_MS, SHADOW_MARGIN};
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.RealityFurniture = api;
})(typeof window !== 'undefined' ? window : globalThis);
