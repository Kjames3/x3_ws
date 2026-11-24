// =================================================================
// Reality view structure fitting (pure, no THREE): turns the voxel cells the
// Surroundings view already draws into
//   walls   : straight vertical panels fitted (seen from above) to tall voxel
//             columns, and
//   objects : furniture stand-ins, an OAK spatial detection (label + base
//             position) sized and oriented by the lidar voxels around it,
//             held in the odom frame so they do not flicker.
// Cells are base_footprint {x: fwd, y: left, z: up, s?: size} like
// SurroundingsMemory output. Everything a wall or object explains is
// reported by hidden(cell) so the voxel layer can drop it.
// Usable from node (module.exports) for tests.
// =================================================================
(function (root) {
    'use strict';
    const COL = 0.2;               // column grid (m), same as scene-view's classing
    const WALL_TOP = 1.2;          // column top >= this is structure
    const LINE_TOL = 0.09;         // RANSAC inlier distance (m)
    const MIN_INLIERS = 6;         // columns per accepted line
    const GAP_SPLIT = 0.45;        // split a line where columns are further apart
    const MIN_SEG = 0.4;           // shortest panel (m)
    const MAX_SEGS = 14;
    const OBJ_RADIUS = 0.9;        // voxels considered for a detection (m)
    const OBJ_HOLD_S = 3.0;
    const OBJ_SMOOTH = 0.3;
    const OBJ_MATCH_M = 0.7;

    // Labels from COCO / VOC style detectors -> stand-in kind.
    const KINDS = {
        chair: 'chair', bench: 'couch', couch: 'couch', sofa: 'couch',
        tv: 'tv', tvmonitor: 'tv', monitor: 'tv', laptop: 'tv',
        'dining table': 'table', diningtable: 'table', table: 'table', desk: 'table',
        bed: 'bed', 'potted plant': 'plant', pottedplant: 'plant',
        refrigerator: 'box', toilet: 'box', oven: 'box', sink: 'box',
    };
    // Plausible footprint (length, depth) and height ranges per kind (m).
    const SIZE = {
        chair: { l: [0.4, 0.7], d: [0.4, 0.7], h: [0.7, 1.1] },
        couch: { l: [1.2, 2.6], d: [0.7, 1.1], h: [0.6, 1.0] },
        tv:    { l: [0.5, 1.8], d: [0.08, 0.5], h: [0.4, 1.6] },
        table: { l: [0.6, 2.2], d: [0.5, 1.2], h: [0.4, 0.9] },
        bed:   { l: [1.8, 2.3], d: [0.9, 2.1], h: [0.4, 0.8] },
        plant: { l: [0.3, 0.8], d: [0.3, 0.8], h: [0.3, 1.6] },
        box:   { l: [0.4, 1.0], d: [0.4, 0.9], h: [0.5, 2.0] },
    };
    const clamp = (v, [a, b]) => Math.max(a, Math.min(b, v));
    const colKey = (x, y) => `${Math.floor(x / COL)},${Math.floor(y / COL)}`;

    // Deterministic PRNG so the same cloud fits the same lines (no flicker).
    function rng(seed) {
        let s = seed >>> 0 || 1;
        return () => { s ^= s << 13; s ^= s >>> 17; s ^= s << 5; return (s >>> 0) / 4294967296; };
    }

    function columns(cells) {
        const cols = new Map();
        for (const c of cells) {
            const k = colKey(c.x, c.y);
            let col = cols.get(k);
            if (!col) { col = { sx: 0, sy: 0, n: 0, top: -Infinity, bottom: Infinity, k,
                                i: Math.floor(c.x / COL), j: Math.floor(c.y / COL) }; cols.set(k, col); }
            col.sx += c.x; col.sy += c.y; col.n++;
            col.top = Math.max(col.top, c.z); col.bottom = Math.min(col.bottom, c.z);
        }
        // Mean, not grid centre: a wall straddling a grid line stays one line.
        for (const col of cols.values()) { col.x = col.sx / col.n; col.y = col.sy / col.n; }
        return cols;
    }

    function fitWalls(cols) {
        let pts = [...cols.values()].filter(c => c.top >= WALL_TOP);
        const rand = rng(pts.length * 2654435761);
        const segs = [], used = new Set();
        while (segs.length < MAX_SEGS && pts.length >= MIN_INLIERS) {
            let best = null;
            for (let it = 0; it < 160; it++) {
                const a = pts[Math.floor(rand() * pts.length)], b = pts[Math.floor(rand() * pts.length)];
                const dx = b.x - a.x, dy = b.y - a.y, len = Math.hypot(dx, dy);
                if (len < 0.3 || len > 4) continue;
                const nx = -dy / len, ny = dx / len;
                let n = 0;
                for (const p of pts) if (Math.abs((p.x - a.x) * nx + (p.y - a.y) * ny) < LINE_TOL) n++;
                if (!best || n > best.n) best = { n, a, nx, ny };
            }
            if (!best || best.n < MIN_INLIERS) break;
            const inl = pts.filter(p => Math.abs((p.x - best.a.x) * best.nx + (p.y - best.a.y) * best.ny) < LINE_TOL);
            // PCA refine.
            let mx = 0, my = 0; for (const p of inl) { mx += p.x; my += p.y; } mx /= inl.length; my /= inl.length;
            let sxx = 0, sxy = 0, syy = 0;
            for (const p of inl) { const u = p.x - mx, v = p.y - my; sxx += u * u; sxy += u * v; syy += v * v; }
            const ang = 0.5 * Math.atan2(2 * sxy, sxx - syy), dx = Math.cos(ang), dy = Math.sin(ang);
            const proj = inl.map(p => ({ p, t: (p.x - mx) * dx + (p.y - my) * dy })).sort((a, b) => a.t - b.t);
            let start = 0;
            for (let i = 1; i <= proj.length; i++) {
                if (i < proj.length && proj[i].t - proj[i - 1].t <= GAP_SPLIT) continue;
                const piece = proj.slice(start, i); start = i;
                const t0 = piece[0].t - COL / 2, t1 = piece[piece.length - 1].t + COL / 2;
                if (piece.length < 3 || t1 - t0 < MIN_SEG) continue;
                const tops = piece.map(q => q.p.top).sort((a, b) => a - b);
                segs.push({ x0: mx + dx * t0, y0: my + dy * t0, x1: mx + dx * t1, y1: my + dy * t1,
                            top: tops[Math.floor(tops.length * .8)] });
                for (const q of piece) used.add(q.p.k);
            }
            const gone = new Set(inl);
            pts = pts.filter(p => !gone.has(p));
        }
        return { segs, used };
    }

    function cluster(low, cols, p, wallUsed) {
        // Columns holding object-height cells (> 10 cm, so floor noise does not bridge).
        const occ = new Map();
        for (const q of low) {
            if (q.z < 0.1 || Math.hypot(q.x - p.x, q.y - p.y) > OBJ_RADIUS) continue;
            const k = colKey(q.x, q.y);
            if (!occ.has(k)) occ.set(k, []);
            occ.get(k).push(q);
        }
        // A lone cell is noise, not furniture.
        for (const [k, qs] of occ) if (qs.length < 2) occ.delete(k);
        let seed = null, bd = 0.45;
        for (const [k, qs] of occ) {
            const col = cols.get(k), d = Math.hypot(col.x - p.x, col.y - p.y);
            if (d < bd) { bd = d; seed = col; }
        }
        if (!seed) return [];
        const out = [], seen = new Set([seed.k]), stack = [seed];
        while (stack.length) {
            const col = stack.pop();
            out.push(...occ.get(col.k));
            for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
                const k = `${col.i + di},${col.j + dj}`;
                if (seen.has(k) || !occ.has(k) || wallUsed.has(k)) continue;
                seen.add(k); stack.push(cols.get(k));
            }
        }
        return out;
    }

    // Object tracks persist across calls (odom frame).
    function Tracker() { this.tracks = []; this.nextId = 1; }

    Tracker.prototype.update = function (cells, cols, wallUsed, dets, pose, nowS) {
        const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
        const toOdom = (x, y) => [pose.x + c * x - s * y, pose.y + s * x + c * y];
        const toBase = (X, Y) => { const dx = X - pose.x, dy = Y - pose.y; return [c * dx + s * dy, -s * dx + c * dy]; };
        const low = cells.filter(q => q.z > 0.03 && !wallUsed.has(colKey(q.x, q.y)) &&
                                     (cols.get(colKey(q.x, q.y))?.top ?? 0) < WALL_TOP + 0.4);
        for (const det of dets || []) {
            const kind = KINDS[String(det.label || '').toLowerCase()];
            const p = det.xyz_base_m;
            if (!kind || !p || !(det.conf >= 0.5) || !isFinite(p.x) || !isFinite(p.y)) continue;
            // Lidar cells of the connected low-column cluster nearest the detection
            // give footprint, heading and height.
            const near = cluster(low, cols, p, wallUsed);
            const sz = SIZE[kind];
            let cx = p.x, cy = p.y, yaw = Math.atan2(p.y, p.x) + Math.PI / 2, L = sz.l[0], D = sz.d[0], H = sz.h[0];
            if (near.length >= 4) {
                let mx = 0, my = 0; for (const q of near) { mx += q.x; my += q.y; } mx /= near.length; my /= near.length;
                let sxx = 0, sxy = 0, syy = 0, top = 0;
                for (const q of near) { const u = q.x - mx, v = q.y - my; sxx += u * u; sxy += u * v; syy += v * v; top = Math.max(top, q.z); }
                yaw = 0.5 * Math.atan2(2 * sxy, sxx - syy);
                const ux = Math.cos(yaw), uy = Math.sin(yaw);
                // 5th-95th percentile extents: one stray cell cannot stretch the box.
                const as = near.map(q => (q.x - mx) * ux + (q.y - my) * uy).sort((x, y) => x - y);
                const bs = near.map(q => -(q.x - mx) * uy + (q.y - my) * ux).sort((x, y) => x - y);
                const pct = (arr, f) => arr[Math.min(arr.length - 1, Math.floor(arr.length * f))];
                const a0 = pct(as, .05), a1 = pct(as, .95), b0 = pct(bs, .05), b1 = pct(bs, .95);
                cx = mx + ux * (a0 + a1) / 2 - uy * (b0 + b1) / 2;
                cy = my + uy * (a0 + a1) / 2 + ux * (b0 + b1) / 2;
                L = a1 - a0 + .05; D = b1 - b0 + .05; H = top + .03;
                if (D > L) { [L, D] = [D, L]; yaw += Math.PI / 2; }
            }
            L = clamp(L, sz.l); D = clamp(D, sz.d); H = clamp(H, sz.h);
            const [X, Y] = toOdom(cx, cy), Yaw = yaw + pose.theta;
            let t = this.tracks.find(t => t.kind === kind && Math.hypot(t.X - X, t.Y - Y) < OBJ_MATCH_M);
            if (!t) { t = { id: this.nextId++, kind, X, Y, yaw: Yaw, L, D, H, seen: nowS, hits: 0 }; this.tracks.push(t); }
            const k = OBJ_SMOOTH;
            t.X += k * (X - t.X); t.Y += k * (Y - t.Y);
            // Box yaw is symmetric by pi: move the shorter way.
            let dyaw = Yaw - t.yaw; dyaw = Math.atan2(Math.sin(2 * dyaw), Math.cos(2 * dyaw)) / 2;
            t.yaw += k * dyaw;
            t.L += k * (L - t.L); t.D += k * (D - t.D); t.H += k * (H - t.H);
            t.seen = nowS; t.hits++;
        }
        this.tracks = this.tracks.filter(t => nowS - t.seen < OBJ_HOLD_S);
        return this.tracks.filter(t => t.hits >= 2).map(t => {
            const [x, y] = toBase(t.X, t.Y);
            return { id: t.id, kind: t.kind, x, y, yaw: t.yaw - pose.theta, L: t.L, D: t.D, H: t.H,
                     age: nowS - t.seen };
        });
    };

    function inside(o, q, pad) {
        const dx = q.x - o.x, dy = q.y - o.y, c = Math.cos(o.yaw), s = Math.sin(o.yaw);
        return Math.abs(dx * c + dy * s) <= o.L / 2 + pad && Math.abs(-dx * s + dy * c) <= o.D / 2 + pad && q.z <= o.H + pad;
    }

    // One call per cloud: walls + objects + the hide predicate for the voxel layer.
    function analyze(cells, dets, pose, tracker, nowS) {
        const cols = columns(cells);
        const { segs, used } = fitWalls(cols);
        const objects = pose && tracker ? tracker.update(cells, cols, used, dets, pose, nowS) : [];
        return {
            walls: segs, objects,
            hidden(q) {
                if (used.has(colKey(q.x, q.y))) return true;
                for (const o of objects) if (inside(o, q, 0.06)) return true;
                return false;
            },
        };
    }

    const api = { analyze, Tracker, fitWalls, columns, KINDS, WALL_TOP };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.RealityStructure = api;
})(typeof window !== 'undefined' ? window : globalThis);
