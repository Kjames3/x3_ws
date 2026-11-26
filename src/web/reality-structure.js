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
    const Lookalike = root.RealityLookalike || (typeof require === 'function' && require('./reality-lookalike'));
    const COL = 0.2;               // column grid (m), same as scene-view's classing
    const WALL_TOP = 1.2;          // column top >= this is structure
    const LINE_TOL = 0.09;         // RANSAC inlier distance (m)
    const MIN_INLIERS = 6;         // columns per accepted line
    const GAP_SPLIT = 0.45;        // split a line where columns are further apart
    const MIN_SEG = 0.6;           // shortest panel (m)
    const MAX_SEGS = 14;
    const MIN_FILL = 0.55;         // occupied columns / columns a full wall of that length has
    const SNAP_DEG = 15;           // snap to the room's square grid within this
    const MERGE_OFFSET = 0.4;      // parallel panels closer than this merge ...
    const MERGE_GAP = 0.3;         // ... when their extents overlap or nearly touch
    const SIDE_OFFSET = [0.25, 0.55];  // strip each side of a wall (m) ...
    const SIDE_MAX_FILL = 0.5;     // ... at least one must be this empty (else: inside a blob)
    const OBJ_RADIUS = 0.9;        // voxels considered for a detection (m)
    const OBJ_HOLD_S = 3.0;
    const OBJ_SMOOTH = 0.3;
    const OBJ_MATCH_M = 0.7;
    const LIDAR_CONFIRM_HITS = 10, LIDAR_CONFIRM_S = 3.0, INFERRED_GAP_S = 4.0;   // promote a lidar-only surface

    // Labels from COCO / VOC style detectors -> stand-in kind.
    const KINDS = {
        chair: 'chair', bench: 'couch', couch: 'couch', sofa: 'couch',
        tv: 'tv', tvmonitor: 'tv', monitor: 'tv',
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

    function columns(cells, heightBands = false) {
        const cols = new Map();
        for (const c of cells) {
            const k = colKey(c.x, c.y);
            let col = cols.get(k);
            if (!col) { col = { sx: 0, sy: 0, n: 0, top: -Infinity, bottom: Infinity, k,
                                i: Math.floor(c.x / COL), j: Math.floor(c.y / COL) }; cols.set(k, col); }
            if (heightBands) col.bands = (col.bands || 0) | (1 << Math.floor(c.z / .3));
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
                if (piece.length < 4 || t1 - t0 < MIN_SEG) continue;
                // Sparse lines strung through clutter: reject on fill. A diagonal
                // run of columns is ~1/sqrt2 as dense as an axis one, so measure
                // the expected count along the larger grid axis.
                const expect = (t1 - t0) * Math.max(Math.abs(dx), Math.abs(dy)) / COL;
                const fill = piece.length / Math.max(1, expect);
                if (fill < MIN_FILL) continue;
                const tops = piece.map(q => q.p.top).sort((a, b) => a - b);
                segs.push({ x0: mx + dx * t0, y0: my + dy * t0, x1: mx + dx * t1, y1: my + dy * t1,
                            top: tops[Math.floor(tops.length * .8)], fill, keys: piece.map(q => q.p.k) });
            }
            const gone = new Set(inl);
            pts = pts.filter(p => !gone.has(p));
        }
        // Square-grid snap: dominant direction mod 90 deg from the length-weighted
        // mean of 4*angle; walls within SNAP_DEG rotate onto it about their midpoint.
        // Re-estimated twice from walls within 8 deg of the previous estimate,
        // so one clutter line cannot tilt the whole grid. Weight = length * fill^2.
        // Seeded from the strongest single wall (longest x fill^2).
        let dom = null, bestW = 0;
        for (const g of segs) {
            const wgt = Math.hypot(g.x1 - g.x0, g.y1 - g.y0) * g.fill * g.fill;
            if (wgt > bestW) { bestW = wgt; dom = Math.atan2(g.y1 - g.y0, g.x1 - g.x0); }
        }
        for (let pass = 0; pass < 2; pass++) {
            let cx = 0, cy = 0;
            for (const g of segs) {
                const a = Math.atan2(g.y1 - g.y0, g.x1 - g.x0), L = Math.hypot(g.x1 - g.x0, g.y1 - g.y0);
                if (dom !== null && Math.abs(Math.atan2(Math.sin(4 * (a - dom)), Math.cos(4 * (a - dom))) / 4) > 8 * Math.PI / 180) continue;
                const wgt = L * g.fill * g.fill;
                cx += wgt * Math.cos(4 * a); cy += wgt * Math.sin(4 * a);
            }
            if (cx || cy) dom = Math.atan2(cy, cx) / 4;
        }
        if (dom === null) dom = 0;
        const out = [];
        for (const g of segs) {
            const a = Math.atan2(g.y1 - g.y0, g.x1 - g.x0), L = Math.hypot(g.x1 - g.x0, g.y1 - g.y0);
            const off = Math.atan2(Math.sin(4 * (a - dom)), Math.cos(4 * (a - dom))) / 4;
            if (Math.abs(off) <= SNAP_DEG * Math.PI / 180) {
                const na = a - off, mx = (g.x0 + g.x1) / 2, my = (g.y0 + g.y1) / 2;
                g.x0 = mx - Math.cos(na) * L / 2; g.y0 = my - Math.sin(na) * L / 2;
                g.x1 = mx + Math.cos(na) * L / 2; g.y1 = my + Math.sin(na) * L / 2;
            } else continue;   // rooms here are square; off-grid lines were clutter
            out.push(g);
        }
        // Open-side test: a wall has free space on at least one side; a line
        // through a dense blob (bed, shelving, piled furniture) has neither.
        const tall = new Set([...cols.values()].filter(c => c.top >= WALL_TOP).map(c => c.k));
        const walls = out.filter(g => {
            const L = Math.hypot(g.x1 - g.x0, g.y1 - g.y0), ux = (g.x1 - g.x0) / L, uy = (g.y1 - g.y0) / L;
            const side = sgn => {
                let hit = 0, n = 0;
                for (let t = COL / 2; t < L; t += COL) for (let o = SIDE_OFFSET[0]; o <= SIDE_OFFSET[1]; o += COL) {
                    const x = g.x0 + ux * t - uy * o * sgn, y = g.y0 + uy * t + ux * o * sgn;
                    n++; if (tall.has(colKey(x, y))) hit++;
                }
                return n ? hit / n : 0;
            };
            return Math.min(side(1), side(-1)) <= SIDE_MAX_FILL;
        });
        // Merge parallel neighbours (thick or noisy walls fit as 2-3 lines).
        const merged = [];
        for (const g of walls.sort((a, b) => Math.hypot(b.x1 - b.x0, b.y1 - b.y0) - Math.hypot(a.x1 - a.x0, a.y1 - a.y0))) {
            const L = Math.hypot(g.x1 - g.x0, g.y1 - g.y0), ux = (g.x1 - g.x0) / L, uy = (g.y1 - g.y0) / L;
            const host = merged.find(m => {
                if (Math.abs(m.ux * uy - m.uy * ux) > 0.09) return false;         // > ~5 deg apart
                const off = (g.x0 + g.x1) / 2 * -m.uy + (g.y0 + g.y1) / 2 * m.ux - m.off;
                if (Math.abs(off) > MERGE_OFFSET) return false;
                const a = g.x0 * m.ux + g.y0 * m.uy, b = g.x1 * m.ux + g.y1 * m.uy;
                return Math.min(a, b) <= m.t1 + MERGE_GAP && Math.max(a, b) >= m.t0 - MERGE_GAP;
            });
            if (!host) {
                const t0 = g.x0 * ux + g.y0 * uy;
                merged.push({ ux, uy, off: g.x0 * -uy + g.y0 * ux, t0, t1: t0 + L, w: L, top: g.top, keys: [...g.keys] });
                continue;
            }
            const a = g.x0 * host.ux + g.y0 * host.uy, b = g.x1 * host.ux + g.y1 * host.uy;
            const off = (g.x0 + g.x1) / 2 * -host.uy + (g.y0 + g.y1) / 2 * host.ux;
            host.off = (host.off * host.w + off * L) / (host.w + L); host.w += L;
            host.t0 = Math.min(host.t0, a, b); host.t1 = Math.max(host.t1, a, b);
            host.top = Math.max(host.top, g.top); host.keys.push(...g.keys);
        }
        const fitted = merged.map(m => {
            for (const k of m.keys) used.add(k);
            const px = -m.uy * m.off, py = m.ux * m.off;   // foot of the line
            return { x0: px + m.ux * m.t0, y0: py + m.uy * m.t0, x1: px + m.ux * m.t1, y1: py + m.uy * m.t1, top: m.top };
        });
        return { segs: fitted, used };
    }

    // A vertical plane is a line in plan view. Each supported column votes
    // once for its perpendicular distance at each normal angle. Refine each
    // contiguous run separately so distant collinear surfaces cannot steer it.
    function houghWalls(columns) {
        const points = [...columns.values()].filter(p => p.top >= WALL_TOP);
        if (points.length < 7) return [];
        const peaks = [];
        for (let degrees = 0; degrees < 180; degrees += 2) {
            const angle = degrees * Math.PI / 180;
            const nx = Math.cos(angle), ny = Math.sin(angle);
            const bins = new Map();
            for (const p of points) {
                const bin = Math.round((p.x*nx + p.y*ny) / .10);
                bins.set(bin, (bins.get(bin) || 0) + 1);
            }
            for (const [bin, votes] of bins) {
                if (votes >= 7 && votes >= (bins.get(bin-1) || 0) && votes >= (bins.get(bin+1) || 0))
                    peaks.push({nx, ny, offset: bin*.10, votes});
            }
        }
        peaks.sort((a,b) => b.votes-a.votes);
        const used = new Set(), walls = [];
        for (const {nx,ny,offset} of peaks) {
            const support = points.filter(p => !used.has(p) && Math.abs(nx*p.x+ny*p.y-offset) <= .075);
            if (support.length < 7) continue;
            const along = p => -ny*p.x+nx*p.y;
            support.sort((a,b) => along(a)-along(b));
            let first = 0;
            for (let end=1; end<=support.length; end++) {
                if (end<support.length && along(support[end])-along(support[end-1]) < .38) continue;
                const run = support.slice(first,end);
                first=end;
                if (run.length < 7) continue;
                let mx=0, my=0;
                for (const p of run) { mx+=p.x; my+=p.y; }
                mx/=run.length; my/=run.length;
                let xx=0, yy=0, xy=0;
                for (const p of run) {
                    const x=p.x-mx, y=p.y-my;
                    xx+=x*x; yy+=y*y; xy+=x*y;
                }
                const heading=.5*Math.atan2(2*xy,xx-yy);
                const dx=Math.cos(heading), dy=Math.sin(heading);
                let lo=Infinity, hi=-Infinity, squaredError=0;
                for (const p of run) {
                    const x=p.x-mx, y=p.y-my, t=x*dx+y*dy;
                    lo=Math.min(lo,t); hi=Math.max(hi,t);
                    squaredError+=(y*dx-x*dy)**2;
                }
                if (hi-lo < 1.1 || Math.sqrt(squaredError/run.length) > .042) continue;
                // Count occupied intervals, so clumps separated by repeated gaps
                // cannot form a single convincing plane.
                const intervals = new Set(run.map(p => Math.floor(((p.x-mx)*dx+(p.y-my)*dy-lo)/COL)));
                if (intervals.size < .8*(hi-lo)/COL) continue;
                walls.push({x0:mx+lo*dx,y0:my+lo*dy,x1:mx+hi*dx,y1:my+hi*dy,
                            top: Math.max(...run.map(p=>p.top))});
                for (const p of run) used.add(p);
                if (walls.length >= MAX_SEGS) return walls;
            }
        }
        return walls;
    }

    // A doorway candidate needs facing ends on one wall line and measured rays
    // crossing the opening. Missing points alone can also mean an occlusion.
    function doorwayGap(a, b) {
        const dx = a.x1-a.x0, dy = a.y1-a.y0, len = Math.hypot(dx,dy);
        const u = dx/len, v = dy/len;
        const bx = b.x1-b.x0, by = b.y1-b.y0;
        if (Math.abs(u*by-v*bx) > .13*Math.hypot(bx,by)) return null;
        const project = (x,y) => (x-a.x0)*u+(y-a.y0)*v;
        const t0 = project(b.x0,b.y0), t1 = project(b.x1,b.y1);
        const right = Math.min(t0,t1) > len;
        if (!right && Math.max(t0,t1) >= 0) return null;
        const nearStart = right ? t0<t1 : t0>t1;
        const x0 = right ? a.x1 : a.x0, y0 = right ? a.y1 : a.y0;
        const x1 = nearStart ? b.x0 : b.x1, y1 = nearStart ? b.y0 : b.y1;
        const width = Math.hypot(x1-x0,y1-y0);
        if (width < .65 || width > 1.45 || Math.abs((y1-y0)*u-(x1-x0)*v) > .12) return null;
        return {x0,y0,x1,y1,top:Math.min(a.top,b.top,2.1)};
    }

    function openingEvidence(g, xyz) {
        const dx=g.x1-g.x0, dy=g.y1-g.y0, width=Math.hypot(dx,dy);
        const u=dx/width, v=dy/width;
        // Livox origin is within a centimetre of the base centre in plan view.
        const plane=g.y0*u-g.x0*v;
        if (Math.abs(plane)<.15) return 0;
        let bins=0, blocked=0;
        for (let i=0;i<xyz.length;i+=3) {
            const x=xyz[i], y=xyz[i+1], z=xyz[i+2];
            const normal=y*u-x*v;
            if (Math.abs(normal)<.01) continue;
            const f=plane/normal;
            if (f<=0) continue;
            const along=(f*x-g.x0)*u+(f*y-g.y0)*v;
            const height=.273+f*(z-.273);
            if (along<.12 || along>width-.12 || height<.3 || height>1.65) continue;
            if (Math.abs(normal-plane)<.10) blocked++;
            if (f<1 && Math.abs(normal-plane)>.25) bins |= 1 << Math.floor(4*along/width);
        }
        if (blocked>=4) return -1;
        return (bins & (bins-1)) ? 1 : 0;
    }

    // Wall evidence lives in odom. Each scan votes once per floor column;
    // vertically separated returns reject tabletops, floor and ceiling patches.
    class WallHistory {
        constructor() {
            this.scans = [];
            this.tracks = [];
            this.nextWallId = 0;
            this.openings = new Map();
        }

        update(xyz, pose, time) {
            if (!pose || ![pose.x, pose.y, pose.yaw, time].every(Number.isFinite)) return [];
            const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
            const previous = this.scans.at(-1);
            if (previous && (time - previous.time > 2 || time < previous.time ||
                Math.hypot(previous.pose.x-pose.x, previous.pose.y-pose.y) > .8 ||
                Math.abs(Math.atan2(Math.sin(pose.yaw-previous.pose.yaw), Math.cos(pose.yaw-previous.pose.yaw))) > 1)) {
                this.scans = [];
                this.tracks = [];
            }
            if (this.scans.at(-1)?.cloud !== xyz) {
                const samples = [];
                for (let i = 0; i < xyz.length; i += 3) {
                    const x = xyz[i], y = xyz[i+1], z = xyz[i+2];
                    if (z < .25 || z > 2.2 || !Number.isFinite(x+y+z)) continue;
                    samples.push({x: pose.x+c*x-s*y, y: pose.y+s*x+c*y, z});
                }
                this.scans.push({time, pose, cloud: xyz, cols: columns(samples,true)});
                this.scans = this.scans.filter(scan => time-scan.time < 2.5).slice(-14);
                const evidence = new Map();
                for (const scan of this.scans) for (const col of scan.cols.values()) {
                    const e = evidence.get(col.k);
                    if (!e) evidence.set(col.k, {...col, x:col.x, y:col.y, votes:1});
                    else {
                        e.bands |= col.bands;
                        e.x += col.x; e.y += col.y; e.votes++;
                        e.bottom = Math.min(e.bottom,col.bottom); e.top = Math.max(e.top,col.top);
                    }
                }
                for (const [key,e] of evidence) {
                    // At least three height bands. Two horizontal shelves alone
                    // can have a large span while providing no vertical face.
                    const bandsAfterTwo = e.bands & (e.bands-1);
                    if (!(bandsAfterTwo & (bandsAfterTwo-1)) || e.votes < 3 || e.top-e.bottom < .65) evidence.delete(key);
                    else { e.x /= e.votes; e.y /= e.votes; }
                }
                const segs = houghWalls(evidence);
                const matched = new Set();
                for (const g of segs) {
                    let best = null, score = Infinity;
                    for (const t of this.tracks) {
                        if (matched.has(t)) continue;
                        const dx=t.x1-t.x0, dy=t.y1-t.y0, len=Math.hypot(dx,dy);
                        const u=dx/len, v=dy/len;
                        const a=(g.x0-t.x0)*u+(g.y0-t.y0)*v, b=(g.x1-t.x0)*u+(g.y1-t.y0)*v;
                        const normal = Math.max(Math.abs((g.y0-t.y0)*u-(g.x0-t.x0)*v),
                                                Math.abs((g.y1-t.y0)*u-(g.x1-t.x0)*v));
                        const overlap = Math.min(len,Math.max(a,b))-Math.max(0,Math.min(a,b));
                        if (normal > .12 || overlap < .35 || normal >= score) continue;
                        best = t; score = normal;
                    }
                    if (best) {
                        const direct = (g.x1-g.x0)*(best.x1-best.x0)+(g.y1-g.y0)*(best.y1-best.y0) >= 0;
                        for (const axis of ['x','y']) for (const end of [0,1]) {
                            const k = axis+end, target = g[axis+(direct ? end : 1-end)];
                            best[k] += .18*(target-best[k]);
                        }
                        best.top += .18*(g.top-best.top); best.hits++; best.seen=time;
                    } else {
                        best = {...g,id:this.nextWallId++,hits:1,seen:time}; this.tracks.push(best);
                    }
                    matched.add(best);
                }
                this.tracks = this.tracks.filter(t => time-t.seen < (t.hits >= 3 ? 3 : .6));
            }
            return this.tracks.filter(t => t.hits >= 3).map(t => {
                const wall = {top:t.top,id:t.id};
                for (const end of [0,1]) {
                    const dx=t['x'+end]-pose.x, dy=t['y'+end]-pose.y;
                    wall['x'+end]=c*dx+s*dy; wall['y'+end]=-s*dx+c*dy;
                }
                return wall;
            });
        }
        doorways(walls, xyz, time) {
            const visible = [], candidates = new Set();
            for (let i=0;i<walls.length;i++) for (let j=i+1;j<walls.length;j++) {
                const gap = doorwayGap(walls[i], walls[j]);
                if (!gap) continue;
                const key = [walls[i].id,walls[j].id].sort((a,b)=>a-b).join('/');
                candidates.add(key);
                const track = this.openings.get(key) || {votes:0, seen:-Infinity};
                if (track.cloud !== xyz) {
                    if (time-track.seen>2) track.votes=0;
                    const evidence = openingEvidence(gap,xyz);
                    if (evidence>0) { track.votes++; track.seen=time; }
                    if (evidence<0) track.votes=0;
                    track.cloud=xyz;
                    this.openings.set(key,track);
                }
                if (track.votes>=2 && time-track.seen<=2) visible.push(gap);
            }
            for (const key of this.openings.keys()) if (!candidates.has(key)) this.openings.delete(key);
            return visible;
        }

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

    // Fit separate seat/tabletop layers so a chair tucked under a desk can
    // contribute to both models without joining their footprints.
    // The separating-axis test uses both footprints' edge normals. Ignore
    // a few centimetres of edge contact from voxel quantization.
    function furnitureOverlap(a,b) {
        if (a.kind!==b.kind || !['table','chair'].includes(a.kind)) return false;
        for (const heading of [a.yaw,b.yaw]) for (const axis of [heading,heading+Math.PI/2]) {
            const radius = o => (o.L*Math.abs(Math.cos(o.yaw-axis)) + o.D*Math.abs(Math.sin(o.yaw-axis)))/2;
            const separation = Math.abs((a.x-b.x)*Math.cos(axis)+(a.y-b.y)*Math.sin(axis));
            if (separation >= radius(a)+radius(b)-.055) return false;
        }
        return true;
    }

    function surfaceBounds(patch,step,cx,cy,yaw) {
        const c=Math.cos(yaw),s=Math.sin(yaw);
        let lo=Infinity,hi=-Infinity,bot=Infinity,top=-Infinity;
        for (const col of patch) {
            const x=(col.i+.5)*step-cx, y=(col.j+.5)*step-cy;
            const a=c*x+s*y,b=-s*x+c*y;
            lo=Math.min(lo,a); hi=Math.max(hi,a);
            bot=Math.min(bot,b); top=Math.max(top,b);
        }
        return {yaw,L:hi-lo+step,D:top-bot+step,
            x:cx+c*(lo+hi)/2-s*(bot+top)/2,y:cy+s*(lo+hi)/2+c*(bot+top)/2};
    }

    // Top extents from the surface points themselves (not 12 cm cells), in the
    // fitted frame, trimmed 2% each side so one stray return cannot stretch it.
    function pointBounds(patch, level, rect, keepYaw = false) {
        const c=Math.cos(rect.yaw), s=Math.sin(rect.yaw), as=[], bs=[];
        for (const col of patch) for (const p of col.pts) {
            if (Math.abs(p.z-level)>=.033) continue;
            as.push(c*p.x+s*p.y); bs.push(-s*p.x+c*p.y);
        }
        if (as.length < 8) return rect;
        const ua=as.slice(), ub=bs.slice();
        as.sort((a,b)=>a-b); bs.sort((a,b)=>a-b);
        const q=(v,f)=>v[Math.min(v.length-1,Math.floor(f*v.length))];
        const a0=q(as,.02),a1=q(as,.98),b0=q(bs,.02),b1=q(bs,.98);
        const L=a1-a0+.03, D=b1-b0+.03, ma=(a0+a1)/2, mb=(b0+b1)/2;
        // Share of top points in the box corners (outer 15% on both axes):
        // ~9% for a full rectangle, ~0 for a round top.
        let corner=0;
        for (let i=0;i<ua.length;i++) {
            if (Math.abs(ua[i]-ma)>.35*L && Math.abs(ub[i]-mb)>.35*D) corner++;
        }
        const out={yaw:rect.yaw,L,D,x:c*ma-s*mb,y:s*ma+c*mb,corner:corner/as.length};
        if (!keepYaw && out.D>out.L) { [out.L,out.D]=[out.D,out.L]; out.yaw+=Math.PI/2; }
        return out;
    }

    function furnitureSurfaces(cells) {
        const step = .12;
        const grid = new Map();
        const key = (i,j) => i+','+j;
        for (const p of cells) {
            if (p.age > 1.5 || p.z < .09 || p.z > 1.45) continue;
            const i = Math.floor(p.x/step), j = Math.floor(p.y/step), k=key(i,j);
            let col = grid.get(k);
            if (!col) { col={i,j,z:[],pts:[]}; grid.set(k,col); }
            col.z.push(p.z); col.pts.push(p);
        }
        const output = [];
        for (const [kind,min,max] of [['table',.65,.86],['chair',.37,.53]]) {
            // Adjacent height slices overlap to tolerate measurement noise.
            for (let level=min; level<=max+.001; level+=.045) {
                const active = new Map();
                for (const [k,col] of grid) {
                    if (!col.z.some(z=>Math.abs(z-level)<.033)) continue;
                    // Tabletops: skip cells where a vertical surface (a pushed-in
                    // chair's backrest) carries on well above the top.
                    if (kind==='table' && col.z.filter(z=>z>level+.06 && z<level+.4).length>=3) continue;
                    active.set(k,col);
                }
                for (const [k,seed] of active) {
                    const patch=[seed];
                    active.delete(k);
                    for (let n=0;n<patch.length;n++) {
                        for (const [di,dj] of [[1,0],[-1,0],[0,1],[0,-1]]) {
                            const neighbor=key(patch[n].i+di,patch[n].j+dj);
                            if (active.has(neighbor)) { patch.push(active.get(neighbor)); active.delete(neighbor); }
                        }
                    }
                    if (patch.length < (kind==='chair'?6:16)) continue;
                    let cx=0,cy=0;
                    for (const col of patch) { cx+=(col.i+.5)*step; cy+=(col.j+.5)*step; }
                    cx/=patch.length; cy/=patch.length;
                    let xx=0,xy=0,yy=0,under=0,above=0,bx=0,by=0;
                    for (const col of patch) {
                        const x=(col.i+.5)*step-cx,y=(col.j+.5)*step-cy;
                        xx+=x*x; xy+=x*y; yy+=y*y;
                        if (col.z.some(z=>z<level-.15)) under++;
                        // Vertical structure above the surface (a backrest), not a
                        // flat overhang such as a tabletop over a pushed-in seat.
                        const up=col.z.filter(z=>z>level+.12 && z<1.16);
                        if (up.length>=2 && Math.max(...up)-Math.min(...up)>=.12) {above++;bx+=x;by+=y;}
                    }
                    if (!under || under>.60*patch.length || above>.58*patch.length) continue;
                    let rect = surfaceBounds(patch,step,cx,cy,.5*Math.atan2(2*xy,xx-yy));
                    if (kind==='table') {
                        // A partly visible tabletop biases covariance toward the
                        // visible diagonal. Fit the perimeter of the occupied cells.
                        for (let deg=0;deg<90;deg+=3) {
                            const candidate=surfaceBounds(patch,step,cx,cy,deg*Math.PI/180);
                            if (candidate.L*candidate.D < rect.L*rect.D*.995) rect=candidate;
                        }
                        if (rect.D>rect.L) {
                            [rect.L,rect.D]=[rect.D,rect.L]; rect.yaw+=Math.PI/2;
                        }
                    } else if (above && Math.hypot(bx,by)/above>=.08) {
                        // The backrest line sets width. Its side of the seat
                        // resolves the 180-degree ambiguity of that line.
                        const back = patch.filter(col=>col.z.some(z=>z>level+.17 && z<1.16));
                        if (back.length<2) continue;
                        let bxx=0,bxy=0,byy=0;
                        for (const col of back) {
                            const x=(col.i+.5)*step-cx-bx/above, y=(col.j+.5)*step-cy-by/above;
                            bxx+=x*x; bxy+=x*y; byy+=y*y;
                        }
                        let heading=Math.atan2(by,bx)-Math.PI/2;
                        if (Math.hypot(bxx-byy,2*bxy)>.5*(bxx+byy)) {
                            heading=.5*Math.atan2(2*bxy,bxx-byy);
                            if (-Math.sin(heading)*bx+Math.cos(heading)*by<0) heading+=Math.PI;
                        }
                        rect=pointBounds(patch,level,surfaceBounds(patch,step,cx,cy,heading),true);
                    }
                    if (kind==='table') rect=pointBounds(patch,level,rect);
                    const {L,D}=rect;
                    if (patch.length*step*step < .55*L*D) continue;
                    // Seat-height surface with no backrest and a table-sized top:
                    // a coffee/side table (e.g. 81 cm round, 46 cm tall).
                    let lowTable = false;
                    if (kind==='chair' && (!above || Math.hypot(bx,by)/above<.08 || L>.76)) {
                        if (above > .25*patch.length || L<.55 || L>1.6 || D<.4 || D>1.2) continue;
                        lowTable = true;
                        for (let deg=0;deg<90;deg+=3) {
                            const candidate=surfaceBounds(patch,step,cx,cy,deg*Math.PI/180);
                            if (candidate.L*candidate.D < rect.L*rect.D*.995) rect=candidate;
                        }
                        if (rect.D>rect.L) { [rect.L,rect.D]=[rect.D,rect.L]; rect.yaw+=Math.PI/2; }
                        rect=pointBounds(patch,level,rect);
                    }
                    if (kind==='chair' && !lowTable && (L<.27 || L>.76 || D<.27 || D>.76)) continue;
                    // 60 cm: a 76 cm square table must stand on its own, not only
                    // when a pushed-in chair stretches it past a longer minimum.
                    if (kind==='table' && (L<.6 || L>2.65 || D<.36 || D>1.38)) continue;
                    // Round top: near-square box, but cells fill well under 100% of it.
                    const round = (lowTable || kind==='table') && rect.D > .85*rect.L && rect.L >= .5
                        && rect.corner !== undefined && rect.corner < .02;
                    if (lowTable) {
                        output.push({kind:'table',...rect,H:level,low:true,round,support:patch.length});
                        continue;
                    }
                    const backHeights = kind==='chair'
                        ? patch.flatMap(col=>col.z.filter(z=>z>level+.17 && z<1.16)).sort((a,b)=>a-b) : [];
                    if (kind==='chair' && !backHeights.length) continue;
                    const height = kind==='chair' ? backHeights[Math.floor(.9*backHeights.length)] : level;
                    output.push({kind,...rect,H:height,seat:level,support:patch.length,...(kind==='table'?{round}:{})});
                }
            }
        }
        const selected=[];
        for (const o of output.sort((a,b)=>b.support-a.support)) {
            if (!selected.some(q=>furnitureOverlap(q,o))) selected.push(o);
        }
        return selected;
    }

    // Object tracks persist across calls (odom frame).
    function Tracker() { this.tracks = []; this.nextId = 1; }

    Tracker.prototype.update = function (cells, cols, wallUsed, dets, pose, nowS, frame = {}) {
        const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
        const toOdom = (x, y) => [pose.x + c * x - s * y, pose.y + s * x + c * y];
        const toBase = (X, Y) => { const dx = X - pose.x, dy = Y - pose.y; return [c * dx + s * dy, -s * dx + c * dy]; };
        const low = cells.filter(q => q.z > 0.03 && !wallUsed.has(colKey(q.x, q.y)) &&
                                     (cols.get(colKey(q.x, q.y))?.top ?? 0) < WALL_TOP + 0.4);
        const matched = new Set();
        const surfaces = frame.xyz ? furnitureSurfaces(cells) : [];
        const claimed = new Set();
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
            if (kind === 'tv') {
                // Use the tangent of a nearby measured wall for a wall-mounted screen.
                let closest = .45;
                for (const wall of frame.walls || []) {
                    const dx = wall.x1 - wall.x0, dy = wall.y1 - wall.y0;
                    const u = clamp(((cx-wall.x0)*dx + (cy-wall.y0)*dy) / (dx*dx + dy*dy), [0,1]);
                    const distance = Math.hypot(cx-wall.x0-u*dx, cy-wall.y0-u*dy);
                    if (distance < closest) { closest = distance; yaw = Math.atan2(dy,dx); }
                }
            }
            const surface = surfaces.filter(o => !claimed.has(o) && o.kind===kind &&
                Math.hypot(o.x-p.x,o.y-p.y) < Math.min(.8,o.L*.55+.15))
                .sort((a,b)=>Math.hypot(a.x-p.x,a.y-p.y)-Math.hypot(b.x-p.x,b.y-p.y))[0];
            if (surface) {
                claimed.add(surface);
                ({x:cx,y:cy,L,D,H,yaw}=surface);
            }
            const [X, Y] = toOdom(cx, cy), Yaw = yaw + pose.theta;
            let t = this.tracks.filter(t => !matched.has(t) && t.kind === kind && Math.hypot(t.X - X, t.Y - Y) < OBJ_MATCH_M)
                .sort((a,b) => Math.hypot(a.X-X,a.Y-Y) - Math.hypot(b.X-X,b.Y-Y))[0];
            if (!t) { t = { id: this.nextId++, kind, X, Y, yaw: Yaw, L, D, H, seen: nowS, hits: 0 }; this.tracks.push(t); }
            t.inferred = false;
            if (surface) t.seat = surface.seat;
            matched.add(t);
            if (frame.detectionStamp != null && t.stamp === frame.detectionStamp) continue;
            t.stamp = frame.detectionStamp;
            t.misses = 0;
            const k = OBJ_SMOOTH;
            t.X += k * (X - t.X); t.Y += k * (Y - t.Y);
            // Box yaw is symmetric by pi: move the shorter way.
            let dyaw = Yaw - t.yaw; dyaw = Math.atan2(Math.sin(2 * dyaw), Math.cos(2 * dyaw)) / 2;
            if (surface && kind==='chair') {
                if (!t.measuredHeading) t.yaw=Yaw;
                else t.yaw+=k*Math.atan2(Math.sin(Yaw-t.yaw),Math.cos(Yaw-t.yaw));
                t.measuredHeading=true;
            } else if (surface || !frame.mapped || t.hits < 3) t.yaw += k * dyaw;
            if (kind === 'tv' && Number.isFinite(p.z)) {
                H = clamp(L * 9 / 16, sz.h);
                t.bottom = Math.max(0, p.z - H / 2);
            }
            t.L += k * (L - t.L); t.D += k * (D - t.D); t.H += k * (H - t.H);
            t.seen = nowS; t.hits++;
        }
        // Without a camera label, require several distinct clouds and age the
        // hypothesis out promptly. Match in world coordinates while the base turns.
        for (const o of surfaces) {
            if (claimed.has(o)) continue;
            const [X,Y] = toOdom(o.x,o.y);
            const yaw = o.yaw + pose.theta;
            const radius = o.kind==='chair' ? .36 : .57;
            const nearby = this.tracks.filter(t=>t.kind===o.kind && Math.hypot(t.X-X,t.Y-Y)<radius)
                .sort((a,b)=>Math.hypot(a.X-X,a.Y-Y)-Math.hypot(b.X-X,b.Y-Y));
            const t = nearby[0];
            if (t && ((!t.inferred && !t.lidarOnly) || matched.has(t))) continue;
            if (!t) {
                const track = {...o,X,Y,yaw,id:this.nextId++,hits:1,seen:nowS,first:nowS,inferred:true};
                this.tracks.push(track);
                matched.add(track);
            } else if (frame.xyz !== this.lastCloud) {
                matched.add(t);
                t.X += .25*(X-t.X); t.Y += .25*(Y-t.Y);
                for (const key of ['H','L','D','seat']) if (Number.isFinite(o[key])) t[key]+=.25*(o[key]-t[key]);
                if (o.kind==='table') t.round = o.round;
                const symmetry = o.kind==='chair' ? 1 : 2;
                const delta = symmetry*(yaw-t.yaw);
                t.yaw += .25*Math.atan2(Math.sin(delta),Math.cos(delta))/symmetry;
                t.hits++; t.seen=nowS; t.misses=0;
                // Lidar-only furniture: the low camera rarely labels chairs and
                // tables, so a surface seen consistently becomes a landmark.
                if (t.inferred && t.hits >= LIDAR_CONFIRM_HITS && nowS - (t.first ?? nowS) >= LIDAR_CONFIRM_S) {
                    t.inferred = false; t.lidarOnly = true;
                }
            }
        }
        this.tracks = this.tracks.filter(t => {
            // Surface fits come and go between clouds; keep the candidate long
            // enough to accumulate hits toward promotion. Candidates are not drawn.
            if (t.inferred) return nowS-t.seen < INFERRED_GAP_S;
            if (!frame.mapped || t.hits < 2) return nowS - t.seen < OBJ_HOLD_S;
            if (!matched.has(t) && frame.xyz && this.lastCloud !== frame.xyz && nowS - t.seen > OBJ_HOLD_S) {
                const [x, y] = toBase(t.X, t.Y);
                if (vacated(frame.xyz, x, y, t)) t.misses++;
                else t.misses = 0;
            }
            return !(t.misses >= 3);
        });
        this.lastCloud = frame.xyz;
        const candidates = this.tracks.filter(t => !t.inferred && t.hits >= 2).map(t => {
            const [x, y] = toBase(t.X, t.Y);
            return { id: t.id, kind: t.kind, x, y, yaw: t.yaw - pose.theta, L: t.L, D: t.D, H: t.H,
                     seat: t.seat, round: !!t.round, inferred: !!t.inferred, bottom: t.bottom || 0, age: nowS - t.seen };
        });
        // A changed partial view can start a second track before the old one
        // expires. Resolve competing models after smoothing, in rendered space.
        candidates.sort((a,b)=>a.age-b.age || Number(a.inferred)-Number(b.inferred) || a.id-b.id);
        const visible=[];
        for (const o of candidates) {
            if (!visible.some(q=>furnitureOverlap(o,q))) visible.push(o);
        }
        return visible;
    };

    // Session landmarks: confirmed camera-labelled tracks in the tracker frame
    // (map frame when a map is loaded). Removal stays evidence-based (vacated).
    const LANDMARK_HITS = 4;
    const LANDMARK_KEYS = ['X','Y','yaw','L','D','H','seat','bottom'];
    Tracker.prototype.exportLandmarks = function () {
        const confirmed = this.tracks.filter(t => !t.inferred && t.hits >= LANDMARK_HITS && !(t.misses > 0));
        // Leave room for landmark metadata under the WebSocket 1 MiB limit.
        const modeled = confirmed.filter(t => t.model?.voxels.size).length;
        const limit = Math.floor(95000 / Math.max(1, modeled));
        return confirmed.map(t => {
            const o = {kind: t.kind};
            for (const k of LANDMARK_KEYS) if (Number.isFinite(t[k])) o[k] = t[k];
            if (t.round) o.round = 1;
            const encoded = Lookalike.pack(t.model, limit);
            if (encoded) o.lookalike = encoded;
            return o;
        });
    };
    Tracker.prototype.importLandmarks = function (list, nowS) {
        let added = 0;
        for (const o of list || []) {
            if (!o || !SIZE[o.kind] || !['X','Y','yaw','L','D','H'].every(k => Number.isFinite(o[k]))) continue;
            // A detection can start a provisional track while the GET is in flight.
            // Restore into that id using the saved box (the model's local frame).
            const existing = this.tracks.find(t => t.kind === o.kind && Math.hypot(t.X-o.X,t.Y-o.Y)<OBJ_MATCH_M);
            if (existing && !existing.inferred && existing.hits >= LANDMARK_HITS) continue;
            const t = {kind: o.kind, id: existing?.id ?? this.nextId++, hits: LANDMARK_HITS, misses: 0, seen: nowS,
                       inferred: false, restored: true, measuredHeading: o.kind === 'chair' && Number.isFinite(o.seat)};
            for (const k of LANDMARK_KEYS) if (Number.isFinite(o[k])) t[k] = o[k];
            t.round = o.round === 1 || o.round === true;
            t.model = Lookalike.unpack(o.lookalike);
            if (existing) this.tracks[this.tracks.indexOf(existing)] = t;
            else { this.tracks.push(t); added++; }
        }
        return added;
    };

    // Require several returns beyond the central footprint at seat/body height.
    // A return inside that region vetoes clearing, including when partly occluded.
    function vacated(xyz, x, y, t) {
        const sx = .0432, sz = .31;
        const dx = x - sx, range = Math.hypot(dx, y);
        if (range < .3) return false;
        const radius = Math.min(t.L, t.D) * .32;
        const level = (t.bottom || 0) + .48 * t.H;
        let through = 0;
        for (let i = 0; i < xyz.length; i += 3) {
            const rx = xyz[i] - sx, ry = xyz[i+1], z = xyz[i+2];
            if (Math.hypot(rx-dx, ry-y) < radius && Math.abs(z-level) < .12) return false;
            const along = (dx*rx + y*ry) / range;
            if (along <= range + Math.max(t.L,t.D)/2 + .2) continue;
            const f = range / along;
            if (Math.hypot(f*rx-dx, f*ry-y) < radius && Math.abs(sz + f*(z-sz)-level) < .1) through++;
        }
        return through >= 4;
    }

    function inside(o, q, pad) {
        const dx = q.x - o.x, dy = q.y - o.y, c = Math.cos(o.yaw), s = Math.sin(o.yaw);
        return Math.abs(dx * c + dy * s) <= o.L / 2 + pad && Math.abs(-dx * s + dy * c) <= o.D / 2 + pad && q.z >= (o.bottom || 0) - pad && q.z <= (o.bottom || 0) + o.H + pad;
    }

    // One call per cloud: walls + objects + the hide predicate for the voxel layer.
    function analyze(cells, dets, pose, tracker, nowS, frame = {}) {
        const cols = columns(cells);
        const { segs, used } = fitWalls(cols);
        const objects = pose && tracker ? tracker.update(cells, cols, used, dets, pose, nowS, {...frame, walls: segs}) : [];
        const walls = frame.wallHistory ? frame.wallHistory.update(frame.xyz, frame.wallPose, nowS) : segs;
        const doorways = frame.wallHistory ? frame.wallHistory.doorways(walls,frame.xyz,nowS) : [];
        return {
            walls, doorways, objects,
            hidden(q) {
                if (!frame.wallHistory && !frame.mapWalls && used.has(colKey(q.x, q.y))) return true;
                for (const o of objects) if (!o.inferred && inside(o, q, 0.06)) return true;
                return false;
            },
        };
    }

    const api = { Lookalike, LANDMARK_HITS, furnitureOverlap, furnitureSurfaces, analyze, Tracker, WallHistory, fitWalls, columns, KINDS, WALL_TOP };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.RealityStructure = api;
})(typeof window !== 'undefined' ? window : globalThis);
