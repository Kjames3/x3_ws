// Display geometry only. Input: floor-relative base_footprint XYZ in metres.
// No free-space inference: these are distances to observed occupied cells.
(function (root) {
    const CELL = 0.10;
    const FOOTPRINT = { front: 0.16, rear: 0.13, halfWidth: 0.11 };
    function analyze(xyz, height) {
        const cells = new Map();
        for (let i = 0; i < xyz.length; i += 3) {
            const x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
            if (![x, y, z].every(Number.isFinite) || z < -0.15) continue;
            const ix = Math.floor(x / CELL), iy = Math.floor(y / CELL), iz = Math.floor(z / CELL);
            const key = `${ix},${iy},${iz}`;
            if (!cells.has(key)) cells.set(key, { x: (ix + .5) * CELL, y: (iy + .5) * CELL, z: (iz + .5) * CELL, observedMax: z, observedMin: z });
            else { const c = cells.get(key); c.observedMax = Math.max(c.observedMax, z); c.observedMin = Math.min(c.observedMin, z); }
        }
        const half = CELL / 2, margin = .05, reach = 1.5;
        const f = FOOTPRINT;
        let front = Infinity, left = Infinity, right = Infinity, overhead = Infinity, overlap = false;
        const sections = new Map();
        for (const c of cells.values()) {
            const minZ = c.z - half, maxZ = c.z + half;
            // Floor cells do not establish clearance. The Livox has a near-floor blind region.
            c.bodyHeight = c.observedMax > .05 && minZ < height;
            const corridor = c.x + half >= -f.rear && c.x - half <= f.front + reach && Math.abs(c.y) - half <= f.halfWidth;
            const body = c.x + half >= -f.rear && c.x - half <= f.front && Math.abs(c.y) - half <= f.halfWidth;
            c.risk = 0;
            if (c.bodyHeight) {
                if (corridor) { c.risk = 2; front = Math.min(front, Math.max(0, c.x - half - f.front)); }
                else if (c.x + half >= -f.rear && c.x - half <= f.front + reach && Math.abs(c.y) - half <= f.halfWidth + margin) c.risk = 1;
                if (body) overlap = true;
                if (c.x + half >= -f.rear && c.x - half <= f.front) {
                    if (c.y > 0) left = Math.min(left, Math.max(0, c.y - half - f.halfWidth));
                    else right = Math.min(right, Math.max(0, -c.y - half - f.halfWidth));
                }
                if (c.x > f.front && c.x < f.front + reach) {
                    const key = Math.floor(c.x / .20);
                    const pair = sections.get(key) || { left: Infinity, right: Infinity, x: c.x, blocked: false };
                    if (Math.abs(c.y) - half <= f.halfWidth) pair.blocked = true;
                    if (c.y > 0) pair.left = Math.min(pair.left, Math.max(0, c.y - half));
                    else pair.right = Math.min(pair.right, Math.max(0, -c.y - half));
                    sections.set(key, pair);
                }
            }
            if (corridor && minZ >= height) {
                overhead = Math.min(overhead, minZ - height);
                if (minZ - height < margin) c.risk = Math.max(c.risk, 1);
            }
        }
        let gap = null;
        for (const s of sections.values()) {
            if (s.blocked || !Number.isFinite(s.left + s.right)) continue;
            const width = s.left + s.right;
            if (!gap || width < gap.width) gap = { ...s, width, spare: width - f.halfWidth * 2 };
        }
        return { cells: [...cells.values()], front, left, right, overhead, overlap, gap, height };
    }
    const api = { CELL, FOOTPRINT, analyze };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.SurroundingsGeometry = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
