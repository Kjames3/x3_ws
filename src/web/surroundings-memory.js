// Bounded local display memory in odom. Never used by navigation or clearance.
(function (root) {
    const SIZE = .10, TTL = 8, LIMIT = 12000;
    const key = (x, y, z) => `${Math.floor(x / SIZE)},${Math.floor(y / SIZE)},${Math.floor(z / SIZE)}`;
    const world = (x, y, pose) => [pose.x + Math.cos(pose.yaw) * x - Math.sin(pose.yaw) * y,
        pose.y + Math.sin(pose.yaw) * x + Math.cos(pose.yaw) * y];
    const local = (x, y, pose) => [Math.cos(pose.yaw) * (x - pose.x) + Math.sin(pose.yaw) * (y - pose.y),
        -Math.sin(pose.yaw) * (x - pose.x) + Math.cos(pose.yaw) * (y - pose.y)];
    class Memory {
        constructor() { this.clear(); }
        clear() { this.cells = new Map(); this.floor = new Map(); this.previous = null; this.dynamic = []; this.revision = (this.revision || 0) + 1; }
        ingest(xyz, pose, movers = []) {
            if (!pose || ![pose.x, pose.y, pose.yaw, pose.stamp].every(Number.isFinite)) return false;
            if (this.previous) {
                const dt = pose.stamp - this.previous.stamp;
                const da = Math.abs(Math.atan2(Math.sin(pose.yaw - this.previous.yaw), Math.cos(pose.yaw - this.previous.yaw)));
                if (dt <= 0 || dt > 2 || Math.hypot(pose.x - this.previous.x, pose.y - this.previous.y) > .5 + .8 * dt || da > .4 + 2 * dt) this.clear();
            }
            this.previous = pose;
            const now = pose.stamp, observed = new Map(), hits = [], dynamic = [];
            const moving = (x, y) => movers.some(m => Math.hypot(x - m.x, y - m.y) < .45);
            for (let i = 0; i < xyz.length; i += 3) {
                const x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
                if (![x, y, z].every(Number.isFinite) || z < -.15) continue;
                const [wx, wy] = world(x, y, pose), k = key(wx, wy, z);
                hits.push([wx, wy, z]);
                if (moving(x, y)) { dynamic.push({x, y, z, age: 0, confidence: 0}); continue; }
                if (!observed.has(k)) observed.set(k, {x: wx, y: wy, z});
                // Only actual near-floor endpoints establish observed floor tiles.
                // A high laser ray passing over a tile says nothing about its floor.
                if (z >= -.05 && z <= .06) {
                    const ix = Math.floor(wx / .25), iy = Math.floor(wy / .25);
                    this.floor.set(`${ix},${iy}`, {x: (ix + .5) * .25, y: (iy + .5) * .25, seen: now});
                }
            }
            // At most 512 measured 3-D rays. Repeated contradictory frames are
            // required; missing returns never clear occupied space.
            const [sx, sy] = world(.0432, 0, pose), sz = .31, cleared = new Set();
            const stride = Math.max(1, Math.ceil(hits.length / 512));
            for (let i = 0; i < hits.length; i += stride) {
                const h = hits[i], dx = h[0] - sx, dy = h[1] - sy, dz = h[2] - sz;
                const length = Math.hypot(dx, dy, dz);
                for (let d = .15; d < length - .20; d += .08) {
                    const k = key(sx + dx * d / length, sy + dy * d / length, sz + dz * d / length);
                    if (this.cells.has(k) && !observed.has(k)) cleared.add(k);
                }
            }
            for (const k of cleared) { const c = this.cells.get(k); if (++c.misses >= 3) this.cells.delete(k); }
            for (const [k, p] of observed) {
                let c = this.cells.get(k);
                if (!c || now - c.seen > 1.2 && c.hits < 3) {
                    c = { x: (Math.floor(p.x / SIZE) + .5) * SIZE, y: (Math.floor(p.y / SIZE) + .5) * SIZE,
                        z: (Math.floor(p.z / SIZE) + .5) * SIZE, hits: 0, seen: now, misses: 0 };
                    this.cells.set(k, c);
                }
                c.hits = Math.min(8, c.hits + 1); c.seen = now; c.misses = 0;
            }
            for (const [k, c] of this.cells) {
                const [x, y] = local(c.x, c.y, pose);
                if (now - c.seen > (c.hits >= 3 ? TTL : .35) || Math.hypot(x, y) > 7 || moving(x, y)) this.cells.delete(k);
            }
            for (const [k, c] of this.floor) if (now - c.seen > TTL || Math.hypot(c.x - pose.x, c.y - pose.y) > 7) this.floor.delete(k);
            if (this.cells.size > LIMIT) {
                const oldest = [...this.cells].sort((a, b) => a[1].seen - b[1].seen);
                for (let i = 0; i < oldest.length - LIMIT; i++) this.cells.delete(oldest[i][0]);
            }
            this.dynamic = dynamic; this.revision++;
            return true;
        }
        snapshot(pose) {
            if (!pose) return {cells: [], floor: []};
            const cells = [...this.cells.values()].map(c => {
                const [x, y] = local(c.x, c.y, pose);
                return {x, y, z: c.z, age: pose.stamp - c.seen, confidence: c.hits >= 3 ? 1 : 0};
            });
            cells.push(...(this.dynamic || []).slice(0, Math.max(0, LIMIT - cells.length)));
            const floor = [...this.floor.values()].map(c => {
                const [x, y] = local(c.x, c.y, pose);
                return {x, y, age: pose.stamp - c.seen};
            });
            return {cells, floor};
        }
    }
    const api = {Memory, world, local, SIZE, TTL, LIMIT};
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.SurroundingsMemory = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
