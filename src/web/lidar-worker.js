/* lidar-worker.js
 * Runs in a Web Worker. Owns the OffscreenCanvas for the lidar visualisation.
 * Receives: { type:'init', canvas:OffscreenCanvas }
 *           { type:'draw', buffer:ArrayBuffer }  — Float32Array of [x0,y0,x1,y1,...] pairs
 */

let canvas = null;
let ctx    = null;

self.onmessage = ({ data }) => {
    if (data.type === 'init') {
        canvas = data.canvas;
        ctx    = canvas.getContext('2d');
    } else if (data.type === 'draw' && ctx) {
        drawLidar(new Float32Array(data.buffer));
    }
};

function drawLidar(points) {
    const { width, height } = canvas;
    const cx    = width  / 2;
    const cy    = height / 2;
    const scale = 75; // pixels per metre

    // Clear
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, width, height);

    // Grid rings at 1 m and 2 m
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth   = 1;
    [1, 2].forEach(r => {
        ctx.beginPath();
        ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
        ctx.stroke();
    });

    // Crosshairs
    ctx.beginPath();
    ctx.moveTo(0, cy);     ctx.lineTo(width, cy);
    ctx.moveTo(cx, 0);     ctx.lineTo(cx, height);
    ctx.stroke();

    // Lidar points — flat [x0, y0, x1, y1, ...] in metres (robot frame)
    ctx.fillStyle = '#22c55e';
    for (let i = 0; i < points.length; i += 2) {
        const px = cx - (points[i + 1] * scale);
        const py = cy - (points[i]     * scale);
        ctx.fillRect(px - 1, py - 1, 2, 2);
    }

    // Robot centre dot
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
}
