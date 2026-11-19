// Conservative screen-facing depth splats. World-space nearest-neighbor spacing
// controls diameter; no temporal accumulation or invented surface connections.
// Four compatible neighbors avoid shrinking a whole patch around one close pair.
(function (root) {
    function diameters(xyz, colors) {
        const n = xyz.length / 3, result = new Float32Array(n), cells = new Map();
        const uncertain = new Uint8Array(n);
        for(let i=0;i<n;i++) uncertain[i]=[0,1,2].every(j=>Math.abs(colors[3*i+j]-160/255)<.002);
        const cell = .04, key = (x,y,z) => `${x},${y},${z}`;
        for (let i=0; i<n; i++) {
            const k=key(Math.floor(xyz[3*i]/cell),Math.floor(xyz[3*i+1]/cell),Math.floor(xyz[3*i+2]/cell));
            if (!cells.has(k)) cells.set(k,[]);
            cells.get(k).push(i);
        }
        for (let i=0; i<n; i++) {
            const p=3*i;
            // Publisher marks uncertain RGB with neutral 160. Legitimate exact
            // gray may also stay small: conservative ambiguity, no recoloring.
            if (uncertain[i]) { result[i]=.01; continue; }
            const x=xyz[p],y=xyz[p+1],z=xyz[p+2];
            const cx=Math.floor(x/cell),cy=Math.floor(y/cell),cz=Math.floor(z/cell);
            let nearest=Infinity, boundary=Infinity;
            let n0=Infinity,n1=Infinity,n2=Infinity,n3=Infinity;
            for (let a=-1;a<=1;a++) for(let b=-1;b<=1;b++) for(let c=-1;c<=1;c++) {
                for (const j of cells.get(key(cx+a,cy+b,cz+c)) || []) {
                    if(i===j) continue;
                    const d=(x-xyz[3*j])**2+(y-xyz[3*j+1])**2+(z-xyz[3*j+2])**2;
                    if(d<=1e-10 || d>.03**2) continue;
                    nearest=Math.min(nearest,d);
                    const q=3*j;
                    const contrast=Math.max(Math.abs(colors[p]-colors[q]),Math.abs(colors[p+1]-colors[q+1]),Math.abs(colors[p+2]-colors[q+2]));
                    if(uncertain[j] || contrast>.25) boundary=Math.min(boundary,d);
                    else if(d<n3) {
                        if(d<n0) {n3=n2;n2=n1;n1=n0;n0=d;}
                        else if(d<n1) {n3=n2;n2=n1;n1=d;}
                        else if(d<n2) {n3=n2;n2=d;}
                        else n3=d;
                    }
                }
            }
            // Isolated samples stay dots. Never grow a patch across a large gap.
            let diameter=nearest <= .03**2 ? 1.35*Math.sqrt(nearest) : .01;
            if(Number.isFinite(n3)) diameter=1.5*Math.sqrt(n3);
            // The radius stays at most halfway to contrasting/uncertain samples.
            // This limits color bleed; it is not a calibrated surface boundary.
            result[i]=Math.max(.008,Math.min(.035,diameter,Math.sqrt(boundary)));
        }
        return result;
    }
    function material(THREE) {
        const m=new THREE.PointsMaterial({size:.01,vertexColors:true,depthTest:true,depthWrite:true});
        m.onBeforeCompile=shader => {
            shader.vertexShader='attribute float splatDiameter;\n'+shader.vertexShader;
            shader.vertexShader=shader.vertexShader.replace('gl_PointSize = size;', 'gl_PointSize = size * splatDiameter / 0.01;');
            shader.fragmentShader=shader.fragmentShader.replace('void main() {',
                'void main() {\n vec2 splatUV = gl_PointCoord * 2.0 - 1.0;\n if (dot(splatUV, splatUV) > 1.0) discard;');
        };
        m.customProgramCacheKey=()=> 'oak-circular-splats-v1';
        return m;
    }
    root.OakSplats={diameters,material};
    if (typeof module !== 'undefined') module.exports=root.OakSplats;
})(typeof window !== 'undefined' ? window : globalThis);
