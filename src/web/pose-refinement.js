// Browser-only geometric priors. These constrain a display skeleton; they are
// not measurements of hidden joints and must never feed robot collision logic.
(function () {
    'use strict';
    const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
    const MAX_KNEE = 150 * Math.PI / 180;
    const MAX_TWIST = 80 * Math.PI / 180;
    const UP = new THREE.Vector3(0, 1, 0);
    function perpendicular(v, axis) {
        return v.clone().addScaledVector(axis, -v.dot(axis));
    }
    function constrainLeg(upper, lower, forward) {
        const u = upper.clone().normalize(), v = lower.clone().normalize();
        // Preserve abduction (a sideways leg lift); only extreme hip extension
        // above 135 degrees from vertical is clipped.
        const elevation = Math.acos(clamp(-u.y, -1, 1));
        if (elevation > Math.PI * 0.75) {
            const horizontal = new THREE.Vector3(u.x, 0, u.z).normalize();
            if (horizontal.lengthSq() < 1e-6) horizontal.copy(forward);
            u.copy(horizontal).multiplyScalar(Math.sin(Math.PI * 0.75));
            u.y = -Math.cos(Math.PI * 0.75);
        }
        const angle = Math.min(MAX_KNEE, Math.acos(clamp(u.dot(v), -1, 1)));
        let back = perpendicular(forward.clone().negate(), u);
        // When the thigh points along body forward, vertical is the stable
        // flexion cue (e.g. a seated thigh with shin dropping toward the floor).
        if (back.lengthSq() < 0.01) back = perpendicular(UP.clone().negate(), u);
        if (back.lengthSq() < 1e-6) back = perpendicular(new THREE.Vector3(1, 0, 0), u);
        back.normalize();
        const observed = perpendicular(v, u);
        let twist = 0;
        if (observed.lengthSq() > 1e-6) {
            observed.normalize();
            twist = Math.atan2(u.dot(back.clone().cross(observed)), back.dot(observed));
        }
        const bend = back.applyAxisAngle(u, clamp(twist, -MAX_TWIST, MAX_TWIST));
        return { upper: u, lower: u.clone().multiplyScalar(Math.cos(angle))
            .addScaledVector(bend, Math.sin(angle)).normalize(), bend, angle };
    }

    // Solve hip -> knee -> ankle for fixed lengths and a preferred knee side.
    // Return null instead of stretching a leg to maintain an impossible contact.
    function solveLeg(hip, knee, ankle, target) {
        const l1 = hip.distanceTo(knee), l2 = knee.distanceTo(ankle);
        const axis = target.clone().sub(hip), d = axis.length();
        const minReach = Math.sqrt(l1*l1 + l2*l2 + 2*l1*l2*Math.cos(MAX_KNEE));
        if (d < minReach || d > l1+l2-1e-5 || d < 1e-5) return null;
        axis.divideScalar(d);
        const along = (l1*l1 - l2*l2 + d*d) / (2*d);
        let side = perpendicular(knee.clone().sub(hip), axis);
        if (side.lengthSq() < 1e-8) return null; // straight leg: no trustworthy bend side
        side.normalize();
        const solvedKnee = hip.clone().addScaledVector(axis, along)
            .addScaledVector(side, Math.sqrt(Math.max(0, l1*l1-along*along)));
        return { upper: solvedKnee.clone().sub(hip).normalize(),
            lower: target.clone().sub(solvedKnee).normalize() };
    }

    function createContact() {
        return { anchor: null, previous: null, quiet: 0 };
    }
    function contactTarget(c, ankle, floorHeight, confidence, dt, enabled) {
        const step = Math.max(0, Math.min(Number.isFinite(dt) ? dt : 0, 0.1));
        const distance = c.previous ? ankle.distanceTo(c.previous) : Infinity;
        const speed = step > 0 ? distance / step : Infinity;
        c.previous = ankle.clone();
        const valid = enabled && confidence >= 0.7 && floorHeight < 0.035;
        if (!valid || speed > 0.45 || (c.anchor && ankle.distanceTo(c.anchor) > 0.075)) {
            c.anchor = null; c.quiet = 0;
            return null;
        }
        if (!c.anchor) {
            c.quiet = speed < 0.12 ? c.quiet + step : 0;
            if (c.quiet >= 0.2) c.anchor = ankle.clone();
        }
        return c.anchor;
    }

    // Broad frontal/back views only. Signed shoulder AND hip ordering must
    // agree. A flattened pose cannot establish side-on yaw; leave it unknown.
    function facingCue(k, scores) {
        if (![5,6,11,12].every(i => k[i] && k[i].length === 3 && k[i].every(Number.isFinite)
            && scores[i] && scores[i][2] >= 0.75)) return null;
        const sw = k[5][0]-k[6][0], hw = k[11][0]-k[12][0];
        const torso = Math.hypot((k[5][0]+k[6][0]-k[11][0]-k[12][0])/2,
            (k[5][1]+k[6][1]-k[11][1]-k[12][1])/2);
        if (torso < 0.15 || sw*hw <= 0 || Math.abs(sw)/torso < 0.65 || Math.abs(hw)/torso < 0.35) return null;
        return { yaw: sw > 0 ? Math.PI : 0, confidence: Math.min(...[5,6,11,12].map(i=>scores[i][2])) };
    }
    window.X3PoseRefinement = { constrainLeg, solveLeg, createContact, contactTarget, facingCue };
})();
