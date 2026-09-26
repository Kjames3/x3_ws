/* Primitive humanoid adapted from NVIDIA IsaacGymEnvs assets/mjcf/nv_humanoid.xml,
 * itself derived from DeepMind dm_control. Apache-2.0; see
 * models/licenses/humanoid-LICENSE.txt.
 * Source: https://github.com/isaac-sim/IsaacGymEnvs/blob/main/assets/mjcf/nv_humanoid.xml
 * Modifications: visual body hierarchy only; physics, actuators and sensors omitted.
 * Dimensions retained, then normalized to a generic 1.7 m standing figure.
 * This is an illustrative avatar, not a measured skeleton or collision model.
 */
(function () {
    'use strict';
    const BODY = {"name":"torso","pos":[0.0,0.0,1.5],"geoms":[{"name":"torso","fromto":[0.0,-0.07,0.0,0.0,0.07,0.0],"size":[0.07]},{"name":"upper_waist","fromto":[-0.01,-0.06,-0.12,-0.01,0.06,-0.12],"size":[0.06]}],"children":[{"name":"head","pos":[0.0,0.0,0.19],"geoms":[{"name":"head","type":"sphere","size":[0.09]}],"children":[]},{"name":"lower_waist","pos":[-0.01,0.0,-0.26],"quat":[1.0,0.0,-0.002,0.0],"geoms":[{"name":"lower_waist","fromto":[0.0,-0.06,0.0,0.0,0.06,0.0],"size":[0.06]}],"children":[{"name":"pelvis","pos":[0.0,0.0,-0.165],"quat":[1.0,0.0,-0.002,0.0],"geoms":[{"name":"butt","fromto":[-0.02,-0.07,0.0,-0.02,0.07,0.0],"size":[0.09]}],"children":[{"name":"right_thigh","pos":[0.0,-0.1,-0.04],"geoms":[{"name":"right_thigh","fromto":[0.0,0.0,0.0,0.0,0.01,-0.34],"size":[0.06]}],"children":[{"name":"right_shin","pos":[0.0,0.01,-0.403],"geoms":[{"name":"right_shin","fromto":[0.0,0.0,0.0,0.0,0.0,-0.3],"size":[0.049]}],"children":[{"name":"right_foot","pos":[0.0,0.0,-0.39],"geoms":[{"name":"right_right_foot","fromto":[-0.07,-0.02,0.0,0.14,-0.04,0.0],"size":[0.027]},{"name":"left_right_foot","fromto":[-0.07,0.0,0.0,0.14,0.02,0.0],"size":[0.027]}],"children":[]}]}]},{"name":"left_thigh","pos":[0.0,0.1,-0.04],"geoms":[{"name":"left_thigh","fromto":[0.0,0.0,0.0,0.0,-0.01,-0.34],"size":[0.06]}],"children":[{"name":"left_shin","pos":[0.0,-0.01,-0.403],"geoms":[{"name":"left_shin","fromto":[0.0,0.0,0.0,0.0,0.0,-0.3],"size":[0.049]}],"children":[{"name":"left_foot","pos":[0.0,0.0,-0.39],"geoms":[{"name":"left_left_foot","fromto":[-0.07,0.02,0.0,0.14,0.04,0.0],"size":[0.027]},{"name":"right_left_foot","fromto":[-0.07,0.0,0.0,0.14,-0.02,0.0],"size":[0.027]}],"children":[]}]}]}]}]},{"name":"right_upper_arm","pos":[0.0,-0.17,0.06],"geoms":[{"name":"right_upper_arm","fromto":[0.0,0.0,0.0,0.16,-0.16,-0.16],"size":[0.04,0.16]}],"children":[{"name":"right_lower_arm","pos":[0.18,-0.18,-0.18],"geoms":[{"name":"right_lower_arm","fromto":[0.01,0.01,0.01,0.17,0.17,0.17],"size":[0.031]}],"children":[{"name":"right_hand","pos":[0.18,0.18,0.18],"geoms":[{"name":"right_hand","type":"sphere","size":[0.04]}],"children":[]}]}]},{"name":"left_upper_arm","pos":[0.0,0.17,0.06],"geoms":[{"name":"left_upper_arm","fromto":[0.0,0.0,0.0,0.16,0.16,-0.16],"size":[0.04,0.16]}],"children":[{"name":"left_lower_arm","pos":[0.18,0.18,-0.18],"geoms":[{"name":"left_lower_arm","fromto":[0.01,-0.01,0.01,0.17,-0.17,0.17],"size":[0.031]}],"children":[{"name":"left_hand","pos":[0.18,-0.18,0.18],"geoms":[{"name":"left_hand","type":"sphere","size":[0.04]}],"children":[]}]}]}]};
    let sphere, cylinder, solidMaterial, ghostMaterial;
    function resources() {
        if (sphere) return;
        sphere = new THREE.SphereGeometry(1, 10, 6);
        cylinder = new THREE.CylinderGeometry(1, 1, 1, 10, 1, true);
        solidMaterial = new THREE.MeshLambertMaterial({ color: 0xf59e0b });
        ghostMaterial = new THREE.MeshLambertMaterial({
            color: 0xf59e0b, transparent: true, opacity: 0.18, depthWrite: false
        });
    }

    function create({ ghost = false } = {}) {
        resources();
        const joints = Object.create(null);
        const material = ghost ? ghostMaterial : solidMaterial;
        function mesh(parent, geometry, position, scale) {
            const m = new THREE.Mesh(geometry, material);
            m.position.copy(position);
            m.scale.copy(scale);
            m.castShadow = !ghost;
            parent.add(m);
            return m;
        }
        function build(spec) {
            const node = new THREE.Group();
            node.name = spec.name;
            node.position.fromArray(spec.pos);
            if (spec.quat) {
                const [w, x, y, z] = spec.quat;
                node.quaternion.set(x, y, z, w).normalize();
            }
            // Named body groups are targeted by the baked mocap clip.
            // Their local frames are the original MJCF frames (Z up, X forward).
            node.userData.restQuaternion = node.quaternion.toArray();
            joints[spec.name] = node;
            for (const g of spec.geoms) {
                const r = g.size[0];
                const radius = new THREE.Vector3(r, r, r);
                if (g.type === 'sphere') {
                    mesh(node, sphere, new THREE.Vector3(...(g.pos || [0, 0, 0])), radius);
                } else {
                    const a = new THREE.Vector3(...g.fromto.slice(0, 3));
                    const b = new THREE.Vector3(...g.fromto.slice(3));
                    const direction = b.clone().sub(a);
                    const limb = mesh(node, cylinder, a.clone().add(b).multiplyScalar(0.5),
                        new THREE.Vector3(r, direction.length(), r));
                    limb.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
                    mesh(node, sphere, a, radius);
                    mesh(node, sphere, b, radius);
                }
            }
            for (const child of spec.children) node.add(build(child));
            return node;
        }
        const group = new THREE.Group();
        group.name = ghost ? 'person-prediction' : 'person-avatar';
        const frame = new THREE.Group();
        // MJCF X forward, Y left, Z up -> viewer X right, Y up, -Z forward.
        frame.quaternion.setFromRotationMatrix(new THREE.Matrix4().set(
            0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1));
        frame.add(build(BODY));
        // The source has raised, bent arms. Start with relaxed arms while
        // stopped; keep the original body names/frames.
        for (const side of ['left', 'right']) {
            const upper = joints[side + '_upper_arm'];
            const lower = joints[side + '_lower_arm'];
            const hand = joints[side + '_hand'];
            const down = new THREE.Vector3(0, side === 'left' ? 0.06 : -0.06, -0.30).normalize();
            upper.quaternion.setFromUnitVectors(lower.position.clone().normalize(), down);
            const localDown = down.clone().applyQuaternion(upper.quaternion.clone().invert());
            lower.quaternion.setFromUnitVectors(hand.position.clone().normalize(), localDown);
            upper.userData.restQuaternion = upper.quaternion.toArray();
            lower.userData.restQuaternion = lower.quaternion.toArray();
        }
        group.add(frame);
        const box = new THREE.Box3().setFromObject(group);
        const scale = 1.7 / (box.max.y - box.min.y);
        frame.scale.setScalar(scale);
        frame.position.y = -box.min.y * scale;
        const baseHeight = frame.position.y;
        const rest = Object.fromEntries(Object.entries(joints).map(([name, node]) => [name, node.quaternion.clone()]));
        const clip = window.X3WalkCycle;
        const qa = new THREE.Quaternion(), qb = new THREE.Quaternion();
        const leftFootBox = new THREE.Box3(), rightFootBox = new THREE.Box3();
        let phase = 0, weight = 0, filteredSpeed = 0;
        function animate(speed, dt) {
            if (!clip) return; // A missing animation asset leaves a standing avatar.
            dt = Number.isFinite(dt) ? Math.max(0, Math.min(dt, 0.1)) : 0;
            speed = Number.isFinite(speed) ? Math.max(0, speed) : 0;
            const moving = speed > 0.15;
            const alpha = 1 - Math.exp(-dt / 0.15);
            filteredSpeed += (speed - filteredSpeed) * alpha;
            weight += ((moving ? 1 : 0) - weight) * alpha;
            if (!moving && weight < 0.001) weight = 0;
            if (moving || weight > 0) {
                const rate = Math.max(0.25, Math.min(2.5, filteredSpeed / clip.referenceSpeed));
                phase = (phase + dt * rate / clip.duration) % 1;
            }
            const sample = phase * (clip.samples.length - 1);
            const i = Math.floor(sample), t = sample - i;
            const next = Math.min(i + 1, clip.samples.length - 1);
            clip.joints.forEach((name, index) => {
                qa.fromArray(clip.samples[i], index * 4).normalize();
                qb.fromArray(clip.samples[next], index * 4).normalize();
                qa.slerp(qb, t);
                joints[name].quaternion.copy(rest[name]).slerp(qa, weight);
            });
            // Keep the planted foot at floor level without moving the track root.
            frame.position.y = baseHeight + (clip.floor[i] * (1 - t) + clip.floor[next] * t) * weight;
            // Quaternion blending is nonlinear: interpolating the baked height
            // alone can sink the feet during start/stop. Correct support only
            // during transitions; steady playback uses the cheap baked offset.
            if (weight > 0 && weight < 0.999) {
                group.updateMatrixWorld(true);
                leftFootBox.setFromObject(joints.left_foot);
                rightFootBox.setFromObject(joints.right_foot);
                frame.position.y += group.position.y - Math.min(leftFootBox.min.y, rightFootBox.min.y);
            }
        }
        return { group, joints, animate };
    }
    window.X3Humanoid = { create };
})();
