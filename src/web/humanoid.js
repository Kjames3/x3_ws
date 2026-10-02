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
        let currentSpeed = 0;
        function animate(speed, dt) {
            currentSpeed = Number.isFinite(speed) ? Math.max(0, speed) : 0;
            // Pose fitting must start from a clean frame, even without a clip.
            frame.position.set(0, baseHeight, 0);
            if (!clip) {
                for (const [name, q] of Object.entries(rest)) joints[name].quaternion.copy(q);
                return;
            }
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
        // Confidence-aware body posing, applied after animate() each frame.
        // dirs: bone name -> { direction, confidence }; direction is in scene frame, pointing
        // from the joint to its child. Bones without a direction fade back to
        // the walk clip. Each bone takes the minimal rotation from its
        // current direction, so the clip's twist is kept.
        const POSE_BONES = [['torso', 'lower_waist'], ['left_upper_arm', 'left_lower_arm'], ['left_lower_arm', 'left_hand'],
            ['right_upper_arm', 'right_lower_arm'], ['right_lower_arm', 'right_hand'],
            ['left_thigh', 'left_shin'], ['left_shin', 'left_foot'],
            ['right_thigh', 'right_shin'], ['right_shin', 'right_foot']];
        const poseState = Object.create(null);
        const pq = new THREE.Quaternion(), dq = new THREE.Quaternion(), tq = new THREE.Quaternion();
        const want = new THREE.Vector3(), have = new THREE.Vector3();
        const refinement = window.X3PoseRefinement;
        const contacts = refinement ? { left: refinement.createContact(), right: refinement.createContact() } : null;
        const lastRoot = group.position.clone();
        let lastHeading = 0;
        function aim(node, child, direction) {
            node.parent.updateWorldMatrix(true, false);
            node.parent.getWorldQuaternion(pq);
            want.copy(direction).applyQuaternion(pq.invert());
            have.copy(child.position).normalize().applyQuaternion(node.quaternion);
            node.quaternion.premultiply(dq.setFromUnitVectors(have, want));
        }
        function fitLeg(side, upper, lower) {
            const thigh = joints[side + '_thigh'], shin = joints[side + '_shin'];
            aim(thigh, shin, upper);
            // Set thigh twist so the knee's bend is in its local X/Z plane.
            // A coupled leg gets one bend plane, rather than two unrelated axes.
            thigh.updateWorldMatrix(true, false);
            const worldQ = thigh.getWorldQuaternion(new THREE.Quaternion());
            const localBack = new THREE.Vector3(-1, 0, 0).applyQuaternion(worldQ);
            localBack.addScaledVector(upper, -localBack.dot(upper)).normalize();
            const bend = lower.clone().addScaledVector(upper, -lower.dot(upper));
            if (bend.lengthSq() > 1e-6) {
                bend.normalize();
                const angle = Math.atan2(upper.dot(localBack.clone().cross(bend)), localBack.dot(bend));
                const axis = upper.clone().applyQuaternion(thigh.parent.getWorldQuaternion(new THREE.Quaternion()).invert());
                thigh.quaternion.premultiply(new THREE.Quaternion().setFromAxisAngle(axis, angle));
            }
            aim(shin, joints[side + '_foot'], lower);
        }
        function refineLegs() {
            if (!refinement) return;
            const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(group.quaternion);
            for (const side of ['left', 'right']) {
                const a = poseState[side + '_thigh'], b = poseState[side + '_shin'];
                if (Math.min(a.w, b.w) < 0.01) continue;
                group.updateMatrixWorld(true);
                const hip = joints[side + '_thigh'].getWorldPosition(new THREE.Vector3());
                const knee = joints[side + '_shin'].getWorldPosition(new THREE.Vector3());
                const ankle = joints[side + '_foot'].getWorldPosition(new THREE.Vector3());
                const u = knee.clone().sub(hip).normalize(), v = ankle.clone().sub(knee).normalize();
                const leg = refinement.constrainLeg(u, v, forward);
                // Fade the constraint with observations, preserving the baseline
                // animation exactly after dropout.
                const w = Math.min(a.w, b.w);
                fitLeg(side, u.lerp(leg.upper, w).normalize(), v.lerp(leg.lower, w).normalize());
            }
        }
        function stabilizeFeet(dirs, dt, options) {
            if (!contacts) return;
            const rootJump = group.position.distanceTo(lastRoot) > 0.08;
            const yawJump = Math.abs(Math.atan2(Math.sin(group.rotation.y-lastHeading),
                Math.cos(group.rotation.y-lastHeading))) > 0.04;
            lastRoot.copy(group.position); lastHeading = group.rotation.y;
            const enabled = !ghost && !rootJump && !yawJump && currentSpeed < 0.2 && options.contacts !== false;
            for (const side of ['left', 'right']) {
                group.updateMatrixWorld(true);
                const foot = joints[side + '_foot'];
                const hip = joints[side + '_thigh'].getWorldPosition(new THREE.Vector3());
                const knee = joints[side + '_shin'].getWorldPosition(new THREE.Vector3());
                const ankle = foot.getWorldPosition(new THREE.Vector3());
                const ground = new THREE.Box3().setFromObject(foot).min.y - group.position.y;
                const confidence = Math.min(...['_thigh', '_shin'].map(suffix =>
                    dirs && dirs[side + suffix] ? dirs[side + suffix].confidence : 0));
                const target = refinement.contactTarget(contacts[side], ankle, ground, confidence, dt, enabled);
                if (target && ankle.distanceTo(target) > 0.001) {
                    const leg = refinement.solveLeg(hip, knee, ankle, target);
                    if (leg) fitLeg(side, leg.upper, leg.lower);
                    else { contacts[side].anchor = null; contacts[side].quiet = 0; }
                }
            }
            group.userData.footContacts = ['left', 'right'].filter(side => contacts[side].anchor);
        }
        function pose(dirs, dt, options = {}) {
            dt = Number.isFinite(dt) ? Math.max(0, Math.min(dt, 0.1)) : 0;
            const alpha = 1 - Math.exp(-dt / 0.12);
            for (const [bone, child] of POSE_BONES) {
                const st = poseState[bone] || (poseState[bone] = { w: 0, age: Infinity, confidence: 0, dir: new THREE.Vector3() });
                const observation = dirs && dirs[bone];
                const d = observation && observation.confidence >= 0.5 &&
                    observation.direction && observation.direction.toArray().every(Number.isFinite) &&
                    observation.direction.lengthSq() > 1e-4 ? observation.direction : null;
                st.age = d ? 0 : st.age + dt;
                if (d) {
                    st.confidence = Math.min(1, observation.confidence);
                    if (st.w === 0) st.dir.copy(d);
                    else st.dir.lerp(d, alpha).normalize();
                }
                // Brief dropouts hold the last direction, then fade to the clip.
                const target = st.age < 0.2 ? Math.min(1, (st.confidence - 0.35) / 0.4) : 0;
                st.w += (target - st.w) * alpha;
                if (st.w < 0.01) { st.w = 0; continue; }
                const node = joints[bone];
                node.parent.updateWorldMatrix(true, false);
                node.parent.getWorldQuaternion(pq);
                want.copy(st.dir).applyQuaternion(pq.invert());
                have.copy(joints[child].position).normalize().applyQuaternion(node.quaternion);
                tq.copy(dq.setFromUnitVectors(have, want)).multiply(node.quaternion);
                node.quaternion.slerp(tq, st.w);
            }
            refineLegs();
            // Ground the support foot after leg/torso fitting. This lowers the
            // pelvis for a visible seated bend without moving the tracked root,
            // and leaves a raised leg free when the other foot supports the body.
            const supportWeight = Math.max(...['torso', 'left_thigh', 'left_shin', 'right_thigh', 'right_shin']
                .map(name => poseState[name].w));
            if (supportWeight > 0) {
                group.updateMatrixWorld(true);
                leftFootBox.setFromObject(joints.left_foot);
                rightFootBox.setFromObject(joints.right_foot);
                frame.position.y += group.position.y - Math.min(leftFootBox.min.y, rightFootBox.min.y);
            }
            stabilizeFeet(dirs, dt, options);
            group.userData.poseConfidence = Math.min(...['left_thigh', 'left_shin', 'right_thigh', 'right_shin']
                .map(name => poseState[name].w));
        }
        return { group, joints, animate, pose };
    }
    window.X3Humanoid = { create };
})();
