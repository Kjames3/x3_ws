#!/usr/bin/env python3
"""Derive the X3 v2 URDF frames from the SolidWorks per-component STL export.

Input: a directory of STLs exported from the assembly with "do not translate"
(assembly coordinates, millimetres, +Y up), as in New_version/New_X3_components_STL.
Output: a YAML of poses in base_footprint (metres, x forward, y left, z up) that
x3_v2.urdf.xacro loads. Re-run it after every chassis change:

    python3 src/tools/calibration/cad_x3_frames.py New_version/New_X3_components_STL \\
        --out src/yahboomcar_description/config/x3_v2_cad_frames.yaml

Locked Design_v2 export (2026-11-18: assembly_poses.json + part-local STLs in
stl/, unzip the *.STL.zip files there first) also writes the URDF meshes:

    python3 src/tools/calibration/cad_x3_frames.py <Design_v2 dir> \
        --out src/yahboomcar_description/config/x3_v2_cad_frames.yaml \
        --mesh-dir src/yahboomcar_description/meshes/x3_v2

Its front is the Top_bus_light_yahboom end (user-confirmed); see main_posed.

Frame construction (nothing hand-entered):
  * wheel centres = bbox centres of each "Mecanum Wheel Assembly-N" group;
    radius = half their vertical extent;
  * up = normal of the plane through the four wheel centres. The 2026-11-13
    assembly is rolled ~0.7 deg; levelling here keeps that roll out of every
    sensor pose (the draft URDF carried it);
  * origin = wheel-centre centroid dropped by the radius onto that plane (the
    floor under the centroid), i.e. base_footprint;
  * forward = from the rear wheel pair to the front pair, where "front" is the
    end holding the front OAK. CAD cannot tell which end is the front:
    --front-oak picks which OAK enclosure (1 or 2 in the export) is the front
    one. Default: the enclosure nearer the part named front_bottomplate (the
    designer's own labels); the 2026-11-13 draft URDF and its meshes agree.
    Not yet confirmed on the physical robot.

Sensor poses are CAD positions of the mount parts. Anything the CAD gets wrong
is overridden in the xacro, not here (e.g. the Livox optical-centre offset
above its housing base).
"""
import argparse
import glob
import os
import struct
import sys
from datetime import date

import numpy as np


def load_stl(path):
    with open(path, "rb") as f:
        b = f.read()
    n = struct.unpack("<I", b[80:84])[0] if len(b) >= 84 else 0
    if 84 + n * 50 == len(b):
        a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype(
            [("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        return a["v"].reshape(-1, 3).astype(float)
    lines = b.decode(errors="ignore").splitlines()
    return np.array([[float(t) for t in ln.split()[1:4]] for ln in lines
                     if ln.strip().startswith("vertex")])


def group(d, pattern):
    files = sorted(glob.glob(os.path.join(d, pattern)))
    if not files:
        raise SystemExit(f"no STL matches {pattern!r} in {d}")
    return np.vstack([load_stl(f) for f in files]) / 1000.0     # mm -> m


def bbox_centre(v):
    return (v.min(0) + v.max(0)) / 2


class RobotFrame:
    """Assembly (m, +Y up) -> base_footprint (x fwd, y left, z up)."""

    def __init__(self, wheels, radius, front_point):
        P = np.array(wheels)
        c = P.mean(0)
        _, _, vt = np.linalg.svd(P - c)
        up = vt[2] if vt[2][1] > 0 else -vt[2]
        f = front_point - c
        f = f - up * f.dot(up)
        # forward = the rectangle side direction nearest the front point
        sides = []
        for i in range(4):
            for j in range(i + 1, 4):
                d = P[j] - P[i]
                d = d - up * d.dot(up)
                sides.append(d / np.linalg.norm(d))
        fwd = max(sides, key=lambda s: abs(s.dot(f)))
        fwd = fwd if fwd.dot(f) > 0 else -fwd
        left = np.cross(up, fwd)
        self.R = np.vstack([fwd, left, up])            # rows: robot axes in assembly coords
        self.origin = c - up * radius
        self.roll_deg = float(np.degrees(np.arccos(np.clip(up[1], -1, 1))))

    def p(self, x):
        return self.R @ (np.asarray(x) - self.origin)

    def d(self, v):
        return self.R @ np.asarray(v)


def plane_normal(v):
    c = v.mean(0)
    _, _, vt = np.linalg.svd(v - c, full_matrices=False)
    return vt[2]


def r6(x):
    return [round(float(t), 4) for t in x]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stl_dir")
    ap.add_argument("--out", default="-")
    ap.add_argument("--front-oak", type=int, choices=(1, 2),
                    help="force which OAK enclosure is the front one (default: the one nearer "
                         "the part named front_bottomplate, i.e. the designer's own labels)")
    ap.add_argument("--draft-urdf", help="draft URDF whose wheel-joint xyz place its meshes "
                                         "(reports the mesh->base_footprint transform)")
    ap.add_argument("--mesh-dir", help="posed export only: write decimated base_link.stl and "
                                       "wheel_*.stl (base_footprint-aligned) here")
    a = ap.parse_args()
    d = a.stl_dir
    if os.path.exists(os.path.join(d, "assembly_poses.json")):
        return main_posed(a)

    wheels_raw, radius = [], 0.0
    for i in range(1, 5):
        v = group(d, f"Mecanum Wheel Assembly-{i} *")
        wheels_raw.append(bbox_centre(v))
        # Vertical extent (+Y in the export): the rollers stick out further along
        # the rolling direction, so the largest extent overstates the radius.
        radius = max(radius, float(v[:, 1].max() - v[:, 1].min()) / 2)
    oak_glass = {k: group(d, f"DM9098ProW_enclosure (1).stp-{k} *GLASS-LENS*") for k in (1, 2)}
    named_front = sorted(glob.glob(os.path.join(d, "front_bottomplate*")))
    named_back = sorted(glob.glob(os.path.join(d, "back_plate*")))
    by_name = None
    if named_front:
        fp = bbox_centre(group(d, "front_bottomplate*"))
        by_name = min((1, 2), key=lambda k: np.linalg.norm(bbox_centre(oak_glass[k]) - fp))
    front = a.front_oak or by_name or 1
    rear = 3 - front
    F = RobotFrame(wheels_raw, radius, bbox_centre(oak_glass[front]))
    front_rule = "--front-oak" if a.front_oak else ("part names" if by_name else "default 1")
    if by_name and front != by_name:
        print(f"WARNING: --front-oak {front} disagrees with the CAD part names "
              f"(front_bottomplate is nearer OAK {by_name})", file=sys.stderr)
    if named_back and F.p(bbox_centre(group(d, "back_plate*")))[0] > 0:
        print("WARNING: back_plate ends up at the FRONT of this frame", file=sys.stderr)

    wheels = {}
    for w in wheels_raw:
        p = F.p(w)
        key = ("front" if p[0] > 0 else "back") + "_" + ("left" if p[1] > 0 else "right")
        wheels[key] = r6(p)
    if len(wheels) != 4:
        raise SystemExit(f"wheel quadrant assignment failed: {wheels}")

    def oak(k):
        g = oak_glass[k]
        centre = F.p(bbox_centre(g))
        n = F.d(plane_normal(g))
        n = n if n.dot(centre - [0, 0, centre[2]]) > 0 else -n   # out of the robot
        yaw = float(np.arctan2(n[1], n[0]))
        pitch = float(-np.arcsin(np.clip(n[2], -1, 1)))         # +pitch = nose down
        return {"glass_centre": r6(centre), "yaw": round(yaw, 4), "pitch": round(pitch, 4)}

    liv = group(d, "mid-360-asm*")
    liv_r = np.array([F.p(x) for x in liv[::max(1, len(liv) // 20000)]])
    icm = group(d, "ICM_imu_module_mount*")
    icm_r = np.array([F.p(x) for x in icm])

    allv = []
    for f in sorted(glob.glob(os.path.join(d, "*.STL")) + glob.glob(os.path.join(d, "*.stl"))):
        allv.append(load_stl(f) / 1000.0)      # every vertex: extremes are what count
    allv = np.array([F.p(x) for x in np.vstack(allv)])
    # Ignore parts floating far off the robot (the export carries a stray panel).
    near = np.hypot(allv[:, 0], allv[:, 1]) < 0.9
    ext = allv[near]

    out = {
        "source": os.path.basename(os.path.normpath(d)),
        "generated": str(date.today()),
        "front_oak_enclosure": front,
        "front_chosen_by": front_rule,
        "assembly_roll_removed_deg": round(F.roll_deg, 3),
        "wheel_radius": round(radius, 4),
        "wheels": wheels,
        "wheelbase": round(float(np.mean([wheels["front_left"][0], wheels["front_right"][0]])
                                 - np.mean([wheels["back_left"][0], wheels["back_right"][0]])), 4),
        "track": round(float(np.mean([wheels["front_left"][1], wheels["back_left"][1]])
                             - np.mean([wheels["front_right"][1], wheels["back_right"][1]])), 4),
        "oak_front": oak(front),
        "oak_rear": oak(rear),
        "livox_housing": {"centre_xy": r6(bbox_centre(liv_r)[:2]),
                          "base_z": round(float(liv_r[:, 2].min()), 4),
                          "top_z": round(float(liv_r[:, 2].max()), 4)},
        "icm_mount": {"centre": r6(bbox_centre(icm_r)), "top_z": round(float(icm_r[:, 2].max()), 4)},
        "extent": {"x": r6([ext[:, 0].min(), ext[:, 0].max()]),
                   "y": r6([ext[:, 1].min(), ext[:, 1].max()]),
                   "z_top": round(float(ext[:, 2].max()), 4)},
    }

    if a.draft_urdf:
        import re
        txt = open(a.draft_urdf).read()
        draft = {}
        for name in ("front_left", "front_right", "rear_left", "rear_right"):
            m = re.search(rf'wheel_{name}_joint".*?<origin xyz="([^"]+)"', txt, re.S)
            draft[name.replace("rear", "back")] = np.array([float(t) for t in m.group(1).split()])
        flip = {"front_left": "back_right", "front_right": "back_left",
                "back_left": "front_right", "back_right": "front_left"}
        # Four wheel centres on a rectangle fit equally well rotated 180 deg, so
        # the draft base mesh is checked against the CAD parts to break the tie.
        mesh_path = os.path.join(os.path.dirname(a.draft_urdf), "..", "meshes", "base_link.stl")
        parts = []
        for f in sorted(glob.glob(os.path.join(d, "*.STL"))):
            if "Mecanum" not in f:
                v = load_stl(f) / 1000.0
                parts.append(v[:: max(1, len(v) // 300)])
        parts = np.array([F.p(x) for x in np.vstack(parts)])
        parts = parts[np.hypot(parts[:, 0], parts[:, 1]) < 0.9]
        mesh_v = load_stl(mesh_path) if os.path.exists(mesh_path) else None
        best = None
        for perm in ({k: k for k in draft}, flip):      # same front, or the draft reversed
            names = sorted(draft)
            A = np.array([draft[k] for k in names])
            B = np.array([wheels[perm[k]] for k in names])
            ca, cb = A.mean(0), B.mean(0)
            U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
            Rm = (U @ Vt).T
            if np.linalg.det(Rm) < 0:
                Vt[-1] *= -1
                Rm = (U @ Vt).T
            t = cb - Rm @ ca
            resid = float(np.abs((A @ Rm.T + t) - B).max())
            score = resid
            if mesh_v is not None:
                from scipy.spatial import cKDTree
                score = float(np.median(cKDTree(mesh_v @ Rm.T + t).query(parts)[0]))
            if best is None or score < best[0]:
                best = (score, Rm, t, perm, resid)
        score, Rm, t, perm, resid = best
        # which draft wheel mesh sits at each of our wheel positions
        out["wheel_mesh"] = {perm[k]: "wheel_" + k.replace("back", "rear") for k in draft}
        sy = np.hypot(Rm[0, 0], Rm[1, 0])
        rpy = [np.arctan2(Rm[2, 1], Rm[2, 2]), np.arctan2(-Rm[2, 0], sy), np.arctan2(Rm[1, 0], Rm[0, 0])]
        out["draft_mesh_to_base_footprint"] = {"xyz": r6(t), "rpy": r6(rpy),
                                               "max_wheel_residual_m": round(resid, 4),
                                               "median_mesh_to_parts_m": round(score, 4)}

    import yaml
    text = ("# GENERATED by src/tools/calibration/cad_x3_frames.py -- do not edit by hand.\n"
            "# base_footprint frame: floor under the wheel centroid, x forward, y left, z up (m, rad).\n"
            + yaml.safe_dump(out, sort_keys=False, default_flow_style=None))
    if a.out == "-":
        sys.stdout.write(text)
    else:
        with open(a.out, "w") as f:
            f.write(text)
        print(f"wrote {a.out}")
        sys.stdout.write(text)


# ---------------------------------------------------------------------------
# Posed export (Design_v2, 2026-11-18): stl/<mesh>.STL in PART-LOCAL mm frames
# plus assembly_poses.json (p_asm = R^T p_local + t, metres, +Y up).
# ---------------------------------------------------------------------------
def _load_tris(path):
    with open(path, "rb") as f:
        b = f.read()
    n = struct.unpack("<I", b[80:84])[0]
    if 84 + n * 50 != len(b):
        raise SystemExit(f"{path}: not a binary STL")
    a = np.frombuffer(b[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n)
    return a["v"].reshape(-1, 3, 3).astype(float) / 1000.0


def _write_stl(path, tris):
    tris = np.asarray(tris, dtype="<f4")
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"] = n
    rec["v"] = tris.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"x3_v2 cad_x3_frames.py".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def _decimate(tris, cell):
    """Vertex clustering: snap to a grid, drop collapsed and duplicate triangles."""
    q = np.round(tris / cell).astype(np.int64)
    ok = ~((q[:, 0] == q[:, 1]).all(1) | (q[:, 1] == q[:, 2]).all(1) | (q[:, 0] == q[:, 2]).all(1))
    q = q[ok]
    key = np.sort(q.reshape(len(q), -1).view([("", np.int64)] * 3), axis=1)
    _, idx = np.unique(key.reshape(len(q), -1), axis=0, return_index=True)
    return q[np.sort(idx)] * cell


class PosedExport:
    def __init__(self, d):
        import json
        self.d = d
        self.inst = json.load(open(os.path.join(d, "assembly_poses.json")))["instances"]
        self._tris = {}

    def local(self, mesh):
        if mesh not in self._tris:
            path = os.path.join(self.d, "stl", mesh + ".STL")
            if not os.path.exists(path):
                raise SystemExit(f"{path} missing (unzip the *.STL.zip files in stl/ first)")
            self._tris[mesh] = _load_tris(path)
        return self._tris[mesh]

    @staticmethod
    def pose(i):
        return np.array(i["rotation_3x3"]).reshape(3, 3), np.array(i["translation_m"])

    def tris(self, i):
        R, t = self.pose(i)
        return self.local(i["mesh"]) @ R + t           # row form of R^T p + t

    def find(self, pred):
        return [i for i in self.inst if pred(i)]


def main_posed(a):
    import fnmatch
    X = PosedExport(a.stl_dir)
    wheel_inst = sorted(X.find(lambda i: i["mesh"] == "Mecanum_Wheel_Assembly"), key=lambda i: i["instance"])
    if len(wheel_inst) != 4:
        raise SystemExit(f"expected 4 wheels, got {len(wheel_inst)}")
    wl = X.local("Mecanum_Wheel_Assembly").reshape(-1, 3)
    wc_local = bbox_centre(wl)
    ext = wl.max(0) - wl.min(0)
    axle_ax = int(np.argmin(ext))
    radius = float(np.delete(ext, axle_ax).min() / 2)   # rollers overstate the other extent
    wheels_raw = []
    for i in wheel_inst:
        R, t = X.pose(i)
        wheels_raw.append(wc_local @ R + t)

    oaks = X.find(lambda i: i["mesh"].startswith("DM9098ProW"))
    # Front = the end with Top_bus_light_yahboom (user-confirmed, Design_v2 make_urdf.py);
    # back_plate must then come out at the rear.
    fp = bbox_centre(X.tris(X.find(lambda i: i["mesh"] == "Top_bus_light_yahboom")[0]).reshape(-1, 3))
    by_name = min(range(len(oaks)), key=lambda k: np.linalg.norm(bbox_centre(X.tris(oaks[k]).reshape(-1, 3)) - fp))
    # --front-oak N names the instance "...stp-N", as in the per-component export.
    front = (next(k for k, i in enumerate(oaks) if i["instance"].endswith(f"-{a.front_oak}"))
             if a.front_oak else by_name)
    rear = 1 - front
    F = RobotFrame(wheels_raw, radius, bbox_centre(X.tris(oaks[front]).reshape(-1, 3)))
    bp = X.find(lambda i: i["mesh"] == "back_plate")
    if bp and F.p(bbox_centre(X.tris(bp[0]).reshape(-1, 3)))[0] > 0:
        print("WARNING: back_plate ends up at the FRONT of this frame", file=sys.stderr)

    wheels, wheel_by_key = {}, {}
    for i, w in zip(wheel_inst, wheels_raw):
        p = F.p(w)
        key = ("front" if p[0] > 0 else "back") + "_" + ("left" if p[1] > 0 else "right")
        wheels[key] = r6(p)
        wheel_by_key[key] = i
    if len(wheels) != 4:
        raise SystemExit(f"wheel quadrant assignment failed: {wheels}")

    def to_robot(tris):
        return (tris - F.origin) @ F.R.T

    def oak(k):
        # OAK Pro W enclosure 97 x 30 x 23 mm: the thinnest local axis is the view axis;
        # the glass is the face on the outward side.
        i = oaks[k]
        v = X.local(i["mesh"]).reshape(-1, 3)
        R, t = X.pose(i)
        ax = int(np.argmin(v.max(0) - v.min(0)))
        c = F.p(bbox_centre(v) @ R + t)
        n = F.d(R[ax])                                     # local axis in assembly, then robot
        n = n if n.dot(c - [0, 0, c[2]]) > 0 else -n
        half = (v.max(0) - v.min(0))[ax] / 2
        g = c + n * half
        return {"glass_centre": r6(g), "yaw": round(float(np.arctan2(n[1], n[0])), 4),
                "pitch": round(float(-np.arcsin(np.clip(n[2], -1, 1))), 4)}

    def part_pts(mesh):
        return np.vstack([to_robot(X.tris(i).reshape(-1, 3)) for i in X.find(lambda i: i["mesh"] == mesh)])

    liv_r = part_pts("mid-360-asm")
    icm_r = part_pts("ICM_imu_module_mount")
    allv = np.vstack([to_robot(X.tris(i).reshape(-1, 3)) for i in X.inst])
    arducams = []
    for i in sorted(X.find(lambda i: i["mesh"] == "Arducam_B0332"), key=lambda i: i["instance"]):
        c = bbox_centre(to_robot(X.tris(i).reshape(-1, 3)))
        arducams.append(r6(c))

    out = {
        "source": os.path.basename(os.path.normpath(a.stl_dir)) + " (assembly_poses.json)",
        "generated": str(date.today()),
        "front_oak_enclosure": int(oaks[front]["instance"].rsplit("-", 1)[1]),   # stp-N, as the old export
        "front_chosen_by": "--front-oak" if a.front_oak else "Top_bus_light_yahboom end (user-confirmed)",
        "assembly_roll_removed_deg": round(F.roll_deg, 3),
        "wheel_radius": round(radius, 4),
        "wheel_width": round(float(ext[axle_ax]), 4),
        "wheels": wheels,
        "wheelbase": round(float(np.mean([wheels["front_left"][0], wheels["front_right"][0]])
                                 - np.mean([wheels["back_left"][0], wheels["back_right"][0]])), 4),
        "track": round(float(np.mean([wheels["front_left"][1], wheels["back_left"][1]])
                             - np.mean([wheels["front_right"][1], wheels["back_right"][1]])), 4),
        "oak_front": oak(front),
        "oak_rear": oak(rear),
        "livox_housing": {"centre_xy": r6(bbox_centre(liv_r)[:2]),
                          "base_z": round(float(liv_r[:, 2].min()), 4),
                          "top_z": round(float(liv_r[:, 2].max()), 4)},
        "icm_mount": {"centre": r6(bbox_centre(icm_r)), "top_z": round(float(icm_r[:, 2].max()), 4)},
        "arducam_centres": arducams,
        "extent": {"x": r6([allv[:, 0].min(), allv[:, 0].max()]),
                   "y": r6([allv[:, 1].min(), allv[:, 1].max()]),
                   "z_top": round(float(allv[:, 2].max()), 4)},
        "wheel_mesh": {"front_left": "wheel_front_left", "front_right": "wheel_front_right",
                       "back_left": "wheel_rear_left", "back_right": "wheel_rear_right"},
        # meshes are written straight into base_footprint / wheel-centre frames
        "draft_mesh_to_base_footprint": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
    }

    if a.mesh_dir:
        os.makedirs(a.mesh_dir, exist_ok=True)
        body = np.vstack([to_robot(X.tris(i)) for i in X.inst if i["mesh"] != "Mecanum_Wheel_Assembly"])
        _write_stl(os.path.join(a.mesh_dir, "base_link.stl"), _decimate(body, 0.002))
        for key, i in wheel_by_key.items():
            tri = to_robot(X.tris(i)) - np.array(wheels[key])
            _write_stl(os.path.join(a.mesh_dir, out["wheel_mesh"][key] + ".stl"), _decimate(tri, 0.0015))
        print(f"wrote meshes to {a.mesh_dir}", file=sys.stderr)

    import yaml
    text = ("# GENERATED by src/tools/calibration/cad_x3_frames.py -- do not edit by hand.\n"
            "# base_footprint frame: floor under the wheel centroid, x forward, y left, z up (m, rad).\n"
            + yaml.safe_dump(out, sort_keys=False, default_flow_style=None))
    if a.out == "-":
        sys.stdout.write(text)
    else:
        with open(a.out, "w") as f:
            f.write(text)
        print(f"wrote {a.out}", file=sys.stderr)
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
