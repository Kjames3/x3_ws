#!/usr/bin/env python3
"""Room inventory from a parked room_capture.py capture (step 2, offline).

Camera segmentation says WHAT and WHICH PIXELS; the stacked Livox cloud, projected
into the same image through the camera-lidar calibration, says WHERE. Per object:

1. Segment every Nth frame of a camera (default yolo11x-seg; the nano models read
   these mono frames at 0.25-0.4 confidence, the x model at 0.55-0.8).
2. Lidar points whose projection falls inside the (slightly eroded) mask, then only
   the nearest connected piece in top view (what shows through gaps or behind an
   edge - the couch behind the round table - is another piece).
3. Instances are merged across frames by class and 3D centre; an object needs to be
   seen in MIN_SHARE of the frames.
4. Fit in base_footprint: top-view min-area rectangle (yaw, L, D), height from the
   95th percentile. Tables: round if the top's hull fills ~pi/4 of its rectangle.
   Chairs/couches: the backrest is where the points rise highest; its side is the
   back (+depth in the GUI mesh). The lidar sees only the camera-facing surfaces, so
   a footprint shallower than the class minimum is extended AWAY from the robot.
5. Clean-up: one object with two labels keeps the better-supported one; tables the
   camera never labels come from the lidar (a surface at 0.60-0.88 m with open space
   under it); chairs within 35 cm of a table face it and are moved out of it.

Usage:
  room_inventory.py CAPTURE_DIR --oak-calib oak_front_calib.json [--model yolo11x-seg.pt]
Writes CAPTURE_DIR/inventory.json, inventory_top.png and inventory_<camera>.jpg.

Calibration: config/camera_lidar_extrinsics_2026-11-20.json gives the front OAK RGB
(CAM_A) pose in livox_frame; the GUI "front" stream is the OAK LEFT mono (CAM_B,
640x400), so its pose is T_lidar_A @ T_A_B from the OAK EEPROM (dump with
depthai readCalibration while x3_server is stopped; front serial 14442C10B1F4D2D600).
"""
import argparse, glob, json, math, os, sys
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
LIVOX_Z = 0.0815 + 0.1916          # base_footprint -> livox_frame (tf_static, no rotation)
KINDS = {"chair": "chair", "couch": "couch", "dining table": "table", "tv": "tv", "bed": "bed"}
# Plausible footprint (L, D) and height ranges; matches reality-structure.js SIZE.
SIZE = {"chair": ((0.4, 0.7), (0.4, 0.7), (0.7, 1.1)), "couch": ((0.75, 2.6), (0.7, 1.1), (0.6, 1.0)),
        "table": ((0.5, 2.2), (0.5, 1.2), (0.4, 0.9)), "tv": ((0.5, 1.8), (0.05, 0.5), (0.3, 1.6)),
        "bed": ((1.8, 2.3), (0.9, 2.1), (0.4, 0.8))}
MIN_SHARE = 0.25                    # of processed frames an instance must appear in
# Per-class confidence floor. The sofa reads "couch" at only ~0.22 but in 29/31
# frames; MIN_SHARE is the real evidence for it.
CONF_FLOOR = {"couch": 0.15}
TABLE_BAND = (0.60, 0.88)           # lidar table search: surface height band (m) ...
TABLE_CELL = 0.05                   # ... on a 5 cm top-view grid ...
SEAT_AT_TABLE_M = 0.35              # chairs this close to a table edge face it
MERGE_M = 0.5                       # same-class detections closer than this are one object


def T7(v):
    T = np.eye(4); T[:3, :3] = R.from_quat(v[3:]).as_matrix(); T[:3, 3] = v[:3]; return T


def front_camera(oak_calib):
    ext = json.load(open(os.path.join(REPO, "config", "camera_lidar_extrinsics_2026-11-20.json")))
    c = json.load(open(oak_calib))
    T_ab = np.array(c["T_CAM_B_to_CAM_A_cm"], float); T_ab[:3, 3] /= 100.0
    T_l_cam = T7(ext["front"]["T_lidar_camera"]) @ T_ab
    return dict(K=np.array(c["CAM_B"]["K_640x400"], float), D=np.array(c["CAM_B"]["D"][:8], float),
                T_cam_l=np.linalg.inv(T_l_cam), size=(640, 400))


def project(cam, X):
    """Lidar points (livox_frame) -> pixel uv, camera depth, valid mask."""
    P = (cam["T_cam_l"][:3, :3] @ X.T).T + cam["T_cam_l"][:3, 3]
    ok = P[:, 2] > 0.2
    uv = np.full((len(X), 2), -1.0)
    uv[ok] = cv2.projectPoints(P[ok].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), cam["K"], cam["D"])[0].reshape(-1, 2)
    w, h = cam["size"]
    ok &= (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    return uv, P[:, 2], ok


def nearest_piece(pts, rng, min_pts=15, cell=0.06):
    """Mask points -> the nearest substantial connected piece (top view, 6 cm cells,
    8-connected). A mask also covers what shows through gaps and behind edges (the
    couch behind the round table); depth layering by range alone keeps it."""
    if len(pts) < min_pts:
        return np.zeros(len(pts), bool)
    ij = np.floor(pts[:, :2] / cell).astype(int)
    ij -= ij.min(0)
    count = np.zeros(ij.max(0) + 1, np.int32)
    np.add.at(count, (ij[:, 0], ij[:, 1]), 1)
    # A single scan line grazing two objects bridges them; need a few points per cell.
    grid = (count >= 4).astype(np.uint8)
    n, lab = cv2.connectedComponents(grid, connectivity=4)
    lab = lab[ij[:, 0], ij[:, 1]]
    best, best_r = None, np.inf
    for k in range(1, n):
        m = lab == k
        if m.sum() < max(min_pts, 0.15 * len(pts)):
            continue
        r = np.percentile(rng[m], 20)
        if r < best_r:
            best, best_r = m, r
    return best if best is not None else np.zeros(len(pts), bool)


def fit(kind, pts, robot_xy=(0.0, 0.0)):
    """Oriented box in base_footprint from an object's lidar points (N x 3)."""
    (lr, dr, hr) = SIZE[kind]
    body = pts[pts[:, 2] > 0.06]
    if len(body) < 20:
        return None
    rect = cv2.minAreaRect(body[:, :2].astype(np.float32))
    (cx, cy), (w, h), ang = rect
    yaw = math.radians(ang)
    L, D = w, h
    if D > L:
        L, D, yaw = D, L, yaw + math.pi / 2
    ux, uy = math.cos(yaw), math.sin(yaw)
    top = float(np.percentile(body[:, 2], 95))
    out = dict(kind=kind, x=cx, y=cy, yaw=yaw, L=L, D=D, H=top, n=int(len(body)))
    if kind == "table":
        # Top surface: points within 4 cm of the top; a circle fills pi/4 of its box.
        surf = body[body[:, 2] > top - 0.04][:, :2].astype(np.float32)
        if len(surf) >= 15:
            hull = cv2.convexHull(surf); a = cv2.contourArea(hull)
            (_, _), (sw, sh), _ = cv2.minAreaRect(surf)
            fill = a / max(sw * sh, 1e-6)
            out["fill"] = round(float(fill), 3)
            out["round"] = bool(fill < 0.86 and min(sw, sh) > 0.7 * max(sw, sh))
            # The lidar sees only the near half of a round top: fit a circle to the
            # rim instead (algebraic fit), and put the table at its centre.
            A = np.c_[2 * surf[:, 0], 2 * surf[:, 1], np.ones(len(surf))]
            b = (surf[:, :2].astype(float) ** 2).sum(1)
            (ax_, ay_, c0), *_ = np.linalg.lstsq(A, b, rcond=None)
            rad = math.sqrt(max(c0 + ax_ ** 2 + ay_ ** 2, 0))
            res = np.abs(np.hypot(surf[:, 0] - ax_, surf[:, 1] - ay_) - rad)
            ang = np.degrees(np.ptp(np.unwrap(np.sort(np.arctan2(surf[:, 1] - ay_, surf[:, 0] - ax_)))))
            out.update(circle_r=round(rad, 3), circle_rms=round(float(np.sqrt((res ** 2).mean())), 3), circle_arc=round(float(ang)))
            if 0.25 <= rad <= 0.65 and out["circle_rms"] < 0.05 and ang >= 100:
                out.update(round=True, x=float(ax_), y=float(ay_), L=2 * rad, D=2 * rad)
                cx, cy, w, h = ax_, ay_, 2 * rad, 2 * rad
    if kind in ("chair", "couch"):
        # Backrest: points well above the seat. Back side = where they sit along depth.
        seat_hi = np.percentile(body[:, 2], 40) + 0.18
        hi = body[body[:, 2] > seat_hi]
        nx, ny = -uy, ux                                     # +depth axis for this yaw
        if len(hi) >= 8:
            side = np.median((hi[:, 0] - cx) * nx + (hi[:, 1] - cy) * ny)
            all_ = np.median((body[:, 0] - cx) * nx + (body[:, 1] - cy) * ny)
            if side < all_:                                  # back on -depth: flip
                yaw += math.pi; nx, ny = -nx, -ny
            out["back_offset"] = round(float(abs(side - all_)), 3)
    # The lidar saw the robot-facing side only: grow a too-shallow footprint away from the robot.
    want_D = min(max(out["D"], dr[0]), dr[1])
    want_L = min(max(out["L"], lr[0]), lr[1])
    if want_D > out["D"]:
        nx, ny = -math.sin(yaw), math.cos(yaw)
        away = np.sign((cx - robot_xy[0]) * nx + (cy - robot_xy[1]) * ny) or 1.0
        grow = (want_D - out["D"]) / 2
        out["x"] += away * nx * grow; out["y"] += away * ny * grow
    if out.get("round") and "circle_r" in out:
        want_L = want_D = out["L"]
    out.update(L=want_L, D=want_D, H=min(max(top, hr[0]), hr[1]), yaw=math.atan2(math.sin(yaw), math.cos(yaw)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--oak-calib", required=True)
    ap.add_argument("--model", default="yolo11x-seg.pt")
    ap.add_argument("--every", type=int, default=4)
    ap.add_argument("--conf", type=float, default=0.3)
    a = ap.parse_args()
    from ultralytics import YOLO

    stack = np.load(os.path.join(a.capture, "livox_stack.npz"))
    X = stack["xyz"].astype(np.float64); X = X[np.linalg.norm(X, axis=1) > 0.3]
    # Voxel-thin the 10 s stack to 2 cm: dense enough, far fewer points to project.
    _, keep = np.unique(np.floor(X / 0.02).astype(np.int64), axis=0, return_index=True)
    X = X[np.sort(keep)]
    base = X + [0, 0, LIVOX_Z]                         # livox_frame -> base_footprint (x fwd, y left)
    cam = front_camera(a.oak_calib)
    uv, depth, inimg = project(cam, X)
    rng = np.linalg.norm(X[:, :2], axis=1)
    model = YOLO(a.model)
    frames = sorted(glob.glob(os.path.join(a.capture, "front", "*.jpg")))[::a.every]
    dets = []                                          # (frame, kind, conf, idx array)
    for fi, f in enumerate(frames):
        img = cv2.imread(f)
        r = model.predict(img, conf=min([a.conf, *CONF_FLOOR.values()]), imgsz=640, retina_masks=True, verbose=False)[0]
        if r.masks is None:
            continue
        for m, k, s in zip(r.masks.data.cpu().numpy(), r.boxes.cls.tolist(), r.boxes.conf.tolist()):
            kind = KINDS.get(model.names[int(k)])
            if not kind or s < CONF_FLOOR.get(kind, a.conf):
                continue
            mask = cv2.erode((m > 0.5).astype(np.uint8), np.ones((5, 5), np.uint8))
            u = uv[inimg].astype(int)
            hit = np.flatnonzero(inimg)[mask[u[:, 1], u[:, 0]] > 0]
            if len(hit) < 15:
                continue
            hit = hit[nearest_piece(base[hit], rng[hit])]
            if len(hit) < 15:
                continue
            dets.append((fi, kind, s, hit, base[hit].mean(0)))
            if os.environ.get("INV_DEBUG"):
                print(fi, kind, round(s, 2), len(hit), np.round(base[hit].mean(0), 2))
    # Merge across frames: greedy by confidence, same kind, centres within MERGE_M.
    objs = []
    for fi, kind, s, hit, c in sorted(dets, key=lambda d: -d[2]):
        o = next((o for o in objs if o["kind"] == kind and np.linalg.norm(o["c"][:2] - c[:2]) < MERGE_M), None)
        if o is None:
            objs.append(dict(kind=kind, c=c, frames={fi}, conf=[s], idx=[hit]))
        else:
            o["frames"].add(fi); o["conf"].append(s); o["idx"].append(hit)
    inv = []
    for o in objs:
        share = len(o["frames"]) / max(len(frames), 1)
        if share < MIN_SHARE:
            continue
        idx = np.unique(np.concatenate(o["idx"]))
        f = fit(o["kind"], base[idx])
        if f is None:
            continue
        f.update(seen=round(share, 2), conf=round(float(np.median(o["conf"])), 2), source="camera")
        f["_idx"] = idx
        inv.append(f)
    inv = drop_duplicates(inv)
    inv += lidar_tables(base, inv)
    seat_chairs(inv)
    for i, f in enumerate(inv):
        if os.environ.get("INV_DEBUG") and "_idx" in f:
            np.save(os.path.join(a.capture, f"inv_pts_{i}_{f['kind']}.npy"), base[f["_idx"]])
        f.pop("_idx", None)
    for f in inv:
        f.update({k: round(float(v), 3) for k, v in f.items() if isinstance(v, float)})
    json.dump(dict(capture=os.path.abspath(a.capture), frame="base_footprint", frames=len(frames),
                   model=os.path.basename(a.model), objects=inv), open(os.path.join(a.capture, "inventory.json"), "w"), indent=1)
    for f in inv:
        print(f"{f['kind']:6s} x{f['x']:5.2f} y{f['y']:5.2f} yaw{math.degrees(f['yaw']):6.0f} "
              f"{f['L']:.2f}x{f['D']:.2f}x{f['H']:.2f} seen {f['seen']:.0%} conf {f['conf']}"
              + (f" round {f['round']} fill {f['fill']}" if 'round' in f else "")
              + (f" back {f['back_offset']}" if 'back_offset' in f else ""))
    draw(a.capture, base, inv, cam, frames)


def drop_duplicates(inv):
    """One physical object labelled twice (e.g. a "chair" and a "couch" mask on the
    same armchair): keep the better-supported label when most points are shared."""
    kept = []
    for f in sorted(inv, key=lambda f: -f["seen"] * f["conf"]):
        if any(len(np.intersect1d(f["_idx"], k["_idx"])) > 0.5 * min(len(f["_idx"]), len(k["_idx"])) for k in kept):
            continue
        kept.append(f)
    return kept


def lidar_tables(base, inv):
    """Tables the camera did not label: 5 cm cells with a surface in TABLE_BAND and
    little directly beneath it (open space under a top) or above it (not a wall or a
    shelf), grouped 8-connected. The parked lidar sees the near part of the top; fit()
    grows the box away from the robot to the class minimum."""
    ij = np.floor(base[:, :2] / TABLE_CELL).astype(int)
    off = ij.min(0); ij -= off
    shape = ij.max(0) + 1
    z = base[:, 2]
    def count(m):
        c = np.zeros(shape, np.int32); np.add.at(c, (ij[m, 0], ij[m, 1]), 1); return c
    top = count((z >= TABLE_BAND[0]) & (z <= TABLE_BAND[1]))
    under = count((z >= 0.25) & (z < 0.55))
    above = count(z > 1.0)
    grid = ((top >= 5) & (under < 0.3 * top) & (above < 0.3 * top)).astype(np.uint8)
    n, lab = cv2.connectedComponents(grid, connectivity=8)
    out = []
    for k in range(1, n):
        cells = np.argwhere(lab == k)
        if len(cells) < 12:
            continue
        xy = (cells + off + 0.5) * TABLE_CELL
        if any(f["kind"] == "table" and math.hypot(f["x"] - xy[:, 0].mean(), f["y"] - xy[:, 1].mean()) < 0.6 for f in inv):
            continue
        sel = lab[ij[:, 0], ij[:, 1]] == k
        pts = base[sel & (z >= TABLE_BAND[0]) & (z <= TABLE_BAND[1])]
        # fit() needs a body; legs are not reliably in the cells, so use the top band only.
        f = fit("table", pts)
        if f is None:
            continue
        f.update(seen=1.0, conf=0.0, source="lidar", round=False)
        f.pop("circle_r", None); f.pop("circle_rms", None); f.pop("circle_arc", None)
        out.append(f)
    return out


def seat_chairs(inv):
    """A chair next to a table faces it (back away, +depth). The camera-only
    backrest guess is weak for chairs at a table, whose seat hides under the top.
    A chair overlapping the table is moved out past the edge its centre is beyond."""
    tables = [f for f in inv if f["kind"] == "table"]
    for ch in (f for f in inv if f["kind"] == "chair"):
        for tb in tables:
            c, s = math.cos(tb["yaw"]), math.sin(tb["yaw"])
            dx, dy = ch["x"] - tb["x"], ch["y"] - tb["y"]
            u, v = c * dx + s * dy, -s * dx + c * dy
            ou, ov = abs(u) - tb["L"] / 2, abs(v) - tb["D"] / 2
            half = ch["D"] / 2
            if ou > half + SEAT_AT_TABLE_M or ov > half + SEAT_AT_TABLE_M:
                continue
            along = ou >= ov
            nu, nv = (math.copysign(1, u), 0.0) if along else (0.0, math.copysign(1, v))
            if (ou if along else ov) < half:                 # overlapping: move out
                pu = nu * (tb["L"] / 2 + half + 0.02) if along else u
                pv = v if along else nv * (tb["D"] / 2 + half + 0.02)
                ch["x"], ch["y"] = tb["x"] + c * pu - s * pv, tb["y"] + s * pu + c * pv
            nx, ny = c * nu - s * nv, s * nu + c * nv
            ch["yaw"] = math.atan2(-nx, ny)                  # +depth (back) away from the table
            ch["at_table"] = True
            break


def box_corners(f, z0=0.0, z1=None):
    c, s = math.cos(f["yaw"]), math.sin(f["yaw"])
    pts = []
    for z in (z0, f["H"] if z1 is None else z1):
        for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            u, v = du * f["L"] / 2, dv * f["D"] / 2
            pts.append((f["x"] + c * u - s * v, f["y"] + s * u + c * v, z))
    return np.array(pts)


def draw(capture, base, inv, cam, frames):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    col = {"chair": "b", "couch": "m", "table": "r", "tv": "g", "bed": "c"}
    fig, ax = plt.subplots(figsize=(9, 9))
    m = (base[:, 2] > 0.05) & (base[:, 2] < 1.8)
    ax.scatter(base[m, 0], base[m, 1], s=0.1, c=base[m, 2], cmap="viridis", alpha=0.4)
    ax.plot(0, 0, "k^", ms=10)
    for f in inv:
        k = box_corners(f)[:4]; k = np.vstack([k, k[:1]])
        ax.plot(k[:, 0], k[:, 1], col[f["kind"]] + "-", lw=2)
        nx, ny = -math.sin(f["yaw"]), math.cos(f["yaw"])
        if f["kind"] in ("chair", "couch"):
            ax.annotate("", xy=(f["x"] + nx * f["D"] * .8, f["y"] + ny * f["D"] * .8), xytext=(f["x"], f["y"]),
                        arrowprops=dict(arrowstyle="->", color=col[f["kind"]]))
        ax.text(f["x"], f["y"], f["kind"] + (" (round)" if f.get("round") else ""), fontsize=8)
    ax.set_aspect("equal"); ax.set_xlim(-0.5, 5); ax.set_ylim(-2.75, 2.75); ax.grid(lw=.3)
    ax.set_title("room inventory, base_footprint (x fwd, y left); arrow = back")
    plt.savefig(os.path.join(capture, "inventory_top.png"), dpi=90)
    # Boxes back into the camera image.
    img = cv2.imread(frames[len(frames) // 2])
    bgr = {"chair": (255, 0, 0), "couch": (255, 0, 255), "table": (0, 0, 255), "tv": (0, 200, 0), "bed": (200, 200, 0)}
    for f in inv:
        P = box_corners(f) - [0, 0, LIVOX_Z]
        uv, _, ok = project(cam, P)
        for i, j in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]:
            if ok[i] and ok[j]:
                cv2.line(img, tuple(map(int, uv[i])), tuple(map(int, uv[j])), bgr[f["kind"]], 1)
    cv2.imwrite(os.path.join(capture, "inventory_front.jpg"), cv2.resize(img, (1280, 800)))


if __name__ == "__main__":
    sys.exit(main())
