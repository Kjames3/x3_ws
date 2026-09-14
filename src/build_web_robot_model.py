#!/usr/bin/env python3
"""Bake the X3 URDF into a small GLB for the GUI's Surroundings view.

The URDF's CAD meshes are far too heavy to stream over WiFi (base_link alone is
14 MB), so every visual is decimated by vertex clustering and baked into one of
two nodes:

  body  -- everything rigidly attached to base_link, in base_link's frame
  tilt  -- the lidar_tilt_link subtree, in lidar_tilt_link's frame, with the
           node transform set to lidar_tilt_joint's origin so the GUI only has
           to rotate it about its local +Y by readout.tilt.deg

Frames stay URDF (x forward, y left, z up, metres); scene-view.js converts.

    python3 src/build_web_robot_model.py            # -> src/web/models/x3_robot.glb
"""
import argparse
import os
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from trimesh.transformations import euler_matrix, translation_matrix

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "yahboomcar_description")
URDF = os.path.join(PKG, "urdf", "yahboomcar_X3.urdf")
OUT = os.path.join(HERE, "web", "models", "x3_robot.glb")
TILT_JOINT = "lidar_tilt_joint"
DEFAULT_RGBA = (0.35, 0.37, 0.40, 1.0)


def origin_matrix(el):
    o = el.find("origin") if el is not None else None
    if o is None:
        return np.eye(4)
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    return translation_matrix(xyz) @ euler_matrix(*rpy, axes="sxyz")


def cluster_decimate(mesh, cell):
    """Vertex clustering: snap vertices to a grid, merge, drop collapsed faces."""
    keys = np.floor(mesh.vertices / cell).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    n = inverse.max() + 1
    verts = np.zeros((n, 3))
    np.add.at(verts, inverse, mesh.vertices)
    verts /= np.bincount(inverse, minlength=n)[:, None]
    faces = inverse[mesh.faces]
    ok = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    out = trimesh.Trimesh(verts, faces[ok], process=False)
    out.fix_normals()
    return out


def load_geometry(geom, cell):
    m = geom.find("mesh")
    if m is not None:
        path = m.get("filename").replace("package://yahboomcar_description/", "")
        mesh = trimesh.load(os.path.join(PKG, path), force="mesh")
        if m.get("scale"):
            mesh.apply_scale([float(v) for v in m.get("scale").split()])
        before = len(mesh.faces)
        mesh = cluster_decimate(mesh, cell)
        print(f"  {path}: {before} -> {len(mesh.faces)} faces")
        return mesh
    if geom.find("box") is not None:
        return trimesh.creation.box([float(v) for v in geom.find("box").get("size").split()])
    if geom.find("cylinder") is not None:
        c = geom.find("cylinder")
        return trimesh.creation.cylinder(float(c.get("radius")), float(c.get("length")))
    if geom.find("sphere") is not None:
        return trimesh.creation.icosphere(radius=float(geom.find("sphere").get("radius")))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell-mm", type=float, default=3.0, help="clustering grid (mm)")
    args = ap.parse_args()
    cell = args.cell_mm / 1000.0

    root = ET.parse(URDF).getroot()
    colors = {m.get("name"): m.find("color").get("rgba")
              for m in root.findall("material") if m.find("color") is not None}
    children = {}
    for j in root.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j)

    # link -> (group, transform into that group's frame)
    placed = {"base_link": ("body", np.eye(4))}
    tilt_origin = None
    stack = ["base_link"]
    while stack:
        parent = stack.pop()
        group, T = placed[parent]
        for j in children.get(parent, []):
            child = j.find("child").get("link")
            if j.get("name") == TILT_JOINT:
                tilt_origin = T @ origin_matrix(j)
                placed[child] = ("tilt", np.eye(4))
            else:
                placed[child] = (group, T @ origin_matrix(j))
            stack.append(child)
    if tilt_origin is None:
        raise SystemExit(f"{TILT_JOINT} not found under base_link")

    parts = {"body": [], "tilt": []}
    for link in root.findall("link"):
        if link.get("name") not in placed:
            continue
        group, T = placed[link.get("name")]
        for vis in link.findall("visual"):
            geom = vis.find("geometry")
            mesh = load_geometry(geom, cell) if geom is not None else None
            if mesh is None:
                continue
            mesh.apply_transform(T @ origin_matrix(vis))
            rgba = DEFAULT_RGBA
            mat = vis.find("material")
            if mat is not None:
                c = mat.find("color")
                txt = c.get("rgba") if c is not None else colors.get(mat.get("name"))
                if txt:
                    rgba = tuple(float(v) for v in txt.split())
            mesh.visual = trimesh.visual.ColorVisuals(
                mesh, face_colors=np.tile(np.array(rgba) * 255, (len(mesh.faces), 1)).astype(np.uint8))
            parts[group].append(mesh)

    scene = trimesh.Scene()
    scene.add_geometry(trimesh.util.concatenate(parts["body"]), node_name="body", geom_name="body")
    scene.add_geometry(trimesh.util.concatenate(parts["tilt"]), node_name="tilt", geom_name="tilt",
                       transform=tilt_origin)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    data = scene.export(file_type="glb")
    with open(OUT, "wb") as f:
        f.write(data)
    print(f"wrote {OUT} ({len(data) / 1024:.0f} KB, "
          f"body {sum(len(m.faces) for m in parts['body'])} / tilt {sum(len(m.faces) for m in parts['tilt'])} faces)")


if __name__ == "__main__":
    main()
