#!/usr/bin/env python3
"""Remove accessories absent from this robot from Yahboom's X3 Plus base STL.

The vendor exports the chassis, wheels, T-slot/display, arm pedestal, and sound
board as one binary STL.  This script splits that STL into connected components
and removes only components inside measured accessory regions.  The original
vendor mesh remains alongside the generated file for inspection/regeneration.
"""

from pathlib import Path
import argparse
import struct

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


TRIANGLE_DTYPE = np.dtype([
    ("normal", "<f4", (3,)),
    ("vertices", "<f4", (3, 3)),
    ("attribute", "<u2"),
])


def read_binary_stl(path):
    with path.open("rb") as stream:
        header = stream.read(80)
        count = struct.unpack("<I", stream.read(4))[0]
        triangles = np.fromfile(stream, dtype=TRIANGLE_DTYPE, count=count)
    if len(triangles) != count:
        raise ValueError(f"{path}: expected {count} triangles, read {len(triangles)}")
    return header, triangles


def component_labels(triangles):
    vertices = triangles["vertices"].reshape(-1, 3).copy()
    vertices[vertices == 0] = 0  # make +0 and -0 hash identically
    _, inverse = np.unique(vertices, axis=0, return_inverse=True)

    triangle_ids = np.repeat(np.arange(len(triangles), dtype=np.int32), 3)
    order = np.argsort(inverse, kind="stable")
    sorted_vertices = inverse[order]
    sorted_triangles = triangle_ids[order]
    starts = np.r_[True, sorted_vertices[1:] != sorted_vertices[:-1]]
    first = np.maximum.accumulate(np.where(starts, np.arange(len(order)), 0))
    roots = sorted_triangles[first]
    joins = sorted_triangles != roots
    rows = np.r_[sorted_triangles[joins], roots[joins]]
    cols = np.r_[roots[joins], sorted_triangles[joins]]
    graph = coo_matrix(
        (np.ones(len(rows), dtype=np.uint8), (rows, cols)),
        shape=(len(triangles), len(triangles)),
    ).tocsr()
    _, labels = connected_components(graph, directed=False)
    return labels


def accessory_name(lower, upper):
    # Tall rear extrusion, display, and their hardware. Nothing on the physical
    # robot should extend this high as part of the chassis mesh.
    if upper[2] > 0.20:
        return "display_and_tslot"

    # Vendor arm pedestal on the front (+X) of the upper plate.
    if lower[0] >= 0.058 and lower[2] >= 0.064:
        return "arm_mount"

    # Small PCB and populated components floating over the upper plate.
    if (lower[0] >= -0.031 and upper[0] <= 0.032
            and lower[2] >= 0.080 and upper[2] <= 0.120):
        return "sound_board"

    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    header, triangles = read_binary_stl(args.source)
    labels = component_labels(triangles)
    remove = np.zeros(len(triangles), dtype=bool)
    totals = {}

    for component in np.unique(labels):
        indices = np.flatnonzero(labels == component)
        points = triangles["vertices"][indices].reshape(-1, 3)
        name = accessory_name(points.min(axis=0), points.max(axis=0))
        if name:
            remove[indices] = True
            totals[name] = totals.get(name, 0) + len(indices)

    kept = triangles[~remove]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as stream:
        note = b"X3 Plus chassis; display, T-slot, arm mount and sound board removed"
        stream.write(note[:80].ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(kept)))
        kept.tofile(stream)

    print(f"source triangles: {len(triangles)}")
    for name, count in sorted(totals.items()):
        print(f"removed {name}: {count}")
    print(f"output triangles: {len(kept)}")


if __name__ == "__main__":
    main()
