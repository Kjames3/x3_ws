#!/usr/bin/env python3
"""
octomap_viewer.py — browse and visualize 3D maps from the X3 robot.

Formats
-------
    .bt / .ot        octomap (needs octomap-python)
    .npz             numpy archive; every Nx3 array becomes a selectable layer,
                     so e.g. deskew vs rigid can be compared in place
    .npy             a single Nx3 array
    .pcd             PCL, ascii and binary (not binary_compressed)
    .ply             ascii and binary_little_endian
    .xyz/.pts/.csv   plain text, first three columns

Usage
-----
    python3 src/octomap_viewer.py                       # GUI, scans the default dirs
    python3 src/octomap_viewer.py <dir-or-map> [...]    # GUI, scans the given paths

Mouse (GUI)
-----------
    Left Drag        orbit (the map follows the cursor)
    Right/Middle     pan (screen-relative, 1:1 with the cursor)
    Shift+Left       pan
    Wheel            zoom, proportional to wheel travel

Keys (GUI)
----------
    f                fit the view to the cloud
    t / y            top-down / side-on view
    [ or ]           smaller / larger points
    r                rescan directories
    q or Esc         quit
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time

import numpy as np

try:
    import octomap
except ImportError:
    # Only .bt/.ot need it; numpy/PCD/PLY maps still load without it.
    octomap = None

try:
    import PyQt5.QtCore  # noqa: F401
    from PyQt5 import QtCore, QtGui, QtWidgets
    from PyQt5.QtWidgets import QOpenGLWidget
except ImportError:
    print("ERROR: PyQt5 is not installed.")
    sys.exit(1)

try:
    import OpenGL.GL as gl
except ImportError:
    print("ERROR: PyOpenGL is not installed.")
    sys.exit(1)

_HERE = os.path.dirname(os.path.abspath(__file__))

DEFAULT_ROOTS = [
    "~/EE_244_Final_Project/maps",
    "~/maps",
    os.path.join(_HERE, "yahboomcar_nav", "maps"),
    os.path.join(_HERE, "maps"),
    "./maps",
    "~/bags",
    "./bags",
    os.path.join(os.path.dirname(_HERE), "artifacts"),
]

def human_size(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024 or unit == "TB":
            return f"{n:.1f} {unit}" if unit != "B" else f"{n:.0f} B"
        n /= 1024
    return f"{n:.1f} TB"

CLOUD_EXTS = (".bt", ".ot", ".npz", ".npy", ".pcd", ".ply", ".xyz", ".pts", ".csv")

# npz keys that are Nx3 but are NOT point clouds (tilt joint states, odom, per-cloud metadata)
_NON_CLOUD_KEYS = {"joints", "odom", "meta", "cloud_meta", "cloud_id", "tilt", "rows"}

# Preferred display order when an archive holds several clouds.
_LAYER_PRIORITY = ["deskew", "points", "matched", "rigid", "alternative"]


def _order_layers(layers):
    """Sort (name, array) pairs so the most 'canonical' cloud is first."""
    def key(item):
        try:
            return (_LAYER_PRIORITY.index(item[0]), item[0])
        except ValueError:
            return (len(_LAYER_PRIORITY), item[0])
    return sorted(layers, key=key)


def _as_xyz(arr):
    """Return an (N,3) float32 view of `arr`, or None if it is not a cloud."""
    a = np.asarray(arr)
    if a.ndim != 2 or a.shape[0] < 4 or a.shape[1] < 3:
        return None
    if not np.issubdtype(a.dtype, np.floating) and not np.issubdtype(a.dtype, np.integer):
        return None
    return np.ascontiguousarray(a[:, :3], dtype=np.float32)


def _load_octomap(path):
    if octomap is None:
        raise RuntimeError(
            "octomap-python is not installed, so .bt/.ot cannot be read.\n"
            "Install it with: pip install octomap-python"
        )
    tree = octomap.OcTree(0.1)
    ok = tree.readBinary(path) if path.endswith(".bt") else tree.read(path)
    if not ok:
        raise RuntimeError("octomap library failed to read the file.")
    pts, _ = tree.extractPointCloud()
    info = {"Resolution": f"{tree.getResolution():.4f} m"}
    return [("occupied", np.asarray(pts, dtype=np.float32))], info


def _load_npz(path):
    d = np.load(path, allow_pickle=False)
    layers, skipped = [], []
    for k in d.files:
        if k in _NON_CLOUD_KEYS:
            skipped.append(k)
            continue
        xyz = _as_xyz(d[k])
        if xyz is None:
            skipped.append(k)
        else:
            layers.append((k, xyz))
    if not layers:
        raise RuntimeError(
            "no Nx3 point array in this archive.\nKeys: " + ", ".join(d.files)
        )
    info = {}
    if skipped:
        info["Other keys"] = ", ".join(skipped)
    return _order_layers(layers), info


def _load_npy(path):
    xyz = _as_xyz(np.load(path, allow_pickle=False))
    if xyz is None:
        raise RuntimeError("array is not Nx3.")
    return [("points", xyz)], {}


_PCD_TYPES = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2",
              ("U", 4): "u4", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}


def _load_pcd(path):
    fields, sizes, types, counts, npts, fmt = [], [], [], [], 0, "ascii"
    with open(path, "rb") as fh:
        while True:
            raw = fh.readline()
            if not raw:
                raise RuntimeError("PCD header ended without a DATA line.")
            line = raw.decode("ascii", "replace").strip()
            if not line or line.startswith("#"):
                continue
            tok = line.split()
            key = tok[0].upper()
            if key == "FIELDS":
                fields = tok[1:]
            elif key == "SIZE":
                sizes = [int(v) for v in tok[1:]]
            elif key == "TYPE":
                types = tok[1:]
            elif key == "COUNT":
                counts = [int(v) for v in tok[1:]]
            elif key == "POINTS":
                npts = int(tok[1])
            elif key == "DATA":
                fmt = tok[1].lower()
                break
        if not counts:
            counts = [1] * len(fields)
        for ax in ("x", "y", "z"):
            if ax not in fields:
                raise RuntimeError(f"PCD has no '{ax}' field (fields: {fields}).")
        if fmt == "binary_compressed":
            raise RuntimeError("binary_compressed PCD is not supported.")
        if fmt == "ascii":
            data = np.loadtxt(fh, dtype=np.float64, ndmin=2)
            cols = []
            for f, c in zip(fields, counts):
                cols.extend([f] * c)
            idx = [cols.index(ax) for ax in ("x", "y", "z")]
            xyz = data[:, idx]
        else:
            dt = []
            for f, s, t, c in zip(fields, sizes, types, counts):
                np_t = _PCD_TYPES.get((t.upper(), s))
                if np_t is None:
                    raise RuntimeError(f"unsupported PCD field type {t}{s}.")
                dt.append((f, np_t, (c,)) if c > 1 else (f, np_t))
            rec = np.frombuffer(fh.read(), dtype=np.dtype(dt), count=npts)
            xyz = np.column_stack([rec["x"], rec["y"], rec["z"]])
    xyz = np.asarray(xyz, dtype=np.float32)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return [("points", xyz)], {"Fields": " ".join(fields), "Data": fmt}


_PLY_TYPES = {"float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
              "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
              "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
              "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4"}


def _load_ply(path):
    props, nvert, fmt, in_vertex = [], 0, "ascii", False
    with open(path, "rb") as fh:
        if fh.readline().strip() != b"ply":
            raise RuntimeError("not a PLY file.")
        while True:
            raw = fh.readline()
            if not raw:
                raise RuntimeError("PLY header ended without end_header.")
            tok = raw.decode("ascii", "replace").strip().split()
            if not tok:
                continue
            if tok[0] == "format":
                fmt = tok[1]
            elif tok[0] == "element":
                in_vertex = tok[1] == "vertex"
                if in_vertex:
                    nvert = int(tok[2])
            elif tok[0] == "property" and in_vertex:
                if tok[1] == "list":
                    raise RuntimeError("list properties on vertices are not supported.")
                props.append((tok[2], _PLY_TYPES.get(tok[1], "f4")))
            elif tok[0] == "end_header":
                break
        names = [p[0] for p in props]
        for ax in ("x", "y", "z"):
            if ax not in names:
                raise RuntimeError(f"PLY vertices have no '{ax}' property.")
        if fmt == "ascii":
            data = np.loadtxt(fh, dtype=np.float64, max_rows=nvert, ndmin=2)
            xyz = data[:, [names.index(a) for a in ("x", "y", "z")]]
        elif fmt == "binary_little_endian":
            rec = np.frombuffer(fh.read(), dtype=np.dtype(props), count=nvert)
            xyz = np.column_stack([rec["x"], rec["y"], rec["z"]])
        else:
            raise RuntimeError(f"PLY format '{fmt}' is not supported.")
    xyz = np.asarray(xyz, dtype=np.float32)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return [("points", xyz)], {"Format": fmt}


def _load_text(path):
    delim = "," if path.endswith(".csv") else None
    data = np.loadtxt(path, delimiter=delim, comments="#", ndmin=2)
    xyz = _as_xyz(data)
    if xyz is None:
        raise RuntimeError(f"expected at least 3 numeric columns, got shape {data.shape}.")
    return [("points", xyz)], {}


def load_cloud(path):
    """Load `path` into [(layer_name, (N,3) float32), ...] plus an info dict.

    Supported: .bt/.ot (octomap), .npz/.npy (numpy), .pcd, .ply, .xyz/.pts/.csv.
    An .npz may hold several clouds (e.g. deskew vs rigid) — each becomes a layer.
    """
    ext = os.path.splitext(path)[1].lower()
    loader = {
        ".bt": _load_octomap, ".ot": _load_octomap,
        ".npz": _load_npz, ".npy": _load_npy,
        ".pcd": _load_pcd, ".ply": _load_ply,
        ".xyz": _load_text, ".pts": _load_text, ".csv": _load_text,
    }.get(ext)
    if loader is None:
        raise RuntimeError(f"unsupported extension '{ext}'.")
    return loader(path)


def find_maps(roots) -> list:
    """Find every supported map file under `roots`."""
    found, seen = [], set()
    for root in roots:
        root = os.path.abspath(os.path.expanduser(root))
        if not os.path.exists(root):
            continue
        if os.path.isfile(root) and root.lower().endswith(CLOUD_EXTS):
            if root not in seen:
                seen.add(root)
                found.append(root)
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames.sort()
            for f in sorted(filenames):
                if f.lower().endswith(CLOUD_EXTS):
                    p = os.path.join(dirpath, f)
                    if p not in seen:
                        seen.add(p)
                        found.append(p)
    found.sort(key=lambda p: os.path.basename(p))
    return found


class MapSummary:
    def __init__(self, path: str):
        self.path = path
        self.name = os.path.basename(path)
        self.size = 0
        try:
            self.size = os.path.getsize(path)
        except OSError:
            pass

    def label(self) -> str:
        return f"{self.name}\n   {human_size(self.size)}"


class OctomapGLWidget(QOpenGLWidget):
    """3D point cloud renderer using PyOpenGL."""

    def __init__(self):
        super().__init__()
        self.points = np.zeros((0, 3), dtype=np.float32)
        self.colors = np.zeros((0, 3), dtype=np.float32)
        
        self.fov = 45.0
        self.camera_dist = 10.0
        self.elev = 35.0                            # deg above the horizon
        self.yaw = 45.0                             # deg about +Z
        self.target = np.zeros(3, dtype=np.float64)  # what the orbit turns around
        self.point_size = 2.0

        self.last_pos = None
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setMouseTracking(False)

    def set_points(self, points):
        self.points = np.asarray(points, dtype=np.float32)
        # Give points a nice depth-based color map or just uniform color
        if len(self.points) > 0:
            z = self.points[:, 2]
            z_min, z_max = z.min(), z.max()
            span = z_max - z_min if z_max > z_min else 1.0
            z_norm = (z - z_min) / span
            # Turbo-like or just green-to-blue gradient
            self.colors = np.column_stack((
                np.clip(1.0 - z_norm, 0, 1),
                np.ones_like(z_norm) * 0.8,
                np.clip(z_norm, 0, 1)
            )).astype(np.float32)
            self.fit_camera()
        else:
            self.colors = np.zeros((0, 3), dtype=np.float32)
            self.update()

    def fit_camera(self):
        """Frame the whole cloud: centre on its bounding box and pull back to fit.

        Uses percentiles rather than min/max so a handful of stray returns (a
        single 8 m ceiling hit, a NaN-adjacent outlier) cannot push the camera
        kilometres away and leave the map a dot in the middle of the screen.
        """
        if len(self.points) == 0:
            return
        lo = np.percentile(self.points, 0.5, axis=0)
        hi = np.percentile(self.points, 99.5, axis=0)
        self.target = 0.5 * (lo + hi)
        radius = 0.5 * float(np.linalg.norm(hi - lo))
        # Distance that puts a sphere of `radius` inside the vertical fov.
        self.camera_dist = max(radius / np.tan(np.radians(self.fov) * 0.5), 1.0)
        self.update()

    def initializeGL(self):
        gl.glClearColor(0.12, 0.14, 0.18, 1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_PROGRAM_POINT_SIZE)

    def resizeGL(self, w, h):
        gl.glViewport(0, 0, w, h)
        self._set_projection(w, h)

    def _set_projection(self, w, h):
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        ratio = w / float(h) if h > 0 else 1.0
        # Scale the depth range with the zoom level: a fixed 0.1 near plane
        # clips the map away when you zoom in close, and a fixed 1000 far
        # plane wastes depth precision on a 5 m room.
        near = max(1e-3, self.camera_dist * 0.005)
        far = max(near * 1e4, self.camera_dist * 50.0)
        fov = self.fov
        f = 1.0 / np.tan(fov * np.pi / 360.0)
        zf = (far + near) / (near - far)
        zd = (2 * far * near) / (near - far)
        proj = np.array([
            [f / ratio, 0, 0, 0],
            [0, f, 0, 0],
            [0, 0, zf, zd],
            [0, 0, -1.0, 0],
        ], dtype=np.float32)
        gl.glMultMatrixf(proj.T)
        gl.glMatrixMode(gl.GL_MODELVIEW)

    def paintGL(self):
        # The projection depends on camera_dist, so refresh it every frame.
        self._set_projection(self.width(), self.height())
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()
        
        # Camera transform: spherical orbit around the target point.
        # camera_rot[0] = elevation (deg, 0 level .. 90 top-down), camera_rot[1] = yaw.
        cam = self._orbit_view()
        gl.glMultMatrixf(cam)

        # Draw a small grid at origin
        self._draw_grid()

        if len(self.points) == 0:
            return

        gl.glPointSize(self.point_size)
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glEnableClientState(gl.GL_COLOR_ARRAY)
        
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.points)
        gl.glColorPointer(3, gl.GL_FLOAT, 0, self.colors)
        
        gl.glDrawArrays(gl.GL_POINTS, 0, len(self.points))
        
        gl.glDisableClientState(gl.GL_COLOR_ARRAY)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

    def _basis(self):
        """(eye, forward, screen-right, screen-up) for the current orbit."""
        th = np.radians(float(self.elev))
        ph = np.radians(float(self.yaw))
        eye = self.target + max(self.camera_dist, 1e-6) * np.array([
            np.cos(th) * np.cos(ph),
            np.cos(th) * np.sin(ph),
            np.sin(th),
        ])
        fwd = self.target - eye
        fwd /= np.linalg.norm(fwd)
        side = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
        n = np.linalg.norm(side)
        # Straight down/up: any horizontal axis is a valid right vector.
        side = side / n if n > 1e-9 else np.array([0.0, 1.0, 0.0])
        return eye, fwd, side, np.cross(side, fwd)

    def _orbit_view(self):
        """Column-major OGL modelview for a spherical orbit around the target."""
        eye, fwd, side, up2 = self._basis()
        V = np.array([
            [side[0], up2[0], -fwd[0], -side @ eye],
            [side[1], up2[1], -fwd[1], -up2 @ eye],
            [side[2], up2[2], -fwd[2], fwd @ eye],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        return V.T

    def _draw_grid(self):
        """Floor grid sized to the current zoom and centred under the target.

        A fixed +-5 m grid at the world origin is worse than none: on a large
        or offset map it is an unrelated postage stamp off to one side, which
        is exactly what makes the motion read as wrong.
        """
        # ~1, 2 or 5 x 10^n, so the grid stays 10-40 cells wide at any zoom.
        raw = max(self.camera_dist, 1e-3) / 10.0
        mag = 10.0 ** np.floor(np.log10(raw))
        step = float(mag * min([1, 2, 5, 10], key=lambda m: abs(raw - m * mag)))
        n = 12
        cx = np.round(self.target[0] / step) * step
        cy = np.round(self.target[1] / step) * step
        gl.glLineWidth(1.0)
        gl.glBegin(gl.GL_LINES)
        gl.glColor3f(0.3, 0.3, 0.35)
        for i in range(-n, n + 1):
            x, y = cx + i * step, cy + i * step
            gl.glVertex3f(x, cy - n * step, 0)
            gl.glVertex3f(x, cy + n * step, 0)
            gl.glVertex3f(cx - n * step, y, 0)
            gl.glVertex3f(cx + n * step, y, 0)
        # Axes, scaled so they stay visible when zoomed out.
        a = step
        # Axes
        gl.glColor3f(1.0, 0.0, 0.0); gl.glVertex3f(0, 0, 0); gl.glVertex3f(a, 0, 0)
        gl.glColor3f(0.0, 1.0, 0.0); gl.glVertex3f(0, 0, 0); gl.glVertex3f(0, a, 0)
        gl.glColor3f(0.0, 0.0, 1.0); gl.glVertex3f(0, 0, 0); gl.glVertex3f(0, 0, a)
        gl.glEnd()

    def _is_pan_drag(self, event):
        # Right and middle both pan; shift+left too, for trackpads with neither.
        return bool(event.buttons() & (QtCore.Qt.MiddleButton | QtCore.Qt.RightButton)) or (
            bool(event.buttons() & QtCore.Qt.LeftButton)
            and bool(event.modifiers() & QtCore.Qt.ShiftModifier)
        )

    def mousePressEvent(self, event):
        self.setFocus()
        self.last_pos = event.pos()

    def mouseReleaseEvent(self, event):
        self.last_pos = None

    def mouseMoveEvent(self, event):
        if self.last_pos is None:
            return
        dx = event.x() - self.last_pos.x()
        dy = event.y() - self.last_pos.y()
        self.last_pos = event.pos()

        if self._is_pan_drag(event):
            # Screen-relative pan: move the target along the camera's own right
            # and up axes, scaled so the map tracks the cursor 1:1 in pixels.
            _, _, side, up2 = self._basis()
            h = max(self.height(), 1)
            scale = 2.0 * self.camera_dist * np.tan(np.radians(self.fov) * 0.5) / h
            self.target = self.target - side * (dx * scale) + up2 * (dy * scale)
            self.update()
        elif event.buttons() & QtCore.Qt.LeftButton:
            # Drag down tips the map towards you (camera climbs), drag right
            # swings the map to the right - i.e. the cursor drags the map, the
            # way RViz and every mesh viewer behave.
            self.elev = max(-89.0, min(89.0, self.elev + dy * 0.4))
            self.yaw -= dx * 0.4
            self.update()

    def wheelEvent(self, event):
        # Proportional to actual wheel travel, so a fast flick and a slow one
        # move the same total distance, and high-res trackpads do not crawl.
        steps = event.angleDelta().y() / 120.0
        if steps == 0:
            return
        self.camera_dist = float(np.clip(self.camera_dist * (0.85 ** steps), 1e-3, 1e5))
        self.update()

    def keyPressEvent(self, event):
        k = event.key()
        if k == QtCore.Qt.Key_F:
            self.fit_camera()
        elif k == QtCore.Qt.Key_T:
            self.elev, self.yaw = 89.0, 90.0
            self.update()
        elif k == QtCore.Qt.Key_Y:
            self.elev, self.yaw = 0.0, 180.0
            self.update()
        elif k in (QtCore.Qt.Key_BracketLeft, QtCore.Qt.Key_BracketRight):
            step = 1.0 if k == QtCore.Qt.Key_BracketRight else -1.0
            self.point_size = float(np.clip(self.point_size + step, 1.0, 12.0))
            self.update()
        else:
            event.ignore()


class MapListModel(QtCore.QAbstractListModel):
    def __init__(self, maps, parent=None):
        super().__init__(parent)
        self.maps = maps

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.maps)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid():
            return QtCore.QVariant()
        m = self.maps[index.row()]
        if role == QtCore.Qt.DisplayRole:
            return m.label()
        if role == QtCore.Qt.UserRole:
            return m
        return QtCore.QVariant()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, roots):
        super().__init__()
        self.roots = roots
        self.current_map_path = None
        self.layers = []

        self.setWindowTitle("3D Map Viewer")
        self.resize(1200, 800)

        # UI Layout
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        layout = QtWidgets.QHBoxLayout(main_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        layout.addWidget(splitter)

        # Left panel
        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget)
        left_layout.setContentsMargins(4, 4, 4, 4)

        self.list_view = QtWidgets.QListView()
        self.list_view.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.list_view.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        
        # Layer picker: an .npz often holds several clouds of the same scene
        # (deskew vs rigid), and flipping between them is the whole point.
        self.layer_box = QtWidgets.QComboBox()
        self.layer_box.setVisible(False)
        self.layer_box.currentIndexChanged.connect(self._on_layer_changed)

        self.info_text = QtWidgets.QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setFont(QtGui.QFont("monospace", 10))
        self.info_text.setMaximumHeight(200)

        left_layout.addWidget(self.list_view)
        left_layout.addWidget(self.layer_box)
        left_layout.addWidget(self.info_text)

        # Right panel
        self.gl_widget = OctomapGLWidget()

        splitter.addWidget(left_widget)
        splitter.addWidget(self.gl_widget)
        splitter.setSizes([300, 900])

        # Connections
        self.rescan()

    def rescan(self):
        paths = find_maps(self.roots)
        self.maps = [MapSummary(p) for p in paths]
        self.model = MapListModel(self.maps)
        self.list_view.setModel(self.model)
        self.list_view.selectionModel().selectionChanged.connect(self._on_selection_changed)
        
        if not self.maps:
            searched = "\n".join(
                f"  - {os.path.expanduser(r)}" for r in self.roots
            )
            self.info_text.setPlainText(
                "No 3D maps found (" + ", ".join(CLOUD_EXTS) + ").\n\n"
                "Searched:\n" + searched + "\n\n"
                "Copy a map into one of those dirs (or pass it as an "
                "argument) and press R."
            )
        else:
            self.list_view.setCurrentIndex(self.model.index(0, 0))
            self.list_view.setFocus()
            
    def _on_selection_changed(self, selected, deselected):
        indexes = self.list_view.selectionModel().selectedIndexes()
        if not indexes:
            return
        m = self.model.data(indexes[0], QtCore.Qt.UserRole)
        self._load_map(m.path)

    def _load_map(self, path):
        if not os.path.exists(path):
            self.info_text.setPlainText(f"File not found:\n{path}")
            return

        self.current_map_path = path
        self.info_text.setPlainText(f"Loading {os.path.basename(path)}...")
        QtWidgets.QApplication.processEvents()

        t0 = time.time()
        try:
            layers, extra = load_cloud(path)
        except Exception as exc:
            self.layers = []
            self.layer_box.setVisible(False)
            self.info_text.setPlainText(f"Error loading map:\n{exc}")
            self.gl_widget.set_points([])
            return

        self.load_secs = time.time() - t0
        self.load_info = extra
        self.layers = layers

        self.layer_box.blockSignals(True)
        self.layer_box.clear()
        for name, pts in layers:
            self.layer_box.addItem(f"{name}  ({len(pts):,} pts)")
        self.layer_box.setCurrentIndex(0)
        self.layer_box.blockSignals(False)
        self.layer_box.setVisible(len(layers) > 1)
        self._show_layer(0)

    def _on_layer_changed(self, index):
        if 0 <= index < len(self.layers):
            self._show_layer(index)

    def _show_layer(self, index):
        name, pts = self.layers[index]
        path = self.current_map_path
        info = [
            f"File: {os.path.basename(path)}",
            f"Type: {os.path.splitext(path)[1] or '?'}",
            f"Points: {len(pts):,}",
        ]
        if len(self.layers) > 1:
            info.append(f"Layer: {name}  ({index + 1}/{len(self.layers)})")
        if len(pts):
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            info.append("Bounds x: %.2f .. %.2f m" % (lo[0], hi[0]))
            info.append("       y: %.2f .. %.2f m" % (lo[1], hi[1]))
            info.append("       z: %.2f .. %.2f m" % (lo[2], hi[2]))
        for k, v in self.load_info.items():
            info.append(f"{k}: {v}")
        info.append(f"Load time: {self.load_secs:.2f} s")
        self.info_text.setPlainText("\n".join(info))
        self.gl_widget.set_points(pts)

    def keyPressEvent(self, event):
        k = event.key()
        if k == QtCore.Qt.Key_Escape or k == QtCore.Qt.Key_Q:
            self.close()
        elif k == QtCore.Qt.Key_R:
            self.rescan()
        else:
            super().keyPressEvent(event)


def main():
    parser = argparse.ArgumentParser(
        description="Browse and visualize 3D maps (" + ", ".join(CLOUD_EXTS) + ").")
    parser.add_argument("paths", nargs="*", default=DEFAULT_ROOTS,
                        help="Directories or files to scan for maps")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    
    # Modern dark theme for PyQt5
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(30, 32, 36))
    palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(20, 22, 26))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(30, 32, 36))
    palette.setColor(QtGui.QPalette.ToolTipBase, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.ToolTipText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Text, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor(45, 50, 58))
    palette.setColor(QtGui.QPalette.ButtonText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.HighlightedText, QtCore.Qt.white)
    app.setPalette(palette)

    win = MainWindow(args.paths)
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
