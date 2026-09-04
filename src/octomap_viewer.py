#!/usr/bin/env python3
"""
octomap_viewer.py — browse and visualize .bt/.ot octomap files from the X3 robot.

Usage
-----
    python3 src/octomap_viewer.py                       # GUI, scans the default dirs
    python3 src/octomap_viewer.py <dir-or-map> [...]    # GUI, scans the given paths

Keys (GUI)
----------
    Mouse Drag       rotate map
    Mouse Wheel      zoom in / out
    Middle Drag      pan
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
    print("ERROR: octomap-python is not installed.")
    print("Please install it with: pip install octomap-python")
    sys.exit(1)

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

DEFAULT_ROOTS = [
    "~/EE_244_Final_Project/maps",
    "~/maps",
    "./maps",
    "~/bags",
    "./bags",
]

def human_size(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024 or unit == "TB":
            return f"{n:.1f} {unit}" if unit != "B" else f"{n:.0f} B"
        n /= 1024
    return f"{n:.1f} TB"

def find_maps(roots) -> list:
    """Find every .bt or .ot file under `roots`."""
    found, seen = [], set()
    for root in roots:
        root = os.path.abspath(os.path.expanduser(root))
        if not os.path.exists(root):
            continue
        if os.path.isfile(root) and root.endswith((".bt", ".ot")):
            if root not in seen:
                seen.add(root)
                found.append(root)
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames.sort()
            for f in sorted(filenames):
                if f.endswith((".bt", ".ot")):
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
        
        self.camera_dist = 10.0
        self.camera_rot = [45.0, 45.0]  # pitch, yaw
        self.camera_pan = [0.0, 0.0]    # x, y

        self.last_pos = None
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

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
        else:
            self.colors = np.zeros((0, 3), dtype=np.float32)
            
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
        near, far = 0.1, 1000.0
        fov = 45.0
        f = 1.0 / np.tan(fov * np.pi / 360.0)
        proj = np.array([
            [f/ratio, 0, 0, 0],
            [0, f, 0, 0],
            [0, 0, (far+near)/(near-far), -1.0],
            [0, 0, (2*far*near)/(near-far), 0]
        ], dtype=np.float32)
        gl.glMultMatrixf(proj.T)
        gl.glMatrixMode(gl.GL_MODELVIEW)

    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()
        
        # Camera transform
        gl.glTranslatef(self.camera_pan[0], self.camera_pan[1], -self.camera_dist)
        gl.glRotatef(self.camera_rot[0], 1.0, 0.0, 0.0)
        gl.glRotatef(self.camera_rot[1], 0.0, 1.0, 0.0)

        # Draw a small grid at origin
        self._draw_grid()

        if len(self.points) == 0:
            return

        gl.glPointSize(3.0)
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glEnableClientState(gl.GL_COLOR_ARRAY)
        
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.points)
        gl.glColorPointer(3, gl.GL_FLOAT, 0, self.colors)
        
        gl.glDrawArrays(gl.GL_POINTS, 0, len(self.points))
        
        gl.glDisableClientState(gl.GL_COLOR_ARRAY)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

    def _draw_grid(self):
        gl.glLineWidth(1.0)
        gl.glBegin(gl.GL_LINES)
        gl.glColor3f(0.3, 0.3, 0.35)
        extent = 5
        for i in range(-extent, extent + 1):
            gl.glVertex3f(i, -extent, 0)
            gl.glVertex3f(i, extent, 0)
            gl.glVertex3f(-extent, i, 0)
            gl.glVertex3f(extent, i, 0)
        # Axes
        gl.glColor3f(1.0, 0.0, 0.0); gl.glVertex3f(0, 0, 0); gl.glVertex3f(1, 0, 0)
        gl.glColor3f(0.0, 1.0, 0.0); gl.glVertex3f(0, 0, 0); gl.glVertex3f(0, 1, 0)
        gl.glColor3f(0.0, 0.0, 1.0); gl.glVertex3f(0, 0, 0); gl.glVertex3f(0, 0, 1)
        gl.glEnd()

    def mousePressEvent(self, event):
        self.last_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.last_pos is None:
            return
        dx = event.x() - self.last_pos.x()
        dy = event.y() - self.last_pos.y()
        self.last_pos = event.pos()

        if event.buttons() & QtCore.Qt.LeftButton:
            self.camera_rot[0] += dy * 0.5
            self.camera_rot[1] += dx * 0.5
            self.update()
        elif event.buttons() & QtCore.Qt.MiddleButton:
            pan_speed = self.camera_dist * 0.002
            self.camera_pan[0] += dx * pan_speed
            self.camera_pan[1] -= dy * pan_speed
            self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.camera_dist *= 0.9
        else:
            self.camera_dist *= 1.1
        self.update()


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
        
        self.setWindowTitle("Octomap Viewer")
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
        self.list_view.selectionModel()
        
        self.info_text = QtWidgets.QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setFont(QtGui.QFont("monospace", 10))
        self.info_text.setMaximumHeight(200)

        left_layout.addWidget(self.list_view)
        left_layout.addWidget(self.info_text)

        # Right panel
        self.gl_widget = OctomapGLWidget()

        splitter.addWidget(left_widget)
        splitter.addWidget(self.gl_widget)
        splitter.setSizes([300, 900])

        # Connections
        self.list_view.selectionModel().selectionChanged.connect(self._on_selection_changed)

        # Init
        self.rescan()

    def rescan(self):
        paths = find_maps(self.roots)
        self.maps = [MapSummary(p) for p in paths]
        self.model = MapListModel(self.maps)
        self.list_view.setModel(self.model)
        self.list_view.selectionModel().selectionChanged.connect(self._on_selection_changed)
        
        if self.maps:
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
            tree = octomap.OcTree(0.1)
            if path.endswith(".bt"):
                success = tree.readBinary(path)
            else:
                success = tree.read(path)
                
            if not success:
                raise RuntimeError("octomap library failed to read the file.")
                
            res = tree.getResolution()
            pts, _ = tree.extractPointCloud()
            
            t1 = time.time()
            
            info = [
                f"File: {os.path.basename(path)}",
                f"Type: {'.bt (binary)' if path.endswith('.bt') else '.ot (full)'}",
                f"Resolution: {res:.4f} m",
                f"Occupied nodes: {len(pts)}",
                f"Load time: {t1 - t0:.2f} s"
            ]
            self.info_text.setPlainText("\n".join(info))
            self.gl_widget.set_points(pts)
            
        except Exception as exc:
            self.info_text.setPlainText(f"Error loading map:\n{exc}")
            self.gl_widget.set_points([])

    def keyPressEvent(self, event):
        k = event.key()
        if k == QtCore.Qt.Key_Escape or k == QtCore.Qt.Key_Q:
            self.close()
        elif k == QtCore.Qt.Key_R:
            self.rescan()
        else:
            super().keyPressEvent(event)


def main():
    parser = argparse.ArgumentParser(description="Browse and visualize .bt/.ot octomap files.")
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
