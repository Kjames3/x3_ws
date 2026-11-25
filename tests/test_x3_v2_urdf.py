"""Contract for x3_v2.urdf.xacro: the frame names the runtime uses, and the
geometry invariants the CAD script is supposed to guarantee.

Expands the xacro against the SOURCE tree (a throwaway ament index), so it
needs ROS's `xacro` on PATH; skipped otherwise.
"""
import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / "src" / "yahboomcar_description"
XACRO = DESC / "urdf" / "x3_v2.urdf.xacro"
CAD = yaml.safe_load(open(DESC / "config" / "x3_v2_cad_frames.yaml"))

RUNTIME_FRAMES = [
    "base_footprint", "base_link", "imu_link", "icm_imu_link",
    "front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel",
    "livox_mount_link", "livox_frame", "laser_link",
    "oak-d-base-frame", "oak_rgb_camera_frame", "oak_rgb_camera_optical_frame",
    "oak_left_camera_optical_frame", "oak_right_camera_optical_frame", "oak_imu_frame",
    "oak_rear-d-base-frame", "oak_rear_rgb_camera_optical_frame",
    "oak_rear_left_camera_optical_frame", "oak_rear_right_camera_optical_frame",
]
WHEEL_JOINTS = ["front_left_joint", "front_right_joint", "back_left_joint", "back_right_joint"]


@pytest.fixture(scope="module")
def urdf(tmp_path_factory):
    xacro = shutil.which("xacro") or "/opt/ros/humble/bin/xacro"
    if not os.path.exists(xacro):
        pytest.skip("xacro not installed")
    idx = tmp_path_factory.mktemp("ament")
    (idx / "share" / "ament_index" / "resource_index" / "packages").mkdir(parents=True)
    (idx / "share" / "ament_index" / "resource_index" / "packages" / "yahboomcar_description").touch()
    (idx / "share" / "yahboomcar_description").symlink_to(DESC)
    env = dict(os.environ, AMENT_PREFIX_PATH=f"{idx}:{os.environ.get('AMENT_PREFIX_PATH', '/opt/ros/humble')}")
    out = subprocess.run([xacro, str(XACRO)], env=env, capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    return ET.fromstring(out.stdout)


def _rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _fk(root):
    """frame -> 4x4 pose in base_footprint."""
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        T = np.eye(4)
        T[:3, :3] = _rot(*rpy)
        T[:3, 3] = xyz
        joints[j.find("child").get("link")] = (j.find("parent").get("link"), T)
    poses = {}

    def pose(link):
        if link not in poses:
            if link not in joints:
                poses[link] = np.eye(4)
            else:
                parent, T = joints[link]
                poses[link] = pose(parent) @ T
        return poses[link]
    for link in [l.get("name") for l in root.findall("link")]:
        pose(link)
    return poses, joints


def test_runtime_frames_exist_under_one_root(urdf):
    links = {l.get("name") for l in urdf.findall("link")}
    missing = [f for f in RUNTIME_FRAMES if f not in links]
    assert not missing, missing
    _, joints = _fk(urdf)
    roots = links - set(joints)
    assert roots == {"base_footprint"}
    names = {j.get("name") for j in urdf.findall("joint")}
    assert set(WHEEL_JOINTS) <= names            # Mcnamu_driver_X3 publishes these


def test_wheels_level_symmetric_and_in_their_quadrants(urdf):
    poses, _ = _fk(urdf)
    r = CAD["wheel_radius"]
    assert 0.050 < r < 0.054                      # 104 mm mecanum wheels
    for name in ("front_left", "front_right", "back_left", "back_right"):
        p = poses[f"{name}_wheel"][:3, 3]
        assert abs(p[2] - r) < 1e-4, name         # levelled: every centre at z = r
        assert (p[0] > 0) == name.startswith("front") and (p[1] > 0) == name.endswith("left")
        assert abs(abs(p[0]) - CAD["wheelbase"] / 2) < 0.003, name
        assert abs(abs(p[1]) - CAD["track"] / 2) < 0.003, name
    c = np.mean([poses[f"{n}_wheel"][:3, 3][:2] for n in
                 ("front_left", "front_right", "back_left", "back_right")], axis=0)
    assert np.abs(c).max() < 1e-3                 # base_footprint under the wheel centroid


def test_livox_and_laser(urdf):
    poses, _ = _fk(urdf)
    liv = poses["livox_frame"]
    las = poses["laser_link"]
    hb = CAD["livox_housing"]
    assert abs(liv[2, 3] - 0.2731) < 1e-6         # floor fit, 2026-11-18
    assert hb["base_z"] < liv[2, 3] < hb["top_z"]  # optical centre inside the housing
    assert np.allclose(liv[:3, :3], np.eye(3))
    assert np.allclose(las[:3, 3], liv[:3, 3])    # /scan ranges measured from the Livox
    yaw = math.atan2(las[1, 0], las[0, 0]) % (2 * math.pi)
    assert abs(yaw - 3.226241) < 1e-6             # old laser_link convention kept
    assert np.hypot(*liv[:2, 3]) < 0.02           # Livox sits over the wheel centroid
    icm = poses["icm_imu_link"][:3, 3]
    assert np.hypot(*(icm[:2] - liv[:2, 3])) < 0.03 and icm[2] < liv[2, 3]   # below the Livox


def test_oaks_face_out_of_their_ends(urdf):
    poses, _ = _fk(urdf)
    # nose-up, rad: front from the Livox calibration (2026-11-20 lab), rear from the 2026-11-24 crowd-run correction
    measured_up = {"oak": 0.022810, "oak_rear": 0.1019}
    for name, sign in (("oak", 1), ("oak_rear", -1)):
        T = poses[f"{name}_rgb_camera_optical_frame"]
        view = T[:3, 2]                            # optical +Z = viewing direction
        assert sign * view[0] > 0.9, (name, view)
        assert abs(view[2] - math.sin(measured_up[name])) < 1e-4, (name, view)   # tilted UP
        assert sign * T[0, 3] > 0.3                # at that end of the chassis
        assert 0.10 < T[2, 3] < 0.25
        left = poses[f"{name}_left_camera_optical_frame"][:3, 3]
        right = poses[f"{name}_right_camera_optical_frame"][:3, 3]
        assert abs(np.linalg.norm(left - right) - 0.075) < 1e-6
        # robot-left mono is on the camera's left: +y for the front camera, -y for the rear
        assert sign * (left[1] - right[1]) > 0


def test_front_oak_matches_lidar_calibration(urdf):
    """livox_frame -> oak_rgb_camera_optical_frame must equal the JSON's T_lidar_camera."""
    import json
    from scipy.spatial.transform import Rotation
    poses, _ = _fk(urdf)
    T = np.linalg.inv(poses["livox_frame"]) @ poses["oak_rgb_camera_optical_frame"]
    d = json.load(open(ROOT / "config/camera_lidar_extrinsics_2026-11-20.json"))["front"]["T_lidar_camera"]
    assert np.allclose(T[:3, 3], d[:3], atol=2e-4)
    dR = Rotation.from_matrix(T[:3, :3]).inv() * Rotation.from_quat(d[3:])
    assert np.degrees(dR.magnitude()) < 0.01


def test_locked_design_v2_geometry():
    """Design_v2 (2026-11-18): its own new_x3.urdf put the wheels at
    x +-0.267, y +-0.2636, r 0.0521; the front is the Top_bus_light end."""
    assert abs(CAD["wheelbase"] - 0.5338) < 0.002 and abs(CAD["track"] - 0.5272) < 0.002
    assert CAD["oak_front"]["glass_centre"][0] > 0.3 and CAD["oak_rear"]["glass_centre"][0] < -0.3


def test_footprints_cover_the_cad_extent():
    """CBF and Nav2 outlines must contain the chassis, not the old frame."""
    import re
    ext = CAD["extent"]
    srv = (ROOT / "src" / "server_x3.py").read_text()
    m = re.search(r"X3_CBF_FOOTPRINT', '([0-9.,]+)'", srv)
    front, rear, left, right = (float(v) for v in m.group(1).split(","))
    assert front >= ext["x"][1] and rear >= -ext["x"][0]
    assert left >= ext["y"][1] and right >= -ext["y"][0]
    assert front < ext["x"][1] + 0.05 and left < ext["y"][1] + 0.05     # not a stale, larger frame
    nav = yaml.safe_load(open(ROOT / "src" / "yahboomcar_nav" / "params" / "nav2_params_x3.yaml"))
    for cm in ("local_costmap", "global_costmap"):
        p = nav[cm][cm]["ros__parameters"]
        fp = np.array(yaml.safe_load(p["footprint"]))
        assert np.allclose(np.abs(fp), [[front, left]] * 4)
        assert p["inflation_layer"]["inflation_radius"] >= math.hypot(front, left)


def test_hard_coded_lidar_x_matches_the_urdf(urdf):
    """_scan_cb (CBF obstacles) and scene-view.js place /scan without TF."""
    import re
    poses, _ = _fk(urdf)
    laser_x_base_link = poses["laser_link"][0, 3]          # base_link shares base_footprint's x
    srv = (ROOT / "src" / "server_x3.py").read_text()
    m = re.search(r"LIDAR_X_M = float\(os\.environ\.get\('X3_LIDAR_X_M', ([-0-9.]+)\)\)", srv)
    assert m and abs(float(m.group(1)) - laser_x_base_link) < 0.002
    js = (ROOT / "src" / "web" / "scene-view.js").read_text()
    m = re.search(r"const LASER_X = ([-0-9.]+);", js)
    assert m and abs(float(m.group(1)) - laser_x_base_link) < 0.002


def test_arducam_matches_the_calibration_json(urdf):
    """arducam_optical_frame in livox_frame == the 2026-11-20 solve (within 1 mm / 0.05 deg)."""
    import json
    from scipy.spatial.transform import Rotation
    poses, _ = _fk(urdf)
    T = np.linalg.inv(poses["livox_frame"]) @ poses["arducam_optical_frame"]
    v = json.loads((ROOT / "config" / "camera_lidar_extrinsics_2026-11-20.json").read_text())["right"]["T_lidar_camera"]
    assert np.allclose(T[:3, 3], v[:3], atol=1e-3)
    d = Rotation.from_matrix(T[:3, :3]).inv() * Rotation.from_quat(v[3:])
    assert math.degrees(d.magnitude()) < 0.05
    # faces the robot's right: optical z mostly along -y of base
    assert (poses["arducam_optical_frame"][:3, 2] @ np.array([0, -1, 0])) > 0.95


def test_left_cam_matches_the_calibration_json(urdf):
    """left_cam_optical_frame in livox_frame == the 2026-11-21 solve (within 1 mm / 0.05 deg)."""
    import json
    from scipy.spatial.transform import Rotation
    poses, _ = _fk(urdf)
    T = np.linalg.inv(poses["livox_frame"]) @ poses["left_cam_optical_frame"]
    v = json.loads((ROOT / "config" / "camera_lidar_extrinsics_2026-11-20.json").read_text())["left"]["T_lidar_camera"]
    assert np.allclose(T[:3, 3], v[:3], atol=1e-3)
    d = Rotation.from_matrix(T[:3, :3]).inv() * Rotation.from_quat(v[3:])
    assert math.degrees(d.magnitude()) < 0.05
    # faces the robot's left: optical z mostly along +y of base
    assert (poses["left_cam_optical_frame"][:3, 2] @ np.array([0, 1, 0])) > 0.95
