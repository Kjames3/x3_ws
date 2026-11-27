"""Opt-in omni-direct Nav2 params (strafe experiment, 2026-11-26).

nav2_params_x3_omni_direct.yaml must equal nav2_params_x3.yaml everywhere
except controller_server.FollowPath, where the RotationShimController is
replaced by plain MPPI. Also covers X3_NAV2_PARAMS resolution in nav2_client.
"""
import ast
import copy
import pathlib

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
PARAMS = ROOT / "src" / "yahboomcar_nav" / "params"
BASE = PARAMS / "nav2_params_x3.yaml"
OMNI = PARAMS / "nav2_params_x3_omni_direct.yaml"
NAV2_CLIENT = ROOT / "src" / "nav2_client.py"

SHIM_KEYS = {
    "primary_controller", "angular_dist_threshold", "forward_sampling_distance",
    "rotate_to_heading_angular_vel", "max_angular_accel", "simulate_ahead_time",
}


def _load(p):
    return yaml.safe_load(p.read_text())


def _follow_path(d):
    return d["controller_server"]["ros__parameters"]["FollowPath"]


def test_omni_yaml_parses():
    assert isinstance(_load(OMNI), dict)


def test_only_follow_path_differs():
    base, omni = _load(BASE), _load(OMNI)
    b, o = copy.deepcopy(base), copy.deepcopy(omni)
    _follow_path(b).clear()
    _follow_path(o).clear()
    assert b == o, "omni_direct drifted from nav2_params_x3.yaml outside FollowPath"


def test_follow_path_is_plain_mppi_minus_the_shim():
    bf, of = _follow_path(_load(BASE)), _follow_path(_load(OMNI))
    assert bf["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert of["plugin"] == "nav2_mppi_controller::MPPIController"
    assert set(bf) - set(of) == SHIM_KEYS
    assert set(of) - set(bf) == set()
    changed = {k for k in of if of[k] != bf[k]}
    assert changed == {"plugin"}


def test_mppi_omni_keys_present():
    of = _follow_path(_load(OMNI))
    assert of["motion_model"] == "Omni"
    for k in ("vy_max", "vy_std", "vx_max", "wz_max", "critics", "time_steps", "batch_size"):
        assert k in of, k
    assert of["vy_max"] > 0 and of["vy_std"] > 0


def test_setup_py_installs_params_glob():
    setup = (ROOT / "src" / "yahboomcar_nav" / "setup.py").read_text()
    assert "glob(os.path.join('params', '*.*'))" in setup


def _resolver():
    # nav2_client imports rclpy at module level; exec only the pure pieces.
    tree = ast.parse(NAV2_CLIENT.read_text())
    keep = [n for n in tree.body
            if (isinstance(n, ast.Assign) and any(getattr(t, "id", "") in ("NAV2_PARAMS_ENV", "_NAV2_PARAMS_ALIASES")
                                                  for t in n.targets))
            or (isinstance(n, ast.FunctionDef) and n.name == "resolve_nav2_params_file")]
    ns = {"os": __import__("os"), "Optional": __import__("typing").Optional}
    exec(compile(ast.Module(body=keep, type_ignores=[]), str(NAV2_CLIENT), "exec"), ns)
    return ns["resolve_nav2_params_file"]


@pytest.mark.parametrize("value", [None, "", "   "])
def test_resolve_unset_is_default(value):
    path, msg = _resolver()(value, str(PARAMS))
    assert path is None and msg.startswith("default")


def test_resolve_alias():
    path, msg = _resolver()("omni_direct", str(PARAMS))
    assert path == str(OMNI)
    assert not msg.startswith("ERROR")


def test_resolve_absolute_path(tmp_path):
    f = tmp_path / "custom.yaml"
    f.write_text("{}\n")
    path, _ = _resolver()(str(f), str(PARAMS))
    assert path == str(f)


def test_resolve_missing_falls_back_with_error(tmp_path):
    r = _resolver()
    path, msg = r(str(tmp_path / "nope.yaml"), str(PARAMS))
    assert path is None and msg.startswith("ERROR")
    # alias whose installed file is missing (stale install tree)
    path, msg = r("omni_direct", str(tmp_path))
    assert path is None and msg.startswith("ERROR")


def test_resolve_relative_rejected():
    path, msg = _resolver()("nav2_params_x3_omni_direct.yaml", str(PARAMS))
    assert path is None and msg.startswith("ERROR")
