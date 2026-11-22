"""classify_nav_failure: Nav2 (Humble) log lines -> a human reason for an ABORTED goal."""
import ast
import pathlib

SRC = pathlib.Path(__file__).resolve().parents[1] / "src" / "nav2_client.py"


def _load():
    # nav2_client imports rclpy at module level; exec only the pure pieces.
    tree = ast.parse(SRC.read_text())
    keep = [n for n in tree.body
            if (isinstance(n, ast.Assign) and any(getattr(t, "id", "") in ("_FAILURE_PATTERNS", "_NAV2_LOG_NODES")
                                                  for t in n.targets))
            or (isinstance(n, ast.FunctionDef) and n.name == "classify_nav_failure")]
    ns = {}
    exec(compile(ast.Module(body=keep, type_ignores=[]), str(SRC), "exec"), ns)
    return ns["classify_nav_failure"]


def test_specific_reasons_win_over_generic_lines():
    f = _load()
    assert "progress" in f(["bt_navigator: Goal failed",
                            "controller_server: Failed to make progress"])
    assert "no path" in f(["planner_server: GridBased plugin failed to plan calculation to (1, 2): "
                           "Failed to create plan with tolerance of: 0.5"]).lower()
    assert "goal is inside an obstacle" in f(["planner_server: Goal Coordinates of(3.0, 4.0) was outside bounds"]).lower()
    assert "robot is inside an obstacle" in f(["planner_server: Start occupied"]).lower()


def test_fallbacks():
    f = _load()
    assert f([]) == "Nav2 aborted the goal without logging a reason"
    assert f(["bt_navigator: something odd"]) == "Nav2: bt_navigator: something odd"
