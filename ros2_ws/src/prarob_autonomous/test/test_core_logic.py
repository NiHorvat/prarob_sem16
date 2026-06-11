"""Pure-Python tests for the autonomous core (parser, mapping, planning, IK).

These run without ROS so they can be executed on any machine:

    python3 -m pytest src/prarob_autonomous/test/test_core_logic.py
    # or simply:
    python3 src/prarob_autonomous/test/test_core_logic.py
"""

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
# prarob_autonomous package
sys.path.insert(0, os.path.join(_HERE, ".."))
# sibling prarob_interact package (path_planning, kinematics)
sys.path.insert(0, os.path.join(_HERE, "..", "..", "prarob_interact"))

from prarob_autonomous.command_parser import parse_command
from prarob_autonomous.board_mapping import BoardMapping, compute_homography
from prarob_interact.path_planning import (
    build_obstacle_grid,
    plan_image_path,
)
from prarob_interact.kinematics import Kinematics


# --------------------------------------------------------------------------- #
# command parser
# --------------------------------------------------------------------------- #

def test_parser_basic_english():
    r = parse_command("connect the car and the plane, avoid football")
    assert r["ok"], r
    assert r["connect"] == ["car", "airplane"], r["connect"]
    assert r["avoid"] == ["sports ball"], r["avoid"]


def test_parser_three_connect_two_avoid():
    r = parse_command("connect car, plane and bottle while avoiding football and cat")
    assert r["ok"], r
    assert r["connect"] == ["car", "airplane", "bottle"], r["connect"]
    assert r["avoid"] == ["sports ball", "cat"], r["avoid"]


def test_parser_croatian():
    r = parse_command("spoji auto i avion, izbjegni nogometnu loptu")
    assert r["ok"], r
    assert r["connect"] == ["car", "airplane"], r["connect"]
    assert r["avoid"] == ["sports ball"], r["avoid"]


def test_parser_no_avoid():
    r = parse_command("connect stop sign and the clock")
    assert r["ok"], r
    assert r["connect"] == ["stop sign", "clock"], r["connect"]
    assert r["avoid"] == []


def test_parser_unknown_classes_passthrough():
    r = parse_command("connect widget and gizmo, avoid blob")
    assert r["ok"], r
    assert r["connect"] == ["widget", "gizmo"], r["connect"]
    assert r["avoid"] == ["blob"], r["avoid"]


def test_parser_rejects_single_object():
    r = parse_command("connect car")
    assert not r["ok"]
    assert "two" in r["error"].lower()


def test_parser_example_from_seminar():
    r = parse_command("connect stop sign and the car, avoid cats and traffic lights")
    assert r["ok"], r
    assert r["connect"] == ["stop sign", "car"], r["connect"]
    assert r["avoid"] == ["cat", "traffic light"], r["avoid"]


# --------------------------------------------------------------------------- #
# board mapping
# --------------------------------------------------------------------------- #

def test_homography_round_trip():
    img_pts = [[0, 0], [639, 0], [639, 479], [0, 479]]
    wld_pts = [[0.10, 0.15], [0.10, -0.15], [0.28, -0.15], [0.28, 0.15]]
    H = compute_homography(img_pts, wld_pts)
    bm = BoardMapping(homography=H)
    for (u, v), (x, y) in zip(img_pts, wld_pts):
        mx, my = bm.image_to_world(u, v)
        assert abs(mx - x) < 1e-6, (u, v, mx, x)
        assert abs(my - y) < 1e-6, (u, v, my, y)
    # Centre pixel maps to the board centre.
    cx, cy = bm.image_to_world(319.5, 239.5)
    assert abs(cx - 0.19) < 1e-3, cx
    assert abs(cy - 0.0) < 1e-3, cy


def test_linear_fallback_corners():
    bm = BoardMapping(homography=None)
    # top-right pixel -> (x_max via row top, y_max via col... ) just verify bounds
    x, y = bm.image_to_world(0, 0)
    assert bm.workspace_x_min <= x <= bm.workspace_x_max
    assert bm.workspace_y_min <= y <= bm.workspace_y_max


# --------------------------------------------------------------------------- #
# planning -> world -> IK end to end
# --------------------------------------------------------------------------- #

def _yolo_box(cx, cy, w=40, h=40, name="obj"):
    return {
        "class_name": name,
        "start_x": cx - w / 2, "start_y": cy - h / 2,
        "end_x": cx + w / 2, "end_y": cy + h / 2,
    }


def test_plan_avoids_obstacle_and_is_reachable():
    # Two objects on a horizontal line, an obstacle between them.
    start = _yolo_box(120, 240, name="car")
    goal = _yolo_box(520, 240, name="airplane")
    obstacle = _yolo_box(320, 240, w=80, h=80, name="sports ball")

    planned = plan_image_path(
        start=start, goal=goal, obstacles=[obstacle],
        width=640, height=480, cell_size=5, margin_cells=3, simplify=True,
    )
    assert planned["path_found"], planned

    # The path must not pass through the inflated obstacle.
    grid = build_obstacle_grid([obstacle], 640, 480, 5, 3)
    for (px, py) in planned["path_pixels"]:
        col = int(px // 5)
        row = int(py // 5)
        assert grid[row][col] == 0, f"path crosses obstacle at {(px, py)}"

    # Map to a board band that sits inside the robot's reachable annulus
    # (the +-90 deg joint limits only allow a far ring of the workspace).
    img_pts = [[0, 0], [639, 0], [639, 479], [0, 479]]
    wld_pts = [[0.28, 0.05], [0.28, -0.05], [0.32, -0.05], [0.32, 0.05]]
    bm = BoardMapping(homography=compute_homography(img_pts, wld_pts))
    world = bm.path_to_world(planned["path_pixels"], z=0.0)
    assert world, "mapping produced no world waypoints"

    k = Kinematics()
    reachable = sum(1 for (x, y, z) in world if k.get_ik([x, y, z]))
    assert reachable == len(world), (
        f"only {reachable}/{len(world)} waypoints reachable within joint limits"
    )


def test_plan_drawing_end_to_end():
    from prarob_autonomous.planning_pipeline import plan_drawing

    def box(cx, cy, name, w=40, h=40, score=0.9):
        return {
            "class_name": name, "score": score, "cx": cx, "cy": cy,
            "start_x": cx - w / 2, "start_y": cy - h / 2,
            "end_x": cx + w / 2, "end_y": cy + h / 2,
        }

    connect = [box(120, 240, "car"), box(520, 240, "airplane")]
    obstacles = [box(320, 240, "sports ball", w=80, h=80)]

    img_pts = [[0, 0], [639, 0], [639, 479], [0, 479]]
    wld_pts = [[0.28, 0.05], [0.28, -0.05], [0.32, -0.05], [0.32, 0.05]]
    bm = BoardMapping(homography=compute_homography(img_pts, wld_pts))

    plan = plan_drawing(connect, obstacles, bm, drawing_z=0.0, pen_up_z=0.03)
    assert plan["ok"], plan["error"]
    assert plan["unreachable"] == 0
    assert plan["segments"] == 1
    # First and last waypoints are pen-up, interior are pen-down with IK.
    assert plan["waypoints"][0]["pen_down"] is False
    assert plan["waypoints"][-1]["pen_down"] is False
    assert any(wp["pen_down"] for wp in plan["waypoints"])
    assert all(wp["q"] is not None for wp in plan["waypoints"])


def test_plan_drawing_reports_missing_path():
    from prarob_autonomous.planning_pipeline import plan_drawing

    def box(cx, cy, name, w=40, h=40):
        return {"class_name": name, "score": 0.9, "cx": cx, "cy": cy,
                "start_x": cx - w / 2, "start_y": cy - h / 2,
                "end_x": cx + w / 2, "end_y": cy + h / 2}

    # A wall of obstacle fully separating start and goal -> no path.
    connect = [box(60, 240, "car"), box(580, 240, "airplane")]
    wall = [box(320, y, "sports ball", w=120, h=40) for y in range(0, 480, 30)]
    bm = BoardMapping(homography=None)
    plan = plan_drawing(connect, wall, bm, margin_cells=4)
    assert not plan["ok"]
    assert "path" in plan["error"].lower()


def test_kinematics_round_trip():
    k = Kinematics()
    q0 = [0.4, 0.2, -0.6]
    target = k.get_dk(q0)
    sols = k.get_ik(target)
    assert sols, "IK returned no solution for a reachable pose"
    for q in sols:
        rec = k.get_dk(q)
        assert max(abs(rec[i] - target[i]) for i in range(3)) < 1e-9


def _run_all():
    funcs = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    for fn in funcs:
        try:
            fn()
        except AssertionError as exc:
            print(f"FAIL {fn.__name__}: {exc}")
        except Exception as exc:  # noqa: BLE001
            print(f"ERROR {fn.__name__}: {exc!r}")
        else:
            print(f"ok   {fn.__name__}")
            passed += 1
    print(f"\n{passed}/{len(funcs)} tests passed")
    return passed == len(funcs)


if __name__ == "__main__":
    sys.exit(0 if _run_all() else 1)
