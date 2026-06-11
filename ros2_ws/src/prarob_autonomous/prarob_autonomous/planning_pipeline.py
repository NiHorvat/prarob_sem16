"""ROS-free planning pipeline: detections + command -> executable waypoints.

This is the deterministic core of the autonomous mode.  It is deliberately
free of rclpy so it can be unit tested on any machine.  The orchestrator node
only does ROS I/O (reading detections, publishing angles/overlay) and delegates
all the geometry and planning to :func:`plan_drawing`.

A waypoint is ``{"x", "y", "z", "q": [q1, q2, q3] | None, "pen_down": bool}``.
"""

from __future__ import annotations

from typing import Any

# These come from the sibling prarob_interact package at runtime.
from prarob_interact.path_planning import build_obstacle_grid, plan_image_path
from prarob_interact.kinematics import Kinematics


def order_targets_nearest_neighbour(boxes: list[dict]) -> list[dict]:
    """Greedy nearest-neighbour ordering in pixel space, keeping the first box."""
    if len(boxes) <= 2:
        return list(boxes)
    remaining = list(boxes)
    ordered = [remaining.pop(0)]
    while remaining:
        last = ordered[-1]
        nxt = min(
            remaining,
            key=lambda b: (b["cx"] - last["cx"]) ** 2 + (b["cy"] - last["cy"]) ** 2,
        )
        remaining.remove(nxt)
        ordered.append(nxt)
    return ordered


def _concat_pixel_paths(segments: list[list[tuple[float, float]]]):
    """Join per-segment pixel polylines, dropping duplicated junction points."""
    full: list[tuple[float, float]] = []
    for seg in segments:
        if not seg:
            continue
        if full and seg and abs(full[-1][0] - seg[0][0]) < 1e-6 \
                and abs(full[-1][1] - seg[0][1]) < 1e-6:
            full.extend(seg[1:])
        else:
            full.extend(seg)
    return full


def plan_drawing(
    connect_boxes: list[dict],
    obstacle_boxes: list[dict],
    board_mapping,
    kinematics: Kinematics | None = None,
    image_width: int = 640,
    image_height: int = 480,
    cell_size: int = 5,
    margin_cells: int = 3,
    drawing_z: float = 0.0,
    pen_up_z: float = 0.03,
    prev_q: list[float] | None = None,
    order: str = "given",
) -> dict:
    """Plan the full marker path for a connect/avoid command.

    Args:
        connect_boxes: resolved YOLO boxes (planner-dict form) to connect, in
            order.  Must contain at least two boxes.
        obstacle_boxes: YOLO boxes to avoid.
        board_mapping: a :class:`BoardMapping` instance (pixel -> world XY).
        kinematics: a :class:`Kinematics` instance; created if omitted.
        order: ``"given"`` keeps the command order, ``"nearest"`` reorders
            greedily to shorten travel.

    Returns:
        Dict with ``ok``, ``error``, ``pixel_path``, ``waypoints`` and
        ``unreachable`` (count of drawing waypoints with no IK solution).
    """
    result: dict[str, Any] = {
        "ok": False,
        "error": "",
        "pixel_path": [],
        "waypoints": [],
        "unreachable": 0,
        "segments": 0,
    }
    if kinematics is None:
        kinematics = Kinematics()

    if len(connect_boxes) < 2:
        result["error"] = "Need at least two resolved objects to connect."
        return result

    boxes = (order_targets_nearest_neighbour(connect_boxes)
             if order == "nearest" else list(connect_boxes))

    grid = build_obstacle_grid(
        obstacle_boxes, width=image_width, height=image_height,
        cell_size=cell_size, margin_cells=margin_cells,
    )

    segments: list[list[tuple[float, float]]] = []
    for a, b in zip(boxes[:-1], boxes[1:]):
        planned = plan_image_path(
            start={"x": a["cx"], "y": a["cy"]},
            goal={"x": b["cx"], "y": b["cy"]},
            obstacle_grid=grid,
            width=image_width, height=image_height,
            cell_size=cell_size, margin_cells=margin_cells, simplify=True,
        )
        if not planned["path_found"]:
            result["error"] = (
                f"No obstacle-free path between '{a['class_name']}' and "
                f"'{b['class_name']}'."
            )
            return result
        segments.append([tuple(p) for p in planned["path_pixels"]])

    pixel_path = _concat_pixel_paths(segments)
    result["pixel_path"] = [list(p) for p in pixel_path]
    result["segments"] = len(segments)
    if not pixel_path:
        result["error"] = "Planner produced an empty path."
        return result

    # Map drawing waypoints to the board plane.
    draw_world = board_mapping.path_to_world(pixel_path, z=drawing_z)

    # Assemble pen sequence: approach (pen up) -> draw -> retract (pen up).
    first = draw_world[0]
    last = draw_world[-1]
    sequence: list[dict] = []
    sequence.append({"x": first[0], "y": first[1], "z": pen_up_z, "pen_down": False})
    for (x, y, z) in draw_world:
        sequence.append({"x": x, "y": y, "z": z, "pen_down": True})
    sequence.append({"x": last[0], "y": last[1], "z": pen_up_z, "pen_down": False})

    # Solve IK for each waypoint, chaining from the previous configuration.
    q_ref = list(prev_q) if prev_q else [0.0, 0.0, 0.0]
    unreachable = 0
    for wp in sequence:
        sols = kinematics.get_ik([wp["x"], wp["y"], wp["z"]])
        q = kinematics.get_closest_ik(sols, q_ref) if sols else None
        wp["q"] = [float(v) for v in q] if q is not None else None
        if q is not None:
            q_ref = q
        elif wp["pen_down"]:
            unreachable += 1

    result["waypoints"] = sequence
    result["unreachable"] = unreachable
    if unreachable > 0:
        result["error"] = (
            f"{unreachable} drawing waypoint(s) have no IK solution within "
            "joint limits (object outside reachable board area)."
        )
        return result

    result["ok"] = True
    return result
