"""Helpers to turn a YOLO ``DetectionArray`` into planner-friendly boxes.

Kept ROS-free (operates on already-extracted plain dicts) so it can be reused
and unit tested without rclpy.  The node converts the ROS message into the list
of dicts this module expects.
"""

from __future__ import annotations

from typing import Any


def detection_to_box(detection: Any) -> dict:
    """Convert one ``yolo_msgs/Detection`` to a planner box dict (pixels)."""
    bbox = detection.bbox
    cx = float(bbox.center.position.x)
    cy = float(bbox.center.position.y)
    w = float(bbox.size.x)
    h = float(bbox.size.y)
    return {
        "class_name": str(detection.class_name),
        "score": float(detection.score),
        "cx": cx,
        "cy": cy,
        "start_x": cx - w / 2.0,
        "start_y": cy - h / 2.0,
        "end_x": cx + w / 2.0,
        "end_y": cy + h / 2.0,
    }


def detections_to_boxes(detections: list[Any]) -> list[dict]:
    return [detection_to_box(d) for d in detections]


def best_box_for_class(boxes: list[dict], class_name: str,
                       min_score: float = 0.0) -> dict | None:
    """Return the highest-scoring box whose class matches ``class_name``."""
    candidates = [
        b for b in boxes
        if b["class_name"].lower() == class_name.lower() and b["score"] >= min_score
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda b: b["score"])


def boxes_for_classes(boxes: list[dict], class_names: list[str],
                      min_score: float = 0.0) -> list[dict]:
    """Return every box matching any of ``class_names`` (used for obstacles)."""
    wanted = {c.lower() for c in class_names}
    return [
        b for b in boxes
        if b["class_name"].lower() in wanted and b["score"] >= min_score
    ]


def resolve_targets(boxes: list[dict], connect: list[str],
                    min_score: float = 0.0) -> tuple[list[dict], list[str]]:
    """Resolve each connect class to its best box.

    Returns ``(resolved_boxes, missing_class_names)`` preserving order.
    """
    resolved: list[dict] = []
    missing: list[str] = []
    for name in connect:
        box = best_box_for_class(boxes, name, min_score)
        if box is None:
            missing.append(name)
        else:
            resolved.append(box)
    return resolved, missing
