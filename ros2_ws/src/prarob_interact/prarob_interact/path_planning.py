"""Image-space path planning helpers for the drawing task.

The planner works on a 2D occupancy grid built from YOLO bounding boxes.  The
result can be converted to task-space waypoints and sent to the existing IK
node, which accepts marker-tip targets on ``/ik_node/xyz``.
"""

from __future__ import annotations

from heapq import heappop, heappush
from math import hypot
from typing import Any


Point2D = tuple[float, float]
GridCell = tuple[int, int]  # row, col


def _box_value(box: Any, key: str) -> float:
    if isinstance(box, dict):
        return float(box[key])
    return float(getattr(box, key))


def point_from_detection(detection: Any) -> Point2D:
    """Return the center point of a detection/bounding-box-like object."""
    return (
        (_box_value(detection, "start_x") + _box_value(detection, "end_x")) / 2.0,
        (_box_value(detection, "start_y") + _box_value(detection, "end_y")) / 2.0,
    )


def coerce_image_point(point: Any) -> Point2D:
    """Accept ``[x, y]``, ``{"x": ..., "y": ...}``, or a detection bbox."""
    if isinstance(point, dict):
        if "x" in point and "y" in point:
            return float(point["x"]), float(point["y"])
        if {"start_x", "start_y", "end_x", "end_y"}.issubset(point):
            return point_from_detection(point)
    if isinstance(point, (list, tuple)) and len(point) >= 2:
        return float(point[0]), float(point[1])
    if all(hasattr(point, name) for name in ("start_x", "start_y", "end_x", "end_y")):
        return point_from_detection(point)
    if hasattr(point, "x") and hasattr(point, "y"):
        return float(point.x), float(point.y)
    raise ValueError(f"Unsupported image point format: {point!r}")


def build_obstacle_grid(
    detections: list[Any] | None,
    width: int = 640,
    height: int = 480,
    cell_size: int = 5,
    margin_cells: int = 2,
) -> list[list[int]]:
    """Build an occupancy grid from detection bounding boxes.

    Grid value ``1`` means blocked.  Bounding boxes are expanded by
    ``margin_cells`` to give the marker tip clearance around avoided objects.
    """
    if cell_size <= 0:
        raise ValueError("cell_size must be positive")

    rows = (height + cell_size - 1) // cell_size
    cols = (width + cell_size - 1) // cell_size
    grid = [[0 for _ in range(cols)] for _ in range(rows)]

    for detection in detections or []:
        start_x = _box_value(detection, "start_x")
        start_y = _box_value(detection, "start_y")
        end_x = _box_value(detection, "end_x")
        end_y = _box_value(detection, "end_y")

        c0 = max(0, int(start_x // cell_size) - margin_cells)
        r0 = max(0, int(start_y // cell_size) - margin_cells)
        c1 = min(cols - 1, int(end_x // cell_size) + margin_cells)
        r1 = min(rows - 1, int(end_y // cell_size) + margin_cells)

        for row in range(r0, r1 + 1):
            for col in range(c0, c1 + 1):
                grid[row][col] = 1

    return grid


def pixel_to_cell(point: Any, width: int, height: int, cell_size: int) -> GridCell:
    x, y = coerce_image_point(point)
    col = min(max(int(x // cell_size), 0), (width + cell_size - 1) // cell_size - 1)
    row = min(max(int(y // cell_size), 0), (height + cell_size - 1) // cell_size - 1)
    return row, col


def cell_to_pixel(cell: GridCell, width: int, height: int, cell_size: int) -> Point2D:
    row, col = cell
    x = min(width - 1, col * cell_size + cell_size / 2.0)
    y = min(height - 1, row * cell_size + cell_size / 2.0)
    return x, y


def _neighbors(cell: GridCell, rows: int, cols: int, allow_diagonal: bool):
    row, col = cell
    steps = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if allow_diagonal:
        steps.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])
    for dr, dc in steps:
        nr, nc = row + dr, col + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            yield nr, nc


def astar_grid(
    grid: list[list[int]],
    start: GridCell,
    goal: GridCell,
    allow_diagonal: bool = True,
) -> list[GridCell]:
    """Plan a grid path using A* and return cells from start to goal."""
    if not grid or not grid[0]:
        raise ValueError("obstacle grid must not be empty")

    rows = len(grid)
    cols = len(grid[0])
    for row in grid:
        if len(row) != cols:
            raise ValueError("obstacle grid rows must have equal length")

    if grid[start[0]][start[1]]:
        raise ValueError("start cell is occupied")
    if grid[goal[0]][goal[1]]:
        raise ValueError("goal cell is occupied")

    frontier: list[tuple[float, GridCell]] = []
    heappush(frontier, (0.0, start))
    came_from: dict[GridCell, GridCell | None] = {start: None}
    cost_so_far: dict[GridCell, float] = {start: 0.0}

    while frontier:
        _, current = heappop(frontier)
        if current == goal:
            break

        for nxt in _neighbors(current, rows, cols, allow_diagonal):
            if grid[nxt[0]][nxt[1]]:
                continue

            step_cost = hypot(nxt[0] - current[0], nxt[1] - current[1])
            new_cost = cost_so_far[current] + step_cost
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    if goal not in came_from:
        return []

    path = []
    current: GridCell | None = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def _line_cells(a: GridCell, b: GridCell):
    r0, c0 = a
    r1, c1 = b
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc

    row, col = r0, c0
    while True:
        yield row, col
        if row == r1 and col == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            row += sr
        if e2 < dr:
            err += dr
            col += sc


def has_line_of_sight(grid: list[list[int]], a: GridCell, b: GridCell) -> bool:
    return all(grid[row][col] == 0 for row, col in _line_cells(a, b))


def smooth_grid_path(grid: list[list[int]], path: list[GridCell]) -> list[GridCell]:
    """Remove unnecessary intermediate cells while keeping obstacle clearance."""
    if len(path) <= 2:
        return path

    smoothed = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1:
            if has_line_of_sight(grid, path[anchor], path[candidate]):
                break
            candidate -= 1
        smoothed.append(path[candidate])
        anchor = candidate
    return smoothed


def plan_image_path(
    start: Any,
    goal: Any,
    obstacles: list[Any] | None = None,
    obstacle_grid: list[list[int]] | None = None,
    width: int = 640,
    height: int = 480,
    cell_size: int = 5,
    margin_cells: int = 2,
    simplify: bool = True,
) -> dict[str, Any]:
    """Plan an obstacle-avoiding path in image coordinates."""
    grid = obstacle_grid or build_obstacle_grid(
        obstacles, width=width, height=height, cell_size=cell_size, margin_cells=margin_cells
    )
    start_cell = pixel_to_cell(start, width, height, cell_size)
    goal_cell = pixel_to_cell(goal, width, height, cell_size)
    cells = astar_grid(grid, start_cell, goal_cell, allow_diagonal=True)
    if simplify:
        cells = smooth_grid_path(grid, cells)

    return {
        "path_found": bool(cells),
        "path_cells": cells,
        "path_pixels": [cell_to_pixel(cell, width, height, cell_size) for cell in cells],
        "start_cell": start_cell,
        "goal_cell": goal_cell,
        "grid_size": {"rows": len(grid), "cols": len(grid[0]) if grid else 0},
        "cell_size": cell_size,
    }


def image_path_to_task_path(
    path_pixels: list[Any],
    image_width: int = 640,
    image_height: int = 480,
    workspace_x_min: float = 0.06,
    workspace_x_max: float = 0.28,
    workspace_y_min: float = -0.14,
    workspace_y_max: float = 0.14,
    z: float = 0.0,
) -> list[tuple[float, float, float]]:
    """Map image pixels to robot task-space points.

    The default linear mapping assumes the camera image spans the drawable
    board.  Calibration-specific bounds should be passed in when measured.
    """
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")

    points = []
    for point in path_pixels:
        px, py = coerce_image_point(point)
        px = min(max(px, 0.0), float(image_width - 1))
        py = min(max(py, 0.0), float(image_height - 1))

        x_norm = 1.0 - (py / float(image_height - 1))
        y_norm = 1.0 - (px / float(image_width - 1))
        x = workspace_x_min + x_norm * (workspace_x_max - workspace_x_min)
        y = workspace_y_min + y_norm * (workspace_y_max - workspace_y_min)
        points.append((x, y, z))
    return points
