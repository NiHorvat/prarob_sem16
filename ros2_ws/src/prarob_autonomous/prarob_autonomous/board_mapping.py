"""Image-pixel -> robot-world coordinate mapping for the drawing board.

Two backends are supported:

* **Homography** (preferred): a 3x3 matrix ``H`` mapping image pixels
  ``[u, v, 1]`` to board/world coordinates ``[x, y, 1]`` on the ``z = 0`` plane.
  It is produced by :mod:`board_calibration_node` from the checkerboard and
  stored in a small YAML file, then loaded here.

* **Linear fallback**: the original behaviour where the camera image is assumed
  to span a rectangular workspace.  Used when no homography file is available.

The homography solve (:func:`compute_homography`) uses a plain NumPy DLT so the
whole module is importable and testable without OpenCV or ROS.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np


class BoardMapping:
    """Maps image pixels to robot-world XY (metres) on the board plane."""

    def __init__(
        self,
        homography: Sequence[Sequence[float]] | None = None,
        image_width: int = 640,
        image_height: int = 480,
        workspace_x_min: float = 0.06,
        workspace_x_max: float = 0.28,
        workspace_y_min: float = -0.14,
        workspace_y_max: float = 0.14,
    ):
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.workspace_x_min = float(workspace_x_min)
        self.workspace_x_max = float(workspace_x_max)
        self.workspace_y_min = float(workspace_y_min)
        self.workspace_y_max = float(workspace_y_max)

        self.H = None
        if homography is not None:
            H = np.asarray(homography, dtype=float).reshape(3, 3)
            if abs(np.linalg.det(H)) < 1e-12:
                raise ValueError("Homography matrix is singular.")
            self.H = H

    @property
    def uses_homography(self) -> bool:
        return self.H is not None

    def image_to_world(self, u: float, v: float) -> tuple[float, float]:
        """Map a single image pixel ``(u, v)`` to world ``(x, y)`` in metres."""
        if self.H is not None:
            p = self.H @ np.array([float(u), float(v), 1.0])
            if abs(p[2]) < 1e-12:
                raise ValueError("Degenerate homography projection (w == 0).")
            return float(p[0] / p[2]), float(p[1] / p[2])
        return self._linear_image_to_world(u, v)

    def _linear_image_to_world(self, u: float, v: float) -> tuple[float, float]:
        u = min(max(float(u), 0.0), float(self.image_width - 1))
        v = min(max(float(v), 0.0), float(self.image_height - 1))
        x_norm = 1.0 - (v / float(self.image_height - 1))
        y_norm = 1.0 - (u / float(self.image_width - 1))
        x = self.workspace_x_min + x_norm * (self.workspace_x_max - self.workspace_x_min)
        y = self.workspace_y_min + y_norm * (self.workspace_y_max - self.workspace_y_min)
        return float(x), float(y)

    def path_to_world(self, pixels: Sequence[Sequence[float]], z: float = 0.0):
        """Map a pixel polyline to a list of ``(x, y, z)`` world waypoints."""
        out = []
        for px in pixels:
            x, y = self.image_to_world(px[0], px[1])
            out.append((x, y, float(z)))
        return out

    # ----- persistence -------------------------------------------------------

    @classmethod
    def from_yaml(cls, path: str, **defaults) -> "BoardMapping":
        """Load a mapping from YAML; fall back to a linear mapping on any error."""
        try:
            import yaml  # local import keeps the module ROS-free for tests

            with open(path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh) or {}
            homography = data.get("homography")
            params = {
                "image_width": data.get("image_width", defaults.get("image_width", 640)),
                "image_height": data.get("image_height", defaults.get("image_height", 480)),
                "workspace_x_min": data.get("workspace_x_min", defaults.get("workspace_x_min", 0.06)),
                "workspace_x_max": data.get("workspace_x_max", defaults.get("workspace_x_max", 0.28)),
                "workspace_y_min": data.get("workspace_y_min", defaults.get("workspace_y_min", -0.14)),
                "workspace_y_max": data.get("workspace_y_max", defaults.get("workspace_y_max", 0.14)),
            }
            return cls(homography=homography, **params)
        except Exception:
            return cls(homography=None, **defaults)

    def to_yaml(self, path: str) -> None:
        import yaml

        data = {
            "image_width": self.image_width,
            "image_height": self.image_height,
            "workspace_x_min": self.workspace_x_min,
            "workspace_x_max": self.workspace_x_max,
            "workspace_y_min": self.workspace_y_min,
            "workspace_y_max": self.workspace_y_max,
        }
        if self.H is not None:
            data["homography"] = [[float(v) for v in row] for row in self.H]
        with open(path, "w", encoding="utf-8") as fh:
            yaml.safe_dump(data, fh, sort_keys=False)


def compute_homography(
    image_points: Sequence[Sequence[float]],
    world_points: Sequence[Sequence[float]],
) -> list[list[float]]:
    """Compute the 3x3 homography mapping image pixels to world XY via DLT.

    Args:
        image_points: >= 4 pixel correspondences ``[[u, v], ...]``.
        world_points: matching world points ``[[x, y], ...]`` (metres).

    Returns:
        The homography as a nested list, normalised so ``H[2][2] == 1``.
    """
    img = np.asarray(image_points, dtype=float)
    wld = np.asarray(world_points, dtype=float)
    if img.shape[0] < 4 or img.shape != wld.shape or img.shape[1] != 2:
        raise ValueError("Need >= 4 matching [u, v] / [x, y] correspondences.")

    # Normalise both point sets for numerical stability (Hartley normalisation).
    def _normalise(points):
        centroid = points.mean(axis=0)
        shifted = points - centroid
        mean_dist = np.mean(np.sqrt((shifted ** 2).sum(axis=1)))
        scale = np.sqrt(2.0) / mean_dist if mean_dist > 1e-12 else 1.0
        T = np.array([
            [scale, 0.0, -scale * centroid[0]],
            [0.0, scale, -scale * centroid[1]],
            [0.0, 0.0, 1.0],
        ])
        homog = np.hstack([points, np.ones((points.shape[0], 1))])
        return (T @ homog.T).T, T

    img_n, T_img = _normalise(img)
    wld_n, T_wld = _normalise(wld)

    A = []
    for (u, v, _), (x, y, _) in zip(img_n, wld_n):
        A.append([-u, -v, -1, 0, 0, 0, u * x, v * x, x])
        A.append([0, 0, 0, -u, -v, -1, u * y, v * y, y])
    A = np.asarray(A, dtype=float)

    _, _, vh = np.linalg.svd(A)
    H_n = vh[-1].reshape(3, 3)

    # Denormalise: H = T_wld^-1 * H_n * T_img
    H = np.linalg.inv(T_wld) @ H_n @ T_img
    if abs(H[2, 2]) > 1e-12:
        H = H / H[2, 2]
    return [[float(v) for v in row] for row in H]


if __name__ == "__main__":  # pragma: no cover - manual smoke test
    # Map the four image corners to a 0.30 x 0.30 m board and round-trip a point.
    img_pts = [[0, 0], [639, 0], [639, 479], [0, 479]]
    wld_pts = [[0.10, 0.15], [0.10, -0.15], [0.28, -0.15], [0.28, 0.15]]
    H = compute_homography(img_pts, wld_pts)
    bm = BoardMapping(homography=H)
    print("center ->", bm.image_to_world(320, 240))
