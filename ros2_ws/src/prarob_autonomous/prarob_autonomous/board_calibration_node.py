#!/usr/bin/env python3
"""Persist the image -> board (robot-world) homography from the checkerboard.

The seminar workspace is a flat board, so the mapping from camera pixels to
robot-world XY on the ``z = 0`` plane is a single 3x3 homography.  This node
detects the calibration checkerboard, pairs each detected corner with its known
robot-frame coordinate (same convention as ``prarob_calib/camera_to_world.py``),
fits the homography with a DLT and writes it to a YAML file that
``autonomous_draw_node`` loads via the ``board_homography_file`` parameter.

Run it once with the board and checkerboard in view; it saves on the first good
detection (``save_once``) and reports the mean reprojection error in millimetres.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from prarob_autonomous.board_mapping import BoardMapping, compute_homography


class BoardCalibrationNode(Node):
    def __init__(self):
        super().__init__("board_calibration_node")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("output_file", "board_homography.yaml")
        self.declare_parameter("checkerboard_cols", 9)
        self.declare_parameter("checkerboard_rows", 7)
        self.declare_parameter("square_size", 0.0186)
        # Pose of the checkerboard origin in the robot frame {R} (metres).
        self.declare_parameter("board_origin_x", 0.0423)
        self.declare_parameter("board_origin_y", 0.1016)
        # Axis signs of the checkerboard frame expressed in {R} (x, y).
        self.declare_parameter("board_axis_x", 1.0)
        self.declare_parameter("board_axis_y", -1.0)
        self.declare_parameter("save_once", True)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cols = int(gp("checkerboard_cols"))
        self.rows = int(gp("checkerboard_rows"))
        self.output_file = str(gp("output_file"))
        self.save_once = bool(gp("save_once"))
        self._saved = False

        self.world_xy = self._build_world_points()

        self._bridge = None
        self.status_pub = self.create_publisher(String, str(gp("status_topic")), 10)
        self.create_subscription(Image, str(gp("image_topic")),
                                 self._on_image, 1)
        self.get_logger().info(
            f"board_calibration_node ready ({self.cols}x{self.rows} checkerboard)")

    def _build_world_points(self) -> np.ndarray:
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        square = float(gp("square_size"))
        objp = np.zeros((self.cols * self.rows, 2), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        objp *= square
        # Transform checkerboard XY into the robot frame {R}.
        ax = float(gp("board_axis_x"))
        ay = float(gp("board_axis_y"))
        ox = float(gp("board_origin_x"))
        oy = float(gp("board_origin_y"))
        world = np.column_stack([ax * objp[:, 0] + ox, ay * objp[:, 1] + oy])
        return world

    def _publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)

    def _on_image(self, msg: Image):
        if self._saved and self.save_once:
            return
        try:
            import cv2
            if self._bridge is None:
                from cv_bridge import CvBridge
                self._bridge = CvBridge()
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"image conversion failed: {exc}", once=True)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows), None)
        if not found:
            self.get_logger().info("checkerboard not found in frame", throttle_duration_sec=2.0)
            return

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        image_pts = corners.reshape(-1, 2)

        try:
            H = compute_homography(image_pts.tolist(), self.world_xy.tolist())
        except Exception as exc:  # noqa: BLE001
            self._publish_status(f"homography fit failed: {exc}")
            return

        # Reprojection error in millimetres.
        bm = BoardMapping(homography=H)
        errs = []
        for (u, v), (x, y) in zip(image_pts, self.world_xy):
            mx, my = bm.image_to_world(u, v)
            errs.append(np.hypot(mx - x, my - y))
        mean_mm = float(np.mean(errs) * 1000.0)
        max_mm = float(np.max(errs) * 1000.0)

        bm.image_width = frame.shape[1]
        bm.image_height = frame.shape[0]
        bm.to_yaml(self.output_file)
        self._saved = True
        self._publish_status(
            f"Saved board homography to {self.output_file} "
            f"(reproj mean={mean_mm:.2f} mm, max={max_mm:.2f} mm).")


def main(args=None):
    rclpy.init(args=args)
    node = BoardCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
