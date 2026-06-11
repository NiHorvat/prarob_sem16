#!/usr/bin/env python3
"""Autonomous drawing orchestrator for the PRAROB seminar robot.

Pipeline (all triggered by one natural-language command):

    command -> parse (connect/avoid) -> read latest YOLO detections
            -> resolve objects to board coordinates -> plan obstacle-free path
            -> inverse kinematics -> execute waypoints on the robot

I/O
---
Subscribes:
    ~/command            std_msgs/String        e.g. "connect car and plane, avoid football"
    <detections_topic>   yolo_msgs/DetectionArray
    /joint_states        sensor_msgs/JointState (for closest-IK seeding)
    <image_topic>        sensor_msgs/Image      (optional, only for the overlay)
Publishes:
    ~/status             std_msgs/String        JSON status + phase timings
    ~/debug_image        sensor_msgs/Image      detections + obstacles + planned path
    /move_servos_node/angles  std_msgs/Float32MultiArray  executed joint targets
    /move_servos_node/commands std_msgs/String   "reset" on home/abort

Send ``stop`` (or ``cancel``) as a command to abort an in-progress drawing.
"""

import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, String
from yolo_msgs.msg import DetectionArray

from prarob_interact.kinematics import Kinematics

from prarob_autonomous.command_parser import parse_command
from prarob_autonomous.board_mapping import BoardMapping
from prarob_autonomous.detection_utils import (
    detections_to_boxes,
    boxes_for_classes,
    resolve_targets,
)
from prarob_autonomous.planning_pipeline import plan_drawing

JOINT_NAMES = ["joint1", "joint2", "joint3"]


class AutonomousDrawNode(Node):
    def __init__(self):
        super().__init__("autonomous_draw_node")

        # ---- parameters -----------------------------------------------------
        self.declare_parameter("command_topic", "~/command")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("debug_image_topic", "~/debug_image")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("angles_topic", "/move_servos_node/angles")
        self.declare_parameter("commands_topic", "/move_servos_node/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")

        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("board_homography_file", "")
        self.declare_parameter("workspace_x_min", 0.06)
        self.declare_parameter("workspace_x_max", 0.28)
        self.declare_parameter("workspace_y_min", -0.14)
        self.declare_parameter("workspace_y_max", 0.14)

        self.declare_parameter("cell_size", 5)
        self.declare_parameter("margin_cells", 3)
        self.declare_parameter("drawing_z", 0.0)
        self.declare_parameter("pen_up_z", 0.03)
        self.declare_parameter("seconds_per_waypoint", 0.2)
        self.declare_parameter("min_detection_score", 0.25)
        self.declare_parameter("detection_timeout_sec", 3.0)
        self.declare_parameter("order", "given")  # "given" | "nearest"
        self.declare_parameter("publish_angles", True)
        self.declare_parameter("reset_before_draw", True)
        self.declare_parameter("debug_image_save_path", "")

        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        command_topic = gp("command_topic")
        status_topic = gp("status_topic")
        debug_image_topic = gp("debug_image_topic")
        detections_topic = gp("detections_topic")
        image_topic = gp("image_topic")

        # ---- state ----------------------------------------------------------
        self._lock = threading.Lock()
        self._latest_boxes: list[dict] = []
        self._latest_boxes_stamp = 0.0
        self._latest_frame = None
        self._latest_q: list[float] | None = None
        self._worker: threading.Thread | None = None
        self._abort = threading.Event()
        self._busy = False

        self.kin = Kinematics()
        self.board = self._load_board_mapping()

        # ---- interfaces -----------------------------------------------------
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 1)
        self.angles_pub = self.create_publisher(
            Float32MultiArray, gp("angles_topic"), 10)
        self.commands_pub = self.create_publisher(String, gp("commands_topic"), 10)

        self.create_subscription(String, command_topic, self._on_command, 10)
        self.create_subscription(
            DetectionArray, detections_topic, self._on_detections, 10)
        self.create_subscription(
            JointState, gp("joint_states_topic"), self._on_joint_states, 10)
        self.create_subscription(Image, image_topic, self._on_image, 1)

        self._bridge = None  # lazily created (cv_bridge import is heavy)

        self.publish_status("idle", "Autonomous node ready. Waiting for command.")
        self.get_logger().info(
            f"autonomous_draw_node ready (homography={'yes' if self.board.uses_homography else 'no'})")

    # ------------------------------------------------------------------ setup
    def _load_board_mapping(self) -> BoardMapping:
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        defaults = dict(
            image_width=int(gp("image_width")),
            image_height=int(gp("image_height")),
            workspace_x_min=float(gp("workspace_x_min")),
            workspace_x_max=float(gp("workspace_x_max")),
            workspace_y_min=float(gp("workspace_y_min")),
            workspace_y_max=float(gp("workspace_y_max")),
        )
        path = gp("board_homography_file")
        if path and os.path.isfile(path):
            self.get_logger().info(f"Loading board homography from {path}")
            return BoardMapping.from_yaml(path, **defaults)
        if path:
            self.get_logger().warn(
                f"board_homography_file '{path}' not found; using linear mapping.")
        return BoardMapping(homography=None, **defaults)

    # -------------------------------------------------------------- callbacks
    def _on_detections(self, msg: DetectionArray):
        boxes = detections_to_boxes(msg.detections)
        with self._lock:
            self._latest_boxes = boxes
            self._latest_boxes_stamp = time.time()

    def _on_joint_states(self, msg: JointState):
        q = self._extract_joints(msg)
        if q is not None:
            with self._lock:
                self._latest_q = q

    def _on_image(self, msg: Image):
        try:
            if self._bridge is None:
                from cv_bridge import CvBridge
                self._bridge = CvBridge()
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._latest_frame = frame
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"image conversion failed: {exc}", once=True)

    @staticmethod
    def _extract_joints(msg: JointState):
        if msg.name:
            by_name = dict(zip(msg.name, msg.position))
            if all(n in by_name for n in JOINT_NAMES):
                return [float(by_name[n]) for n in JOINT_NAMES]
        if len(msg.position) >= 3:
            return [float(msg.position[i]) for i in range(3)]
        return None

    def _on_command(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return
        if text.lower() in ("stop", "cancel", "abort", "stani"):
            self._abort.set()
            self.publish_status("aborting", "Stop requested by operator.")
            return
        if self._busy:
            self.publish_status("busy", "A drawing is already in progress.")
            return
        self._abort.clear()
        self._worker = threading.Thread(
            target=self._run_command, args=(text,), daemon=True)
        self._worker.start()

    # --------------------------------------------------------------- pipeline
    def _run_command(self, text: str):
        self._busy = True
        timing = {}
        t_start = time.time()
        try:
            parsed = parse_command(text)
            if not parsed["ok"]:
                self.publish_status("failed", parsed["error"], parsed=parsed)
                return

            connect = parsed["connect"]
            avoid = parsed["avoid"]
            self.publish_status(
                "detecting",
                f"Connecting {connect}; avoiding {avoid}.",
                parsed=parsed)

            # --- detection phase --------------------------------------------
            t0 = time.time()
            boxes = self._wait_for_detections()
            timing["detect"] = round(time.time() - t0, 3)
            if boxes is None:
                self.publish_status(
                    "failed", "No YOLO detections received in time.",
                    parsed=parsed, timing=timing)
                return

            min_score = float(self.get_parameter("min_detection_score").value)
            connect_boxes, missing = resolve_targets(boxes, connect, min_score)
            if missing:
                self.publish_status(
                    "failed",
                    f"Could not see required object(s): {missing}.",
                    parsed=parsed, timing=timing,
                    detected=sorted({b['class_name'] for b in boxes}))
                self._publish_overlay(boxes, connect, avoid, [])
                return
            obstacle_boxes = boxes_for_classes(boxes, avoid, min_score)

            # --- planning phase ---------------------------------------------
            self.publish_status("planning", "Planning obstacle-free path.",
                                parsed=parsed, timing=timing)
            t0 = time.time()
            with self._lock:
                prev_q = list(self._latest_q) if self._latest_q else None
            plan = plan_drawing(
                connect_boxes=connect_boxes,
                obstacle_boxes=obstacle_boxes,
                board_mapping=self.board,
                kinematics=self.kin,
                image_width=int(self.get_parameter("image_width").value),
                image_height=int(self.get_parameter("image_height").value),
                cell_size=int(self.get_parameter("cell_size").value),
                margin_cells=int(self.get_parameter("margin_cells").value),
                drawing_z=float(self.get_parameter("drawing_z").value),
                pen_up_z=float(self.get_parameter("pen_up_z").value),
                prev_q=prev_q,
                order=str(self.get_parameter("order").value),
            )
            timing["plan"] = round(time.time() - t0, 3)
            self._publish_overlay(boxes, connect, avoid, plan["pixel_path"],
                                  obstacle_boxes)
            if not plan["ok"]:
                self.publish_status("failed", plan["error"],
                                    parsed=parsed, timing=timing)
                return

            # --- execution phase --------------------------------------------
            self.publish_status(
                "executing",
                f"Executing {len(plan['waypoints'])} waypoints.",
                parsed=parsed, timing=timing)
            t0 = time.time()
            executed = self._execute(plan["waypoints"])
            timing["execute"] = round(time.time() - t0, 3)
            timing["total"] = round(time.time() - t_start, 3)

            if self._abort.is_set():
                self.publish_status("aborted", "Drawing aborted.",
                                    parsed=parsed, timing=timing)
                return

            self.publish_status(
                "done",
                f"Drawing complete: {executed} waypoints in "
                f"{timing['total']:.1f} s.",
                parsed=parsed, timing=timing, waypoints=executed)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"autonomous run failed: {exc}")
            self.publish_status("failed", f"Internal error: {exc}")
        finally:
            self._busy = False

    def _wait_for_detections(self):
        timeout = float(self.get_parameter("detection_timeout_sec").value)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                fresh = (time.time() - self._latest_boxes_stamp) < timeout
                boxes = list(self._latest_boxes) if fresh else []
            if boxes:
                return boxes
            if self._abort.is_set():
                return None
            time.sleep(0.05)
        return None

    def _execute(self, waypoints: list[dict]) -> int:
        if self.get_parameter("reset_before_draw").value:
            self._send_command("reset")
            time.sleep(1.0)

        do_publish = bool(self.get_parameter("publish_angles").value)
        dt = float(self.get_parameter("seconds_per_waypoint").value)
        count = 0
        for wp in waypoints:
            if self._abort.is_set():
                break
            q = wp.get("q")
            if q is None:
                continue
            if do_publish:
                msg = Float32MultiArray()
                msg.data = [float(v) for v in q]
                self.angles_pub.publish(msg)
            count += 1
            time.sleep(max(0.02, dt))
        return count

    def _send_command(self, command: str):
        msg = String()
        msg.data = command
        self.commands_pub.publish(msg)

    # ----------------------------------------------------------------- output
    def publish_status(self, state: str, message: str, **extra):
        payload = {"state": state, "message": message}
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(f"[{state}] {message}")

    def _publish_overlay(self, boxes, connect, avoid, pixel_path,
                         obstacle_boxes=None):
        try:
            import cv2
            if self._bridge is None:
                from cv_bridge import CvBridge
                self._bridge = CvBridge()

            w = int(self.get_parameter("image_width").value)
            h = int(self.get_parameter("image_height").value)
            with self._lock:
                frame = None if self._latest_frame is None else self._latest_frame.copy()
            if frame is None:
                frame = np.full((h, w, 3), 40, dtype=np.uint8)

            connect_l = {c.lower() for c in connect}
            avoid_l = {c.lower() for c in avoid}
            margin = int(self.get_parameter("margin_cells").value) * \
                int(self.get_parameter("cell_size").value)

            for b in boxes:
                p0 = (int(b["start_x"]), int(b["start_y"]))
                p1 = (int(b["end_x"]), int(b["end_y"]))
                name = b["class_name"].lower()
                if name in avoid_l:
                    color = (0, 0, 255)        # red
                elif name in connect_l:
                    color = (0, 200, 0)        # green
                else:
                    color = (160, 160, 160)    # gray
                cv2.rectangle(frame, p0, p1, color, 2)
                cv2.putText(frame, f"{b['class_name']} {b['score']:.2f}",
                            (p0[0], max(0, p0[1] - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            # Inflated obstacle clearance (what the planner actually blocks).
            for b in (obstacle_boxes or []):
                p0 = (int(b["start_x"]) - margin, int(b["start_y"]) - margin)
                p1 = (int(b["end_x"]) + margin, int(b["end_y"]) + margin)
                cv2.rectangle(frame, p0, p1, (0, 0, 180), 1)

            # Planned path polyline + waypoints.
            pts = [(int(round(p[0])), int(round(p[1]))) for p in pixel_path]
            for a, c in zip(pts[:-1], pts[1:]):
                cv2.line(frame, a, c, (255, 255, 0), 2)
            for p in pts:
                cv2.circle(frame, p, 3, (0, 255, 255), -1)

            self.debug_pub.publish(
                self._bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

            save_path = self.get_parameter("debug_image_save_path").value
            if save_path:
                cv2.imwrite(save_path, frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"overlay failed: {exc}", once=True)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDrawNode()
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
