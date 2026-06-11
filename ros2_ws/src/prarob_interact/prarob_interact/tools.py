"""Custom tools to help ROSA with ROS2 service/message introspection."""

import subprocess
from langchain.agents import tool
import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from yolo_msgs.msg import DetectionArray
from prarob_interact.kinematics import Kinematics
from prarob_interact.path_planning import (
    build_obstacle_grid,
    image_path_to_task_path,
    plan_image_path,
)
import time

JOINT_NAMES = ['joint1', 'joint2', 'joint3']


def _extract_joint_positions(msg: JointState) -> list[float]:
    """Return [joint1, joint2, joint3] from a JointState message."""
    if msg.name:
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name for name in JOINT_NAMES):
            return [float(by_name[name]) for name in JOINT_NAMES]

    if len(msg.position) < 3:
        raise ValueError("JointState message does not contain three joint positions")
    return [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]


def _read_current_joints(node, timeout_sec: float = 1.0) -> list[float] | None:
    received_msg = None

    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    subscription = node.create_subscription(JointState, '/joint_states', callback, 10)
    try:
        start_time = node.get_clock().now()
        while received_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > timeout_sec * 1e9:
                return None
        return _extract_joint_positions(received_msg)
    finally:
        node.destroy_subscription(subscription)


@tool
def ros2_interface_show(interface_type: str) -> str:
    """Show the full definition of a ROS2 message, service, or action type.

    Use this as a fallback when you need to check an interface that was NOT
    included in the pre-scanned environment snapshot (e.g. a node that
    started after the agent was created).

    Args:
        interface_type: Full interface type, e.g. 'geometry_msgs/msg/Point'
                        or 'crazyflie_interfaces/srv/GoTo'.
    """
    try:
        result = subprocess.run(
            ["ros2", "interface", "show", interface_type],
            capture_output=True, text=True, timeout=10,
        )
        if result.returncode != 0:
            return f"Error: {result.stderr.strip()}"
        return result.stdout.strip()
    except Exception as e:
        return f"Error running ros2 interface show: {e}"


@tool
def move_robot_joints(positions: list[float], duration: float = 2.0):
    """
    Moves the robot arm joints to specific positions (radians).
    Args:
        positions: A list of 4 floats representing target angles.
        duration: Time in seconds to reach the target.
    """
    if not rclpy.ok():
        rclpy.init()

    node_name = f'rosa_mover_{int(time.time())}'
    node = rclpy.create_node(node_name)

    try:
        publisher = node.create_publisher(
            Point, '/ik_node/xyz', 10)

        point = Point()
        point.x = positions[0]
        point.y = positions[1]
        point.z = positions[2]


        time.sleep(0.2)
        publisher.publish(point)
        time.sleep(0.2)  

        return f"Successfully sent command to {node_name}."

    except Exception as e:
        return f"Error: {str(e)}"

    finally:
        # Properly cleanup the node
        node.destroy_node()


@tool
def get_tool_pose(timeout_sec: float = 2.0):
    """
    Retrieves the most recent message from the /joint_states topic.
    Use this to get the current positions, velocities, or efforts of the robot's joints.

    Args:
        timeout_sec: How long to wait for a message before giving up.
    """
    # Initialize rclpy if it hasn't been initialized yet
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node('rosa_joint_state_fetcher')
    received_msg = None

    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    subscription = node.create_subscription(
        JointState,
        '/joint_states',
        callback,
        10
    )
    kinematics_node = Kinematics()

    try:
        # Spin until message is received or timeout occurs
        start_time = node.get_clock().now()
        while received_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)

            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > (timeout_sec * 1e9):
                return "Error: Timeout reached. No messages received on /joint_states."

        joint_positions = _extract_joint_positions(received_msg)
        xyz = kinematics_node.get_dk(joint_positions)
        return {"x": xyz[0], "y": xyz[1], "z": xyz[2]}

    except Exception as e:
        return f"Error retrieving joint states: {str(e)}"
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()


@tool
def move_to_pose(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, duration: float = 3.0):
    """
    Moves the robot end-effector to a specific 3D pose (X, Y, Z in meters, Roll, Pitch, Yaw in radians).
    This tool calculates the required joint angles using Inverse Kinematics.

    Args:
        x, y, z: Target position in meters.
        roll, pitch, yaw: Target orientation in radians.
        duration: Time in seconds to complete the movement.
    """
    if not rclpy.ok():
        rclpy.init()

    # Use unique node name to avoid graph collisions
    node_name = f'rosa_ik_mover_{int(time.time())}'
    node = rclpy.create_node(node_name)

    try:
        kinematics_node = Kinematics()
        target_pose = [x, y, z]
        solutions = kinematics_node.get_ik(target_pose)

        if not solutions:
            return f"Error: The pose {target_pose} is out of reach or mathematically impossible for this robot."

        current_joints = _read_current_joints(node, timeout_sec=1.0) or [0.0, 0.0, 0.0]
        joint_angles = kinematics_node.get_closest_ik(solutions, current_joints)
        if joint_angles is None:
            return "Error: No IK solution."

        publisher = node.create_publisher(
            Float32MultiArray, '/move_servos_node/angles', 10)

        msg = Float32MultiArray()
        msg.data = [float(angle) for angle in joint_angles]

        time.sleep(0.1)  # Allow publisher discovery
        publisher.publish(msg)
        time.sleep(0.1)  # Ensure message delivery before node destruction

        return (f"IK Success. Moving to Pose: X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}. "
                f"Computed Joints: {msg.data}")

    except Exception as e:
        return f"Hardware error during IK movement: {str(e)}"

    finally:
        node.destroy_node()


@tool
def get_yolo_boxes(yolo_detections_topic: str, search_for: list, timeout_sec: float = 2.0):
    """
    Gets the bounding boxes of detected objects from the detections topic.
    The method searches for objects named in the search_for list of strings and returns their bounding boxes.

    Args:
        yolo_detections_topic: The topic reporting the bounding boxes of the YOLO detections.
        search_for: A list of strings, each representing an object class that YOLO should search for.
        timeout_sec: How long to wait for a detection message before timing out.
    """

    if not rclpy.ok():
        rclpy.init()

    node_name = f'yolo_fetcher_{int(time.time())}'
    node = rclpy.create_node(node_name)
    received_msg = None

    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    subscription = node.create_subscription(
        DetectionArray,
        yolo_detections_topic,
        callback,
        10
    )

    try:
        start_time = node.get_clock().now()
        while received_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > (timeout_sec * 1e9):
                return f"Error: Timeout reached. No messages received on {yolo_detections_topic}."

        if isinstance(search_for, str):
            search_for = [search_for]

        # Use exact class names from the DetectionArray.
        found = {key: None for key in search_for}
        for detection in received_msg.detections:
            if detection.class_name in found and found[detection.class_name] is None:
                bbox = detection.bbox
                cx = bbox.center.position.x
                cy = bbox.center.position.y
                w = bbox.size.x
                h = bbox.size.y
                found[detection.class_name] = {
                    'class_name': detection.class_name,
                    'start_x': cx - w / 2,
                    'start_y': cy - h / 2,
                    'end_x': cx + w / 2,
                    'end_y': cy + h / 2
                }

        if all(value is None for value in found.values()):
            return f"No matching detections found on {yolo_detections_topic} for {search_for}."

        return found

    except Exception as e:
        return f"Error retrieving YOLO detections: {str(e)}"

    finally:
        node.destroy_node()


def generate_grid(
    detections: list,
    width: int = 640,
    height: int = 480,
    field_size: int = 5,
    margin: int = 2,
):
    """Compatibility wrapper around the image-space occupancy grid builder."""
    return build_obstacle_grid(
        detections,
        width=width,
        height=height,
        cell_size=field_size,
        margin_cells=margin,
    )


def _as_jsonable_points(points: list) -> list[list[float]]:
    return [[float(value) for value in point] for point in points]


def _publish_task_path(
    path_task: list,
    xyz_topic: str = "/ik_node/xyz",
    seconds_per_waypoint: float = 0.15,
):
    if not rclpy.ok():
        rclpy.init()

    node_name = f'path_executor_{int(time.time())}'
    node = rclpy.create_node(node_name)
    publisher = node.create_publisher(Point, xyz_topic, 10)

    try:
        time.sleep(0.4)
        for waypoint in path_task:
            if len(waypoint) < 3:
                raise ValueError(f"Task waypoint must be [x, y, z], got {waypoint}")

            point = Point()
            point.x = float(waypoint[0])
            point.y = float(waypoint[1])
            point.z = float(waypoint[2])
            publisher.publish(point)
            time.sleep(max(0.02, float(seconds_per_waypoint)))

        return f"Published {len(path_task)} waypoints to {xyz_topic}."

    finally:
        node.destroy_node()


@tool
def plan_path(
    start: dict,
    goal: dict,
    obstacles: list = None,
    obstacle_grid: list = None,
    image_width: int = 640,
    image_height: int = 480,
    cell_size: int = 5,
    margin_cells: int = 2,
    workspace_x_min: float = 0.06,
    workspace_x_max: float = 0.28,
    workspace_y_min: float = -0.14,
    workspace_y_max: float = 0.14,
    drawing_z: float = 0.0,
):
    """
    Plan an obstacle-avoiding path for the marker tip.

    Args:
        start: Start point as {'x': px, 'y': py} or a YOLO box dict with
            start_x, start_y, end_x, end_y.
        goal: Goal point in the same format as start.
        obstacles: Optional list of YOLO box dicts that the path must avoid.
        obstacle_grid: Optional precomputed grid where 1 means blocked.
        image_width, image_height: Camera/image dimensions in pixels.
        cell_size: Occupancy grid cell size in pixels.
        margin_cells: Obstacle expansion in grid cells.
        workspace_*: Linear image-to-robot workspace bounds in metres.
        drawing_z: Marker tip z coordinate in metres.
    """
    try:
        planned = plan_image_path(
            start=start,
            goal=goal,
            obstacles=obstacles,
            obstacle_grid=obstacle_grid,
            width=image_width,
            height=image_height,
            cell_size=cell_size,
            margin_cells=margin_cells,
            simplify=True,
        )
        if not planned["path_found"]:
            return {"path_found": False, "error": "No free path found."}

        task_path = image_path_to_task_path(
            planned["path_pixels"],
            image_width=image_width,
            image_height=image_height,
            workspace_x_min=workspace_x_min,
            workspace_x_max=workspace_x_max,
            workspace_y_min=workspace_y_min,
            workspace_y_max=workspace_y_max,
            z=drawing_z,
        )

        return {
            "path_found": True,
            "path_pixels": _as_jsonable_points(planned["path_pixels"]),
            "path_task": _as_jsonable_points(task_path),
            "start_cell": list(planned["start_cell"]),
            "goal_cell": list(planned["goal_cell"]),
            "grid_size": planned["grid_size"],
            "cell_size": planned["cell_size"],
        }
    except Exception as e:
        return {"path_found": False, "error": str(e)}


@tool
def execute_task_path(
    path_task: list,
    xyz_topic: str = "/ik_node/xyz",
    seconds_per_waypoint: float = 0.15,
):
    """
    Execute a task-space marker-tip path by publishing each [x, y, z] waypoint
    to the existing IK node.
    """
    try:
        return _publish_task_path(
            path_task,
            xyz_topic=xyz_topic,
            seconds_per_waypoint=seconds_per_waypoint,
        )
    except Exception as e:
        return f"Error executing task path: {e}"


@tool
def plan_and_execute_path(
    start: dict,
    goal: dict,
    obstacles: list = None,
    image_width: int = 640,
    image_height: int = 480,
    cell_size: int = 5,
    margin_cells: int = 2,
    workspace_x_min: float = 0.06,
    workspace_x_max: float = 0.28,
    workspace_y_min: float = -0.14,
    workspace_y_max: float = 0.14,
    drawing_z: float = 0.0,
    xyz_topic: str = "/ik_node/xyz",
    seconds_per_waypoint: float = 0.15,
):
    """
    Plan and immediately execute an image-space path through the IK node.

    Use start/goal as YOLO box dicts or {'x': px, 'y': py}. Obstacles should
    be the YOLO boxes for objects named as avoid targets.
    """
    result = plan_path.func(
        start=start,
        goal=goal,
        obstacles=obstacles,
        obstacle_grid=None,
        image_width=image_width,
        image_height=image_height,
        cell_size=cell_size,
        margin_cells=margin_cells,
        workspace_x_min=workspace_x_min,
        workspace_x_max=workspace_x_max,
        workspace_y_min=workspace_y_min,
        workspace_y_max=workspace_y_max,
        drawing_z=drawing_z,
    )
    if not result.get("path_found"):
        return result

    execution = _publish_task_path(
        result["path_task"],
        xyz_topic=xyz_topic,
        seconds_per_waypoint=seconds_per_waypoint,
    )
    result["execution"] = execution
    return result


TOOLS = [ros2_interface_show, get_tool_pose,
         move_robot_joints, move_to_pose, get_yolo_boxes, plan_path,
         execute_task_path, plan_and_execute_path]
