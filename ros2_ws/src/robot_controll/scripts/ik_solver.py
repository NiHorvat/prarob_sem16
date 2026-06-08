#!/usr/bin/env python3
"""
Analytical IK solver for Robot_2.0 (3-DOF RRR manipulator).

Kinematic parameters (from robot_2.urdf.xacro):
    D1 = 0.0818 m   height of shoulder (joint1) above world origin
                    = joint3 origin z (0.0609) + joint1 origin z (0.0209)
    L1 = 0.224  m   upper arm  (joint1 -> joint2)
    L2 = 0.125  m   forearm+pen (joint2 -> end_effector)
                    = wrist bracket (0.020) + pen (0.105)

Joint axes:
    joint1 : Y  (arm / shoulder pitch)  Dynamixel ID 11
    joint2 : Y  (hand / end effector)   Dynamixel ID 12
    joint3 : Z  (base rotator)          Dynamixel ID 13

Subscribes:
    /target_position  (geometry_msgs/Point)  — desired end-effector position in world frame

Publishes (selected by 'sim_only' parameter):
    sim_only=True   ->  /joint_states  (sensor_msgs/JointState)
                        robot_state_publisher reads this and updates RViz
    sim_only=False  ->  /joint_trajectory_controller/joint_trajectory
                        (trajectory_msgs/JointTrajectory)
                        same mechanism as scripts/move_robot.py; drives the real robot,
                        joint_state_broadcaster then feeds /joint_states so RViz follows

Usage — send a target position:
    ros2 topic pub --once /target_position geometry_msgs/msg/Point \
        "{x: 0.15, y: 0.05, z: 0.30}"
"""

import math

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class IKSolverNode(Node):

    # Robot parameters (metres)
    D1 = 0.0818   # shoulder height above world origin
    L1 = 0.224    # upper arm length
    L2 = 0.125    # forearm + pen length

    # Order: base(joint3), arm(joint1), end_effector(joint2) — matches IK vars q1,q2,q3
    JOINT_NAMES = ['joint3', 'joint1', 'joint2']
    LIMIT = math.pi / 2.0   # ±90° limit on every joint

    def __init__(self):
        super().__init__('ik_solver')

        self.declare_parameter('sim_only', True)
        self.sim_only: bool = (
            self.get_parameter('sim_only').get_parameter_value().bool_value
        )

        self._sub = self.create_subscription(
            Point, '/target_position', self._on_target, 10
        )

        if self.sim_only:
            self._pub = self.create_publisher(JointState, '/joint_states', 10)
            self.get_logger().info(
                'IK solver ready — simulation mode.  '
                'Publish target to /target_position (geometry_msgs/Point).'
            )
        else:
            self._pub = self.create_publisher(
                JointTrajectory,
                '/joint_trajectory_controller/joint_trajectory',
                10,
            )
            self.get_logger().info(
                'IK solver ready — real-robot mode.  '
                'Publish target to /target_position (geometry_msgs/Point).'
            )

    # ------------------------------------------------------------------ #
    def _on_target(self, msg: Point) -> None:
        result = self._solve(msg.x, msg.y, msg.z)

        if result is None:
            self.get_logger().warn(
                f'Target ({msg.x:.4f}, {msg.y:.4f}, {msg.z:.4f}) is unreachable '
                f'within joint limits ±{math.degrees(self.LIMIT):.0f}°.'
            )
            return

        q1, q2, q3 = result
        self.get_logger().info(
            f'Target ({msg.x:.4f}, {msg.y:.4f}, {msg.z:.4f})  →  '
            f'j3(base)={math.degrees(q1):+.1f}°  '
            f'j1(arm)={math.degrees(q2):+.1f}°  '
            f'j2(end)={math.degrees(q3):+.1f}°'
        )

        if self.sim_only:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.JOINT_NAMES
            js.position = [q1, q2, q3]
            self._pub.publish(js)
        else:
            traj = JointTrajectory()
            traj.joint_names = self.JOINT_NAMES
            pt = JointTrajectoryPoint()
            pt.positions = [q1, q2, q3]
            pt.time_from_start = Duration(sec=1, nanosec=0)
            traj.points = [pt]
            self._pub.publish(traj)

    # ------------------------------------------------------------------ #
    def _solve(self, x: float, y: float, z: float):
        """
        Analytical IK for a 3-DOF RRR arm.

        FK (forward kinematics) at angles (q1, q2, q3):
            r_ee = L1*sin(q2) + L2*sin(q2+q3)   (radial, in arm plane)
            z_ee = D1 + L1*cos(q2) + L2*cos(q2+q3)

        Returns (q1, q2, q3) in radians, or None if unreachable.
        Both elbow-down (q3>0) and elbow-up (q3<0) are tried; the first
        solution that keeps all joints within ±LIMIT is returned.
        """
        D1, L1, L2 = self.D1, self.L1, self.L2

        # --- joint3: yaw (base rotation about Z) ---
        q1 = math.atan2(y, x)

        # --- project target onto the arm plane ---
        r = math.hypot(x, y)    # radial distance from Z axis
        h = z - D1              # height above shoulder joint

        d_sq = r * r + h * h

        # --- law of cosines for elbow angle ---
        cos_q3 = (d_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)

        if abs(cos_q3) > 1.0 + 1e-9:
            return None  # target out of reach
        cos_q3 = max(-1.0, min(1.0, cos_q3))  # clamp floating-point overshoot

        # try elbow-down first (q3 > 0), then elbow-up (q3 < 0)
        for sign in (1, -1):
            sin_q3 = sign * math.sqrt(max(0.0, 1.0 - cos_q3 * cos_q3))
            q3 = math.atan2(sin_q3, cos_q3)

            # shoulder angle: atan2(r,h) is direction to target from shoulder,
            # minus the geometric offset introduced by the elbow bend
            q2 = math.atan2(r, h) - math.atan2(L2 * sin_q3, L1 + L2 * cos_q3)

            if (abs(q1) <= self.LIMIT
                    and abs(q2) <= self.LIMIT
                    and abs(q3) <= self.LIMIT):
                return q1, q2, q3

        return None


def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
