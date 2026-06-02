#!/usr/bin/env python3
"""
Manual joint controller for Robot_2.0.

Provides a "slider" API via ROS2 topics and dynamic parameters.
Every time a topic is published or a parameter is changed the robot moves.

Topics
------
  Subscribe  /manual_joints   sensor_msgs/JointState   joint angles in DEGREES
  Subscribe  /reset_joints    std_msgs/Empty            send all joints to 0°
  Publish    /joint_trajectory_controller/joint_trajectory

Parameters (change live with  ros2 param set /manual_controller <name> <value>)
----------
  joint1_deg   float  [-90, 90]  arm (shoulder, Y axis)
  joint2_deg   float  [-90, 90]  end effector (elbow, Y axis)
  joint3_deg   float  [-90, 90]  base rotator (Z axis)
  duration_sec float  (default 1.0)  trajectory execution time
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ManualController(Node):

    JOINTS = ['joint1', 'joint2', 'joint3']
    LIMIT  = 90.0   # degrees

    def __init__(self):
        super().__init__('manual_controller')

        self.declare_parameter('joint1_deg',   0.0)
        self.declare_parameter('joint2_deg',   0.0)
        self.declare_parameter('joint3_deg',   0.0)
        self.declare_parameter('duration_sec', 1.0)

        self._pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )

        self.create_subscription(JointState, '/manual_joints', self._joints_cb, 10)
        self.create_subscription(Empty,      '/reset_joints',  self._reset_cb,  10)

        self.add_on_set_parameters_callback(self._param_cb)

        self.get_logger().info(
            'Manual controller ready.\n'
            '  Topics : /manual_joints (JointState, deg)  |  /reset_joints (Empty)\n'
            '  Params : ros2 param set /manual_controller joint1_deg 45.0'
        )

    # ------------------------------------------------------------------ #
    def _send(self, degs, duration_sec=None):
        if duration_sec is None:
            duration_sec = self.get_parameter('duration_sec').value

        rad = []
        for name, d in zip(self.JOINTS, degs):
            clamped = max(-self.LIMIT, min(self.LIMIT, d))
            if abs(clamped - d) > 0.01:
                self.get_logger().warn(f'{name}: {d:.1f}° clamped to ±{self.LIMIT}°')
            rad.append(math.radians(clamped))

        traj = JointTrajectory()
        traj.joint_names = self.JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = rad
        sec = int(duration_sec)
        ns  = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=ns)
        traj.points = [pt]
        self._pub.publish(traj)

        self.get_logger().info(
            f'j1={degs[0]:+.1f}°  j2={degs[1]:+.1f}°  j3={degs[2]:+.1f}°'
        )

    def _joints_cb(self, msg: JointState):
        angle_map = {n: math.degrees(p) for n, p in zip(msg.name, msg.position)}
        degs = [angle_map.get(j, 0.0) for j in self.JOINTS]
        self._send(degs)

    def _reset_cb(self, _):
        self._send([0.0, 0.0, 0.0], duration_sec=1.5)
        for j in self.JOINTS:
            self.set_parameters([
                rclpy.parameter.Parameter(
                    f'{j}_deg',
                    rclpy.parameter.Parameter.Type.DOUBLE,
                    0.0,
                )
            ])

    def _param_cb(self, params):
        # Build current degree values, apply any incoming changes
        degs = [self.get_parameter(f'{j}_deg').value for j in self.JOINTS]
        for p in params:
            for i, j in enumerate(self.JOINTS):
                if p.name == f'{j}_deg':
                    degs[i] = p.value
        # Only move on joint-angle changes (ignore duration_sec param)
        joint_params = {p.name for p in params}
        if any(f'{j}_deg' in joint_params for j in self.JOINTS):
            self._send(degs)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ManualController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
