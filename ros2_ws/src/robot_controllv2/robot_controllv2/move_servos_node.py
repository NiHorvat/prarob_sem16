#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from math import radians

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray, String


NODE_NAME = "move_servos_node"

class prarobClientNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        ################################################ declare parameters ################################################

        self.declare_parameter(name="angles_subscriber_topics", value= "/" + NODE_NAME + "/angles")
        self.declare_parameter(name="commands_subscriber_topic", value= "/" + NODE_NAME + "/commands")



        ################################################ publisher ################################################

        # publisher that the controller uses
        self.robot_goal_publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)




        ################################################ subscribers ################################################

        # subscriber that ik_node will publish to
        self.angles_subscriber_ = self.create_subscription(
            msg_type=Float32MultiArray,
            topic=self.get_parameter("angles_subscriber_topics").get_parameter_value().string_value,
            callback=self.angles_subscriber_cb_,
            qos_profile=10
        )

        self.commands_subscriber_ = self.create_subscription(
            msg_type=String,
            topic=self.get_parameter("commands_subscriber_topic").get_parameter_value().string_value,
            callback=self.commands_subscriber_cb_,
            qos_profile=10
        )



        self.get_clock().sleep_for(Duration(seconds=3.0)) # wait for the robot to initialise



    def angles_subscriber_cb_(self, msg : Float32MultiArray):

        """
            - params : msg needs to be len=3 and in order joint1_angle, joint2_angle, join3_angle
        """

        if len(msg.data) != 3:
            self.get_logger().error("angles_subscriber_cb_ : received wrong msg type")
            return


        self.move_robot_(msg.data)


    def commands_subscriber_cb_(self, msg : String):
        if msg.data == "reset":
            self.move_to_init_conf_()
            return

        return


    def move_to_init_conf_(self):
        self.move_robot_([0, 0.0, radians(-90)])



    def move_robot_(self, q):

        goal_trajectory = JointTrajectory()
        goal_trajectory.joint_names.append('joint1')
        goal_trajectory.joint_names.append('joint2')
        goal_trajectory.joint_names.append('joint3')

        goal_point = JointTrajectoryPoint()
        goal_point.positions.append(q[0])
        goal_point.positions.append(q[1])
        goal_point.positions.append(q[2])
        goal_point.time_from_start = Duration(seconds=0.01).to_msg()

        goal_trajectory.points.append(goal_point)

        return self.robot_goal_publisher_.publish(goal_trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = prarobClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
