#!/usr/bin/env python

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
import time
from math import *

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class prarobClientNode(Node):
    def __init__(self):
        super().__init__('prarob_client_node')

        # Define publisher
        self.robot_goal_publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)




        self.get_clock().sleep_for(Duration(seconds=3.0))


        self.dance()
        return
        
        print(self.move_robot([0, 0.0, 0.0]))
        self.get_clock().sleep_for(Duration(seconds=1.0))
        
        
        print(self.move_robot([radians(90), 0.0, 0.0]))
        self.get_clock().sleep_for(Duration(seconds=1.0))
        
        
        print(self.move_robot([0, 0.0, 0.0]))
        self.get_clock().sleep_for(Duration(seconds=1.0))
        
        
        print(self.move_robot([radians(-90), 0.0, 0.0]))
        self.get_clock().sleep_for(Duration(seconds=1.0))
        
        
        
        print(self.move_robot([0, 0.0, 0.0]))
        self.get_clock().sleep_for(Duration(seconds=1.0))
        
        
        print(self.move_robot([0, radians(90), 0.0]))
        self.get_clock().sleep_for(Duration(seconds=1.0))
        
        print(self.move_robot([0, 0.0, 0.0]))
        self.get_clock().sleep_for(Duration(seconds=1.0))
        
        
        print(self.move_robot([0, 0.0, radians(90)]))
        self.get_clock().sleep_for(Duration(seconds=1.0))
        
        print(self.move_robot([0, 0.0, radians(-90)]))
        self.get_clock().sleep_for(Duration(seconds=1.0))
        
        print(self.move_robot([0, 0.0, 0.0]))
        self.get_clock().sleep_for(Duration(seconds=1.0))

        
        
        
        #print(self.move_robot([-1.0, 0.0, -2.0]))
        #print(self.move_robot([0, 0.0, 2.0]))
        #self.get_clock().sleep_for(Duration(seconds=1.0))
        #print(self.move_robot([0, -1.0, 2.0]))
        #self.get_clock().sleep_for(Duration(seconds=1.0))
        #print(self.move_robot([1.0, 0.0, -2.0]))

        
    def dance(self):
        while True:
            r = radians(90)
                
            print(self.move_robot([0*r,0, 0]))
            self.get_clock().sleep_for(Duration(seconds=1.0))
            
                            
            print(self.move_robot([1*r,r, 0]))
            self.get_clock().sleep_for(Duration(seconds=1.0))
            
            print(self.move_robot([2*r,0,0]))
            self.get_clock().sleep_for(Duration(seconds=1.0))
            
            print(self.move_robot([3*r,r, 0]))
            self.get_clock().sleep_for(Duration(seconds=1.0))
            

        

    def move_robot(self, q):
       
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