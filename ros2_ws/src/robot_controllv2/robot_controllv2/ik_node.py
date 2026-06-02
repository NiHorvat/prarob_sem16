#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2, sqrt, acos, asin

from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray


NODE_NAME = "ik_node"


class IKNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        ################################################ declare parameters ################################################

        self.declare_parameter("xyz_subscriber_topic",  "/" + NODE_NAME + "/xyz")
        self.declare_parameter("angles_publisher_topic", "/move_servos_node/angles")

        self.declare_parameter("d1", 0.0609)  # world to joint1
        self.declare_parameter("d2", 0.0209)  # joint1 to joint2
        self.declare_parameter("L1", 0.224)   # arm length (joint2 to joint3)
        self.declare_parameter("L2", 0.125)   # pen length (joint3 to tip)

        xyz_topic    = self.get_parameter("xyz_subscriber_topic").get_parameter_value().string_value
        angles_topic = self.get_parameter("angles_publisher_topic").get_parameter_value().string_value

        ################################################ publisher ################################################

        self.angles_publisher_ = self.create_publisher(Float32MultiArray, angles_topic, 10)

        ################################################ subscriber ################################################

        self.xyz_subscriber_ = self.create_subscription(
            msg_type=Point,
            topic=xyz_topic,
            callback=self.xyz_subscriber_cb_,
            qos_profile=10
        )
        
        self.get_logger().info("Node initialisation finished")



    def xyz_subscriber_cb_(self, msg: Point):
        d1 = self.get_parameter("d1").get_parameter_value().double_value
        d2 = self.get_parameter("d2").get_parameter_value().double_value
        L1 = self.get_parameter("L1").get_parameter_value().double_value
        L2 = self.get_parameter("L2").get_parameter_value().double_value

        x, y, z = msg.x, msg.y, msg.z

        h2 = d1 + d2            # height of joint2 (shoulder) above world origin
        r  = sqrt(x**2 + y**2)  # radial distance from base axis to target
        dz = z - h2             # signed height from shoulder to target

        d = sqrt(r**2 + dz**2)  # straight-line distance from shoulder to target

        # law of cosines coefficients
        D = (L1**2 + L2**2 - d**2) / (2.0 * L1 * L2)
        Q = (L1**2 + d**2 - L2**2) / (2.0 * L1 * d)

        if abs(D) > 1.0 or abs(Q) > 1.0:
            self.get_logger().warn(
                f"Target ({x:.3f}, {y:.3f}, {z:.3f}) is out of reach"
            )
            return

        theta1 = atan2(y, x)            # base yaw
        phi    = atan2(dz, r)           # angle of shoulder→target from horizontal
        theta2 = phi + acos(Q)          # arm pitch (elbow-up solution)
        theta3 = asin(D)                # wrist angle from law of cosines

        out = Float32MultiArray()
        out.data = [float(theta1), float(theta2), float(theta3)]
        self.angles_publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
