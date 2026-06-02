# Launch the Robot_2.0 simulation in RViz with joint sliders.
#
#   ros2 launch robot_controll robot_2_sim.launch.py
#
# Brings up:
#   - robot_state_publisher      (publishes TF from the URDF + /joint_states)
#   - joint_state_publisher_gui  (sliders to drive the shoulder/elbow joints)
#   - rviz2                       (visualisation)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    robot_name = "robot_2"
    package_name = "robot_controll"

    pkg_share = get_package_share_directory(package_name)

    robot_description_path = os.path.join(
        pkg_share, "urdf", robot_name + ".urdf.xacro")
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    rviz_config = os.path.join(pkg_share, "launch", "robot_2.rviz")

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch joint_state_publisher_gui with sliders.",
    )
    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        gui_arg,

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),

        # Sliders to control the joints in simulation.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(gui),
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
