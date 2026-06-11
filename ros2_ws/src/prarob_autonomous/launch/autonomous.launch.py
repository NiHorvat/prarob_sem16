"""Launch just the autonomous orchestrator (+ optional board calibration).

Assumes the robot/sim, camera and YOLO are already running.  For the full
single-click setup use ``challenge.launch.py`` instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('prarob_autonomous')
    params = os.path.join(pkg, 'config', 'autonomous_params.yaml')

    homography_file = LaunchConfiguration('board_homography_file')
    run_calibration = LaunchConfiguration('run_calibration')

    return LaunchDescription([
        DeclareLaunchArgument('board_homography_file', default_value=''),
        DeclareLaunchArgument('run_calibration', default_value='false'),

        Node(
            package='prarob_autonomous',
            executable='board_calibration_node',
            name='board_calibration_node',
            output='screen',
            condition=IfCondition(run_calibration),
            parameters=[{'output_file': homography_file}],
        ),

        Node(
            package='prarob_autonomous',
            executable='autonomous_draw_node',
            name='autonomous_draw_node',
            output='screen',
            parameters=[params, {'board_homography_file': homography_file}],
        ),
    ])
