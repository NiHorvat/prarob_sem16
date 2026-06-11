"""Single-click bring-up for the time challenge.

Starts the robot (sim or real), the USB camera, YOLO detection, the autonomous
orchestrator and (optionally) the PyQt GUI.  Override anything from the CLI::

    ros2 launch prarob_autonomous challenge.launch.py use_sim:=false device:=cpu \
        board_homography_file:=/home/user/board_homography.yaml

Then type the command in the GUI 'Autonomous Mode' tab, or publish it::

    ros2 topic pub --once /autonomous_draw_node/command std_msgs/msg/String \
        "{data: 'connect car and plane, avoid football'}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autonomous_pkg = get_package_share_directory('prarob_autonomous')
    robot_pkg = get_package_share_directory('robot_controllv2')
    calib_pkg = get_package_share_directory('prarob_calib')
    yolo_pkg = get_package_share_directory('yolo_bringup')

    params = os.path.join(autonomous_pkg, 'config', 'autonomous_params.yaml')
    camera_params = os.path.join(calib_pkg, 'config', 'camera_params.yaml')

    use_sim = LaunchConfiguration('use_sim')
    use_camera = LaunchConfiguration('use_camera')
    use_yolo = LaunchConfiguration('use_yolo')
    use_gui = LaunchConfiguration('use_gui')
    device = LaunchConfiguration('device')
    homography_file = LaunchConfiguration('board_homography_file')
    gui_path = LaunchConfiguration('gui_path')

    args = [
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='true: mock sim, false: real robot'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_yolo', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('device', default_value='cpu',
                              description='YOLO device: cpu or cuda:0'),
        DeclareLaunchArgument('board_homography_file', default_value=''),
        DeclareLaunchArgument(
            'gui_path',
            default_value=os.path.expanduser(
                '~/Desktop/robo/prarob_sem16/GUI/robot_gui.py')),
    ]

    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg, 'launch', 'sim.launch.py')),
        condition=IfCondition(use_sim),
    )
    robot_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg, 'launch', 'robot.launch.py')),
        condition=UnlessCondition(use_sim),
    )

    camera = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='camera',
        output='screen', parameters=[camera_params],
        condition=IfCondition(use_camera),
    )

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_pkg, 'launch', 'yolov12.launch.py')),
        launch_arguments={
            'input_image_topic': '/image_raw',
            'device': device,
            'threshold': '0.25',
        }.items(),
        condition=IfCondition(use_yolo),
    )

    autonomous = Node(
        package='prarob_autonomous', executable='autonomous_draw_node',
        name='autonomous_draw_node', output='screen',
        parameters=[params, {'board_homography_file': homography_file}],
    )

    gui = ExecuteProcess(
        cmd=['python3', gui_path],
        output='screen',
        condition=IfCondition(use_gui),
    )

    return LaunchDescription(
        args + [robot_sim, robot_real, camera, yolo, autonomous, gui])
