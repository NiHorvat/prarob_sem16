# Simulation IK launch for Robot_2.0.
#
# Mirrors the structure of the original prarob_manipulator.launch.py but uses
# mock_components/GenericSystem hardware so no physical robot is needed.
#
# Starts:
#   ros2_control_node         (mock hardware + controller manager)
#   joint_state_broadcaster   (reads mock states -> /joint_states)
#   velocity_controller
#   joint_trajectory_controller
#   robot_state_publisher     (/joint_states -> TF)
#   ik_solver                 (subscribes /target_position -> trajectory controller)
#   rviz2
#
# Launch:
#   ros2 launch robot_controll robot_2_ik_sim.launch.py
#
# Send a target (separate terminal):
#   ros2 topic pub --once /target_position geometry_msgs/msg/Point \
#       "{x: 0.15, y: 0.0, z: 0.28}"

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('robot_controll')

    robot_desc = xacro.process_file(
        os.path.join(pkg, 'urdf', 'robot_2.urdf.xacro'),
        mappings={
            'use_ros2_control':  'true',
            'use_mock_hardware': 'true',
        },
    ).toxml()

    controller_config = os.path.join(pkg, 'controllers', 'controllers.yaml')
    rviz_config       = os.path.join(pkg, 'launch', 'robot_2.rviz')

    return LaunchDescription([

        # --- controller manager with mock hardware ---
        # robot_description is NOT passed inline here — ros2_control_node subscribes
        # to /robot_description from robot_state_publisher (single receipt avoids
        # the Jazzy auto-activation race that breaks joint_state_broadcaster loading)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config],
            output='screen',
        ),

        # --- controller spawners ---
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
            output='screen',
        ),

        # --- robot_state_publisher: /joint_states -> TF -> RViz ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),

        # --- IK solver: /target_position -> trajectory controller ---
        Node(
            package='robot_controll',
            executable='ik_solver.py',
            name='ik_solver',
            parameters=[{'sim_only': False}],
            output='screen',
        ),

        # --- Manual controller: /manual_joints + /reset_joints + ros2 param set ---
        Node(
            package='robot_controll',
            executable='manual_controller.py',
            name='manual_controller',
            output='screen',
        ),

        # --- Joint sliders (delayed so controllers are active before rqt queries them) ---
        TimerAction(period=4.0, actions=[
            Node(
                package='rqt_joint_trajectory_controller',
                executable='rqt_joint_trajectory_controller',
                name='rqt_joint_trajectory_controller',
                output='screen',
            ),
        ]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
