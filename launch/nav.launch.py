"""SLAM (slam_toolbox) + Nav2 for the AlphaBot2 digital twin.

Run AFTER sim.launch.py:
    ros2 launch alphabot2_sim nav.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('alphabot2_sim')
    slam_pkg = get_package_share_directory('slam_toolbox')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')
    use_rviz = LaunchConfiguration('rviz')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params,
        }.items())

    lifecycle_nodes = ['controller_server', 'planner_server',
                       'behavior_server', 'bt_navigator']

    nav_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True,
                          'autostart': True,
                          'node_names': lifecycle_nodes}]),
    ]

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'alphabot2.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        slam,
        *nav_nodes,
        rviz,
    ])
