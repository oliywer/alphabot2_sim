"""Gazebo Harmonic simulation of the AlphaBot2 (robot + world + ROS bridge)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('alphabot2_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    use_lidar = LaunchConfiguration('lidar')
    use_rviz = LaunchConfiguration('rviz')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')

    xacro_file = os.path.join(pkg, 'urdf', 'alphabot2.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_lidar:=', use_lidar]),
        value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world],
            'on_exit_shutdown': 'true',
        }.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'alphabot2',
                   '-x', x, '-y', y, '-z', '0.01'])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'alphabot2.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument('world',
                              default_value=os.path.join(pkg, 'worlds', 'office.sdf'),
                              description='Full path to the SDF world'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='Add the virtual 2D lidar (needed for SLAM/Nav2)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start RViz (fixed frame "map" -> use with nav.launch.py)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        rviz,
    ])
