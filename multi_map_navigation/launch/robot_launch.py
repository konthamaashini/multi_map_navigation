#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction # Import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share   = get_package_share_directory('multi_map_navigation')
    gazebo_pkg  = get_package_share_directory('gazebo_ros')

    urdf_path  = os.path.join(pkg_share, 'urdf',   'robot_urdf.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'room.world')

    with open(urdf_path, 'r') as f: # Use 'r' for read mode explicitly
        robot_desc = f.read()

    return LaunchDescription([
        # 1) Gazebo + world + Factory plugin
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world':   world_path,
                # The 'gz_args' here is crucial for the factory plugin
                # Some versions of gazebo.launch.py might expect 'extra_gazebo_args'
                # but 'gz_args' is common. Confirm this in your gazebo.launch.py source.
                'gz_args': '-s libgazebo_ros_factory.so'
            }.items(),
        ),

        # 2) Robot State Publisher (publishes /robot_description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': True}],
            output='screen' # Good to see output
        ),

        # 3) TimerAction to delay the spawning of the robot
        TimerAction(
            period=5.0,  # Delay for 5 seconds
            actions=[
                # 4) Spawn the robot (moved inside TimerAction)
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'bmw_car',
                        '-topic',  'robot_description',
                        '-x', '0', '-y', '0', '-z', '0.05'
                    ],
                    output='screen',
                ),
            ]
        )
    ])