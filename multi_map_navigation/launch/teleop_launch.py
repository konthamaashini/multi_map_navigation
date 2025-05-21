# multi_map_navigation/launch/teleop_launch.py

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit # For controller loading order

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_map_navigation')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_urdf.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'room.world')
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    ld = LaunchDescription()

    # 1. Gazebo + world + Factory plugin
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': world_path,
                'gz_args': '-s libgazebo_ros_factory.so'
            }.items(),
        )
    )

    # 2. Robot State Publisher (publishes /robot_description)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True}],
        output='screen'
    )
    ld.add_action(robot_state_publisher_node)

    # 3. TimerAction to delay the spawning of the robot
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'bmw_car',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05'
        ],
        output='screen',
    )
    ld.add_action(
        TimerAction(
            period=5.0, # Delay for 5 seconds
            actions=[spawn_robot_node]
        )
    )

    # 4. ROS 2 Control Node (controller manager)
    # This node needs to be launched after the robot is spawned in Gazebo
    # to ensure it can find the 'hardware_interface'
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml_path], # Load your controllers.yaml
        output='screen',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description') # Example remapping
            # You might not need this specific remapping, but keeping it in mind
        ]
    )

    # 5. Load Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    # 6. Load Diff Drive Controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    # 7. Teleop Twist Keyboard Node
    # Remap the /cmd_vel topic to the diff_drive_controller's command topic
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e', # Opens in a new terminal, useful for keyboard input
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped') # IMPORTANT REMAPPING
        ],
        parameters=[
            {'use_sim_time': True} # Ensure teleop uses sim time
        ]
    )

    # Order of operations using RegisterEventHandler:
    # 1. Spawn robot
    # 2. Launch controller manager
    # 3. Load joint state broadcaster
    # 4. Load diff drive controller
    # 5. Launch teleop node

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot_node,
                on_exit=[
                    controller_manager_node,
                    # Load joint_state_broadcaster first
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=joint_state_broadcaster_spawner,
                            on_exit=[
                                # Then load diff_drive_controller
                                RegisterEventHandler(
                                    OnProcessExit(
                                        target_action=diff_drive_controller_spawner,
                                        on_exit=[
                                            teleop_node # Finally, launch teleop
                                        ]
                                    )
                                )
                            ]
                        )
                    )
                ]
            )
        )
    )

    return ld