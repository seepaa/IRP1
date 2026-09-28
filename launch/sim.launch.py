"""Launch Gazebo Harmonic with Thomas's world and spawn a TurtleBot 4."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# cust_robo_room.sdf declares <world name="project1_world">. Gazebo is given
# the file path; the TurtleBot 4 spawn/bridge launch needs the world *name*.
WORLD_FILE = 'cust_robo_room.sdf'
WORLD_NAME = 'project1_world'

# Default start pose: centre of the room, facing the doorway (+x).
ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='TurtleBot 4 model'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start RViz'),
    DeclareLaunchArgument('x', default_value='-0.762',
                          description='Start x [m] in the world frame'),
    DeclareLaunchArgument('y', default_value='0.762',
                          description='Start y [m] in the world frame'),
    DeclareLaunchArgument('z', default_value='0.0',
                          description='Start z [m] in the world frame'),
    DeclareLaunchArgument('yaw', default_value='0.0',
                          description='Start yaw [rad] in the world frame'),
]


def generate_launch_description():
    """Start Gazebo with the project world and spawn the TurtleBot 4."""
    pkg_share = get_package_share_directory('reactive_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tb4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_create_description = get_package_share_directory(
        'irobot_create_description')

    world_path = os.path.join(pkg_share, 'worlds', WORLD_FILE)

    # Let Gazebo resolve the TurtleBot 4 / Create 3 meshes (model://...).
    resource_paths = [
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', str(Path(pkg_tb4_description).parent)),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', str(Path(pkg_create_description).parent)),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'{world_path} -r -v 4',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_gz_bringup, 'launch',
                         'turtlebot4_spawn.launch.py')),
        launch_arguments={
            'world': WORLD_NAME,
            'model': LaunchConfiguration('model'),
            'rviz': LaunchConfiguration('rviz'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    for action in resource_paths:
        ld.add_action(action)
    ld.add_action(gz_sim)
    ld.add_action(clock_bridge)
    ld.add_action(robot_spawn)
    return ld
