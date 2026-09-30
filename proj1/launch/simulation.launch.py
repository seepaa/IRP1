"""
Launch Project 1: Gazebo world, TurtleBot 4, reactive controller and mapper.

Everything the TA may change is a launch argument, e.g.::

    ros2 launch proj1 simulation.launch.py x:=0.5 y:=-2.0 yaw:=1.57
    ros2 launch proj1 simulation.launch.py \
        world:=/path/to/other.sdf world_name:=<its <world name="...">>
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start Gazebo, spawn the TurtleBot 4, run controller and mapper."""
    pkg_share = get_package_share_directory('proj1')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tb4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_create_description = get_package_share_directory(
        'irobot_create_description')

    default_world = os.path.join(pkg_share, 'worlds', 'cust_robo_room.sdf')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Path to the Gazebo world .sdf file'),
        # The TurtleBot 4 bridges need the <world name="..."> from inside
        # the .sdf, which is not the same as the file name.
        DeclareLaunchArgument(
            'world_name', default_value='project1_world',
            description='The <world name="..."> declared in the world file'),
        DeclareLaunchArgument(
            'model', default_value='standard',
            choices=['standard', 'lite'], description='TurtleBot 4 model'),
        # Default start pose: centre of the room, facing the doorway (+x).
        DeclareLaunchArgument('x', default_value='-0.762',
                              description='Start x [m]'),
        DeclareLaunchArgument('y', default_value='0.762',
                              description='Start y [m]'),
        DeclareLaunchArgument('z', default_value='0.0',
                              description='Start z [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Start yaw [rad]'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Start RViz'),
        DeclareLaunchArgument(
            'stamped', default_value='true', choices=['true', 'false'],
            description='Publish TwistStamped (true) or Twist on /cmd_vel'),
        DeclareLaunchArgument(
            'disable_reflexes', default_value='true',
            choices=['true', 'false'],
            description="Turn off the Create 3's built-in bump reflexes so "
                        'our halt behavior is in charge'),
    ]

    # Let Gazebo find the TurtleBot 4 / Create 3 meshes (model://...).
    resource_paths = [
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', str(Path(pkg_tb4_description).parent)),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', str(Path(pkg_create_description).parent)),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r -v 2'],
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

    # Spawns the robot and dock, and bridges /scan, /odom, /cmd_vel,
    # /hazard_detection, /tf etc. between Gazebo and ROS.
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_gz_bringup, 'launch',
                         'turtlebot4_spawn.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world_name'),
            'model': LaunchConfiguration('model'),
            'rviz': LaunchConfiguration('rviz'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    controller = Node(
        package='proj1',
        executable='reactive_node',
        name='subsumption_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'stamped': ParameterValue(
                LaunchConfiguration('stamped'), value_type=bool),
        }],
    )

    mapper = Node(
        package='proj1',
        executable='occupancy_mapper',
        name='occupancy_mapper',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # The Create 3 base backs away from bumps on its own by default, which
    # would fight "halt on bumper". Switch that off once its node is up.
    disable_reflexes = TimerAction(
        period=20.0,
        condition=IfCondition(LaunchConfiguration('disable_reflexes')),
        actions=[ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/motion_control',
                 'reflexes_enabled', 'false'],
            output='screen')],
    )

    ld = LaunchDescription(arguments)
    for action in resource_paths:
        ld.add_action(action)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(robot_spawn)
    ld.add_action(controller)
    ld.add_action(mapper)
    ld.add_action(disable_reflexes)
    return ld
