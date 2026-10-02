"""
Launch Project 1: Gazebo world, TurtleBot 4, reactive controller and mapper.

Everything the TA may change is a launch argument, e.g.::

    ros2 launch proj1 simulation.launch.py x:=0.5 y:=-2.0 yaw:=1.57
    ros2 launch proj1 simulation.launch.py world:=/path/to/other.sdf

The <world name="..."> inside the .sdf is read automatically; pass
world_name:=... only to override it.
"""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Retry for up to ~2 minutes: on a slow machine the Create 3's
# /motion_control node can take a while to come up, and a single attempt
# would fail silently and leave the bump reflexes on.
DISABLE_REFLEXES_SCRIPT = """
for i in $(seq 1 60); do
  out=$(ros2 param set /motion_control reflexes_enabled false 2>&1)
  case "$out" in
    *successful*) echo "[proj1] Create 3 reflexes disabled"; exit 0 ;;
  esac
  sleep 2
done
echo "[proj1] WARNING: could not disable Create 3 reflexes: $out" >&2
exit 1
"""


def world_name_from_sdf(path):
    """Return the name attribute of the <world> element in an SDF file."""
    world = ET.parse(path).getroot().find('world')
    if world is None or not world.get('name'):
        raise RuntimeError(f'No <world name="..."> found in {path}')
    return world.get('name')


def spawn_robot(context):
    """Spawn the TurtleBot 4 into the world named inside the .sdf file."""
    world_name = LaunchConfiguration('world_name').perform(context)
    if not world_name:
        world_name = world_name_from_sdf(
            LaunchConfiguration('world').perform(context))
    pkg_tb4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    # Spawns the robot and dock, and bridges /scan, /odom, /cmd_vel,
    # /hazard_detection, /tf etc. between Gazebo and ROS. The /scan bridge
    # is built from the world name, so a wrong name leaves the robot blind.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_gz_bringup, 'launch',
                         'turtlebot4_spawn.launch.py')),
        launch_arguments={
            'world': world_name,
            'model': LaunchConfiguration('model'),
            'rviz': LaunchConfiguration('rviz'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )]


def generate_launch_description():
    """Start Gazebo, spawn the TurtleBot 4, run controller and mapper."""
    pkg_share = get_package_share_directory('proj1')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
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
        # the .sdf, which is not the same as the file name. Empty means
        # "read it from the world file".
        DeclareLaunchArgument(
            'world_name', default_value='',
            description='Override for the <world name="..."> in the world '
                        'file (default: read from the file)'),
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

    # Resolved at launch time so the world name can be read from the file.
    robot_spawn = OpaqueFunction(function=spawn_robot)

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
    # would fight "halt on bumper". Switch that off once its node is up,
    # retrying until it is (see DISABLE_REFLEXES_SCRIPT).
    disable_reflexes = TimerAction(
        period=5.0,
        condition=IfCondition(LaunchConfiguration('disable_reflexes')),
        actions=[ExecuteProcess(
            cmd=['bash', '-c', DISABLE_REFLEXES_SCRIPT],
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
