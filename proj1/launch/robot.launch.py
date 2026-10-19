"""
Launch Project 2: the reactive controller on a physical TurtleBot 4.

Run this on the lab desktop, not on the robot, after running
``robot-setup.sh`` and its exports in the same terminal so this machine can
see the robot's topics. Nothing is simulated: the robot's own drivers
already publish /scan, /odom, /hazard_detection and /tf and accept
/cmd_vel. This file only:

* starts the lidar motor (and stops it again on Ctrl+C),
* turns off the Create 3's built-in bump reflexes,
* runs the reactive controller with speeds suited to the lab, and
* optionally runs the occupancy mapper.

Tuning is done through launch arguments, e.g.::

    ros2 launch proj1 robot.launch.py forward_speed:=0.08 mapper:=true
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Same retry loop as simulation.launch.py. The Create 3 turns its reflexes
# back on at every reboot, so this has to run on every launch.
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

def lidar_cmd(service):
    """Return the command that calls the lidar's start/stop service."""
    return ['ros2', 'service', 'call', service, 'std_srvs/srv/Empty', '{}']


def stop_lidar(context):
    """
    Stop the lidar motor while the launch shuts down.

    Run synchronously: a process started from OnShutdown with ExecuteProcess
    can be killed before it finishes. The robot must not go back on its dock
    with the lidar spinning.
    """
    print('[proj1] Stopping lidar motor...')
    try:
        subprocess.run(lidar_cmd('/stop_motor'), timeout=10.0, check=False,
                       stdout=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        print('[proj1] WARNING: /stop_motor timed out; stop the lidar by '
              'hand: ros2 service call /stop_motor std_srvs/srv/Empty "{}"')
    return []


def generate_launch_description():
    """Start the lidar, disable reflexes, run controller (and mapper)."""
    arguments = [
        DeclareLaunchArgument(
            'stamped', default_value='true', choices=['true', 'false'],
            description='Publish TwistStamped (true) or Twist on /cmd_vel'),
        DeclareLaunchArgument(
            'forward_speed', default_value='0.1',
            description='Forward speed [m/s] (OU lab guide suggests 0.1)'),
        DeclareLaunchArgument(
            'turn_speed', default_value='0.5',
            description='In-place turn speed [rad/s]'),
        DeclareLaunchArgument(
            'disable_reflexes', default_value='true',
            choices=['true', 'false'],
            description="Turn off the Create 3's built-in bump reflexes so "
                        'our halt behavior is in charge'),
        DeclareLaunchArgument(
            'start_lidar', default_value='true', choices=['true', 'false'],
            description='Start the lidar motor now and stop it on exit'),
        DeclareLaunchArgument(
            'halt_on_cliff', default_value='true', choices=['true', 'false'],
            description='Halt on cliff / wheel-drop hazards as well as bumps'),
        DeclareLaunchArgument(
            'mapper', default_value='false', choices=['true', 'false'],
            description='Also run the occupancy-grid mapper (/map)'),
    ]

    # Blocks until the robot's /start_motor service is discovered.
    start_lidar = ExecuteProcess(
        cmd=lidar_cmd('/start_motor'),
        condition=IfCondition(LaunchConfiguration('start_lidar')),
        output='screen')

    stop_lidar_on_exit = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=stop_lidar)]),
        condition=IfCondition(LaunchConfiguration('start_lidar')))

    controller = Node(
        package='proj1',
        executable='reactive_node',
        name='subsumption_controller',
        output='screen',
        parameters=[{
            # Real hardware: wall-clock time, no Gazebo /clock.
            'use_sim_time': False,
            'stamped': ParameterValue(
                LaunchConfiguration('stamped'), value_type=bool),
            'forward_speed': ParameterValue(
                LaunchConfiguration('forward_speed'), value_type=float),
            'turn_speed': ParameterValue(
                LaunchConfiguration('turn_speed'), value_type=float),
            'halt_on_cliff': ParameterValue(
                LaunchConfiguration('halt_on_cliff'), value_type=bool),
            # Real odometry is noisier than Gazebo's; a tighter tolerance
            # makes turns hunt back and forth around the target.
            'heading_tolerance_deg': 5.0,
            # RPLIDAR A1's own minimum; also hides the robot's tower posts.
            'min_valid_range': 0.15,
        }],
    )

    mapper = Node(
        package='proj1',
        executable='occupancy_mapper',
        name='occupancy_mapper',
        output='screen',
        condition=IfCondition(LaunchConfiguration('mapper')),
        parameters=[{'use_sim_time': False}],
    )

    # The Create 3 backs away from bumps on its own by default, which would
    # fight "halt on bumper".
    disable_reflexes = TimerAction(
        period=2.0,
        condition=IfCondition(LaunchConfiguration('disable_reflexes')),
        actions=[ExecuteProcess(
            cmd=['bash', '-c', DISABLE_REFLEXES_SCRIPT],
            output='screen')],
    )

    ld = LaunchDescription(arguments)
    ld.add_action(start_lidar)
    ld.add_action(stop_lidar_on_exit)
    ld.add_action(disable_reflexes)
    ld.add_action(controller)
    ld.add_action(mapper)
    return ld
