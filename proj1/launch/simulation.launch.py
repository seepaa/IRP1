import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('proj1')
    world_path = os.path.join(pkg_share, 'worlds', 'cust_robo_room.sdf')

    # 1. Launch Gazebo Sim (Package: ros_gz_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. Bridge Gazebo topics to ROS 2 (Package: ros_gz_bridge)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',  # <-- MUST BE ros_gz_bridge, NOT ros_gz_sim
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        ros_gz_bridge,
	Node(
	    package='proj1',
	    executable='reactive_node',
	    name='subsumption_controller',
	    output='screen'
	)
    ])
