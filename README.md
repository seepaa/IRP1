# Robotics_Project_1

CS 4023/5023 Project 1: a simulated TurtleBot 4 (ROS 2 Jazzy, Gazebo
Harmonic) running a reactive, subsumption-style controller and building an
occupancy-grid map of a room and L-shaped hallway.

Package `proj1`:

| Path | What it is |
|---|---|
| `proj1/worlds/cust_robo_room.sdf` | The room and hallway (`<world name="project1_world">`) |
| `proj1/launch/simulation.launch.py` | Starts Gazebo, spawns the TurtleBot 4, runs both nodes |
| `proj1/proj1/reactive_node.py` | Controller: halt, teleop, escape, avoid, random turn, forward |
| `proj1/proj1/occupancy_mapper.py` | Mapper: publishes `/map` (`nav_msgs/OccupancyGrid`) |

## Running

Requires ROS 2 Jazzy with `turtlebot4_simulator` (for `turtlebot4_gz_bringup`)
and `teleop_twist_keyboard`. Check with:

```bash
ros2 pkg list | grep -E 'turtlebot4_gz_bringup|irobot_create_gz|teleop_twist_keyboard'
```

Build and launch

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select proj1
source install/setup.bash
ros2 launch proj1 simulation.launch.py

```


Keyboard control (terminal 2). Keys go to `/teleop_cmd`, not `/cmd_vel`,
so the controller stays in charge and a bump still halts the robot:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/teleop_cmd

```

Click into this terminal before pressing keys. Each key press holds control
for `teleop_timeout` (5 s of sim time, longer in real time when Gazebo runs
slow), then the robot goes back to driving itself.

If the robot ignores the keyboard, check that key presses are reaching the
controller (terminal 3):

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /teleop_cmd geometry_msgs/msg/TwistStamped

```

Press a key in the teleop terminal. A message should appear here and the
controller should log `Behavior: teleop`. If nothing appears, the problem is
on the keyboard side (wrong terminal focused, missing remap, or teleop not
running), not in the controller.

View the map: `rviz2`, set Fixed Frame to `odom`, add a Map display on `/map`.

If `ros2 topic info /cmd_vel` shows `geometry_msgs/msg/Twist`, relaunch with
`stamped:=false` and drop `-p stamped:=true` from the teleop command.
