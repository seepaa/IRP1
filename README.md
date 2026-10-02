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

## Requirements

ROS 2 Jazzy with `turtlebot4_simulator` (for `turtlebot4_gz_bringup`),
`teleop_twist_keyboard` and `nav2_map_server`. Check with:

```bash
ros2 pkg list | grep -E 'turtlebot4_gz_bringup|irobot_create_gz|teleop_twist_keyboard|nav2_map_server'
```

First-time setup:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/seepaa/IRP1
```

## Terminal setup (every terminal)

Run these in **every** terminal before any `ros2` command:

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
```

Why: the lab machines set `ROS_LOCALHOST_ONLY=1` in `/etc/bash.bashrc`. With
it, ROS only discovers nodes reliably while there are few of them. The
TurtleBot 4 sim starts about 45 nodes, so nodes started after those (the
mapper, RViz) can't find each other: the map never reaches RViz, and whether
it works changes from launch to launch. `ROS_DOMAIN_ID` keeps your sim
separate from anyone else running ROS on the network. Any number from 1 to
100 works, as long as every terminal uses the same one.

To skip typing the two `export` lines each time, add them to the end of your
own `~/.bashrc`. If one terminal can't see the sim's topics, it's missing
them.

## Running

Build and launch (terminal 1):

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select proj1
source install/setup.bash
ros2 launch proj1 simulation.launch.py
# optional: x:= y:= yaw:=  (start pose), world:=/path/to/other.sdf
# (its <world name> is read from the file automatically)
```

Keyboard control (terminal 2). Keys go to `/teleop_cmd`, not `/cmd_vel`,
so the controller stays in charge and a bump still halts the robot:

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/teleop_cmd
```

Click into this terminal before pressing keys. Each key press holds control
for `teleop_timeout` (5 s of sim time, longer in real time when Gazebo runs
slow), then the robot goes back to driving itself. Run only one teleop at a
time.

If the robot ignores the keyboard, check that key presses are reaching the
controller (terminal 3):

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
ros2 topic echo /teleop_cmd geometry_msgs/msg/TwistStamped
```

Give the type explicitly; without it, `echo` fails with "Could not determine
the type" if the teleop node is not up yet. Press a key in the teleop
terminal. A message should appear here and the controller should log
`Behavior: teleop`. If nothing appears, the problem is on the keyboard side
(wrong terminal focused, missing remap, missing `export` lines, or teleop not
running), not in the controller.

## Viewing the map

Open RViz (terminal 4):

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
rviz2
```

Then, in RViz:

1. In the **Displays** panel, set **Global Options → Fixed Frame** to `odom`.
2. Click **Add → By topic → /map → Map → OK**.
3. If the map doesn't appear, expand the **Map** display and set
   **Topic → Durability Policy** to `Transient Local`.
4. Optional: **Add → By topic** again for **/scan → LaserScan** (lidar) and
   **/robot_description → RobotModel** (the robot).

Gray is unknown, white is free and black is a wall. Let the robot wander, or
drive it with teleop, until the room and hallway are filled in.

If `/map` isn't in the **By topic** list, that terminal is missing the
`export` lines from [Terminal setup](#terminal-setup-every-terminal).

Save the map (writes `project1_map.pgm` and `project1_map.yaml`):

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
cd ~/IRP1
ros2 run nav2_map_server map_saver_cli -f project1_map --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0

```

If the launch output shows "Robot odom pose ... is outside the map grid",
odometry has drifted (usually from wheels slipping against a wall) and scans
are being dropped. Steer the robot free or relaunch.

## Stopping

Ctrl+C the launch, then make sure no Gazebo server is left running. A
leftover server slows the next sim down badly and the map stops updating:

```bash
pkill -f "gz sim"
pgrep -af "gz sim"    # should print nothing
```

## Troubleshooting

If `ros2 topic info /cmd_vel` shows `geometry_msgs/msg/Twist`, relaunch with
`stamped:=false`, drop `-p stamped:=true` from the teleop command, and echo
`/teleop_cmd` as `geometry_msgs/msg/Twist`.
