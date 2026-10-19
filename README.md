# Robotics_Project_1

CS 4023/5023 Projects 1 and 2: a TurtleBot 4 (ROS 2 Jazzy) running a
reactive, subsumption-style controller. Project 2 runs it on a **physical**
TurtleBot 4 in REPF B4; Project 1 ran it in Gazebo Harmonic, building an
occupancy-grid map of a room and L-shaped hallway.

Package `proj1`:

| Path | What it is |
|---|---|
| `proj1/launch/robot.launch.py` | **Project 2.** Starts the lidar, disables bump reflexes, runs the controller on the real robot |
| `proj1/launch/simulation.launch.py` | Project 1. Starts Gazebo, spawns the TurtleBot 4, runs both nodes |
| `proj1/worlds/cust_robo_room.sdf` | Project 1 room and hallway (`<world name="project1_world">`) |
| `proj1/proj1/reactive_node.py` | Controller: halt, teleop, escape, avoid, random turn, forward |
| `proj1/proj1/occupancy_mapper.py` | Mapper: publishes `/map` (`nav_msgs/OccupancyGrid`) |

First-time setup on the lab desktop:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/seepaa/IRP1
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select proj1
```

# Project 2: physical TurtleBot 4

Everything below runs on the **lab desktop**, except step 1. The robot's own
drivers already publish `/scan`, `/odom`, `/hazard_detection` and `/tf`, so
no Gazebo is involved.

## 1. Check the robot (SSH terminal)

Carry the robot by its base (never by the tower or sensors) and put it on the
floor. Then:

```bash
ssh student@<robotName>.cs.nor.ou.edu      # password: student
ros2 topic list                            # expect /scan, /tf, /odom
```

If those topics are missing, run `turtlebot4-daemon-restart`, wait for the
startup chime, and check again. As a last resort:
`ros2 launch turtlebot4_bringup robot.launch.py`.

If the robot is on its dock, undock it (from this terminal or a desktop one):

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

## 2. Desktop terminal setup (every desktop terminal)

In **every** desktop terminal, run `robot-setup.sh`, enter the robot's name,
and run the commands it prints, in order (they `unset ROS_LOCALHOST_ONLY`,
set `ROS_DOMAIN_ID`, `ROS_DISCOVERY_SERVER` and `ROS_SUPER_CLIENT`, and
restart the ROS daemon). Then:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list                            # should show the robot's topics
```

Do **not** use the Project 1 `ROS_LOCALHOST_ONLY=0` / `ROS_DOMAIN_ID=42`
exports here: they put the terminal on a different network than the robot.

## 3. Launch (desktop terminal 1)

```bash
ros2 launch proj1 robot.launch.py
```

This starts the lidar motor, turns off the Create 3's built-in bump reflexes
(so our halt behavior is in charge), and starts the controller. The
controller logs `Waiting for /odom and /scan...` until both arrive, then
`Behavior: forward` and the robot starts moving.

Optional arguments: `forward_speed:=0.1` (m/s), `turn_speed:=0.5` (rad/s),
`mapper:=true` (also publish `/map`), `halt_on_cliff:=false`,
`start_lidar:=false`,
`disable_reflexes:=false`, `stamped:=false`.

## 4. Keyboard control (desktop terminal 2)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/teleop_cmd
```

Same as Project 1: keys go to `/teleop_cmd`, each key press holds control for
5 s, and a bump still halts the robot. Lower the teleop speeds (`z`/`c`
keys) to around 0.1 m/s and 0.2 rad/s.

## 5. Stopping

Ctrl+C the launch. It stops the lidar motor on the way out (it prints
`Stopping lidar motor...`). If it warns that this failed, stop it by hand:

```bash
ros2 service call /stop_motor std_srvs/srv/Empty "{}"
```

Then carry the robot back to its charging dock. The lidar must be stopped
before the robot goes on the dock.

## Robot troubleshooting

- **Stuck on `Waiting for /odom and /scan...`:** check `ros2 topic hz /scan`
  and `ros2 topic hz /odom`. No `/scan` means the lidar isn't spinning; no
  data at all means the terminal is missing the `robot-setup.sh` exports.
- **`No TF base_link <- rplidar_link yet`:** the controller ignores scans
  until it knows how the lidar is mounted. Check `ros2 topic echo /tf_static`.
- **Robot backs away by itself after a bump:** reflexes are still on. Check
  `ros2 param get /motion_control reflexes_enabled`.
- **Robot halts at dark floor tape:** the cliff sensors see it as a drop.
  Relaunch with `halt_on_cliff:=false`.

# Project 1: simulation

## Requirements

ROS 2 Jazzy with `turtlebot4_simulator` (for `turtlebot4_gz_bringup`),
`teleop_twist_keyboard` and `nav2_map_server`. Check with:

```bash
ros2 pkg list | grep -E 'turtlebot4_gz_bringup|irobot_create_gz|teleop_twist_keyboard|nav2_map_server'
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
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/teleop_cmd
```

Click into this terminal before pressing keys. Each key press holds control
for `teleop_timeout` (5 s of sim time, longer in real time when Gazebo runs
slow), then the robot goes back to driving itself. Run only one teleop at a
time.

If the robot ignores the keyboard, check that key presses are reaching the
controller (terminal 3):

```bash
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
ros2 run nav2_map_server map_saver_cli -f project1_map
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
