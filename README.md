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

# Project 2: physical TurtleBot 4

You need four terminals. Terminal 1 is an SSH session on the robot; Terminals
0, 2 and 3 run on the lab desktop. Replace anything in `<angle brackets>` with
your own values. The robot's own drivers already publish `/scan`, `/odom`,
`/hazard_detection` and `/tf`, so no Gazebo is involved.

| Terminal | Runs on | Purpose |
|---|---|---|
| 0 | Desktop | Download and build the code (once) |
| 1 | Robot (SSH) | Check the robot is up, undock it |
| 2 | Desktop | Launch the project |
| 3 | Desktop | Keyboard control |

> **Watch for the handout typo.** The OU handout prints
> `export ROS_DISCOVERY_SERVER:"..."` with a colon. It must be `=`. With the
> colon, the desktop can't see the robot and nothing below works.

## Terminal 0: build (desktop, once)

If the repo is not on this machine yet:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b project-2 https://github.com/seepaa/IRP1
```

If it is already there from Project 1:

```bash
cd ~/ros2_ws/src/IRP1
git fetch && git checkout project-2 && git pull
```

Then build:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select proj1
```

The build should end with `Finished <<< proj1`. You can close this terminal
afterwards.

## Terminal 1: robot (SSH)

Put the robot on the floor, lifting it by the base only (never by the tower or
sensors). Then log in (password: `student`) and check its topics:

```bash
ssh student@<robotName>.cs.nor.ou.edu
ros2 topic list
```

The list must include `/scan`, `/tf` and `/odom`. If any are missing, restart
the daemon, wait for the startup chime, and check again:

```bash
turtlebot4-daemon-restart
ros2 topic list
```

If they are still missing, start the robot's bringup by hand:

```bash
ros2 launch turtlebot4_bringup robot.launch.py
```

If the robot is on its dock, undock it:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

Leave this terminal open; you may need it to stop the LiDAR at the end.

## Terminal 2: launch the project (desktop)

Run the setup script and enter the robot's name:

```bash
robot-setup.sh
```

It prints a list of commands. Run them in order, using the numbers it printed
(note the `=` after `ROS_DISCOVERY_SERVER`):

```bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=<number it printed>
export ROS_DISCOVERY_SERVER=";;;;;;;;<IP it printed>:11811;"
export ROS_SUPER_CLIENT=True
ros2 daemon stop
ros2 daemon start
```

Then build, load the workspace, and launch:

```bash
cd ~/ros2_ws
ls src/IRP1/proj1/package.xml                # must exist
source /opt/ros/jazzy/setup.bash
colcon build --packages-select proj1         # must end with "Finished <<< proj1"
source install/setup.bash                    # loads your workspace into this terminal
ros2 pkg prefix proj1                        # should print ~/ros2_ws/install/proj1
ros2 launch proj1 robot.launch.py
```

You should see, in order:

1. The LiDAR starts spinning.
2. `Create 3 reflexes disabled`
3. `Behavior: forward`, and the robot starts driving.

Optional settings go on the end of the launch line. Press Ctrl+C to stop
first, then relaunch with them, for example:

```bash
ros2 launch proj1 robot.launch.py forward_speed:=0.08 turn_speed:=0.4
ros2 launch proj1 robot.launch.py halt_on_cliff:=false
```

All options: `forward_speed:=0.1` (m/s), `turn_speed:=0.5` (rad/s),
`mapper:=true` (also publish `/map`), `halt_on_cliff:=false`,
`start_lidar:=false`, `disable_reflexes:=false`, `stamped:=false`.

Do **not** use the Project 1 `ROS_LOCALHOST_ONLY=0` / `ROS_DOMAIN_ID=42`
exports here: they put the terminal on a different network than the robot.

## Terminal 3: keyboard control (desktop)

Open a new terminal. Run the setup script, then the same exports as Terminal 2
(with the numbers it prints), then load ROS and the project:

```bash
robot-setup.sh
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=<number it printed>
export ROS_DISCOVERY_SERVER=";;;;;;;;<IP it printed>:11811;"
export ROS_SUPER_CLIENT=True
ros2 daemon stop
ros2 daemon start
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Start the keyboard controller:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Click into this terminal before pressing keys: `i` forward, `j`/`l` turn, `k`
stop. Each key press takes control for 5 s, then the robot drives itself
again. A bump still halts the robot. Lower the speeds with `z` and `c` to
about 0.1 m/s and 0.2 rad/s.

## Stopping after the demo

1. Press Ctrl+C in Terminal 2. It prints `Stopping lidar motor...`.
2. If the LiDAR is still spinning, stop it from Terminal 1:

```bash
   ros2 service call /stop_motor std_srvs/srv/Empty "{}"
```

3. Carry the robot back to its charging dock. The LiDAR must be stopped first.

## If something goes wrong

1. **`Package 'proj1' not found`** — that terminal hasn't loaded your
   workspace. Run:

```bash
   cd ~/ros2_ws
   source install/setup.bash
```

   If it still fails:
   - *colcon says "Duplicate package names" or builds nothing:* the repo is
     cloned more than once or in the wrong place. Find every copy with
     `find ~ -name package.xml -path "*proj1*" 2>/dev/null`, keep exactly one
     at `~/ros2_ws/src/IRP1/proj1/package.xml`, then run
     `rm -rf build install log` in `~/ros2_ws` and rebuild.
   - *Build works but `robot.launch.py` isn't found:* the clone is on the old
     branch. Run `cd ~/ros2_ws/src/IRP1 && git checkout project-2 && git pull`,
     then rebuild and re-source.
2. **Desktop `ros2 topic list` shows only `/rosout` and `/parameter_events`**
   — that terminal is missing the exports, or has `:` instead of `=`. Redo the
   `robot-setup.sh` exports and the daemon restart, then `ros2 topic list`.
3. **Stuck on `Waiting for /odom and /scan...`** — check
   `ros2 topic hz /scan` on the desktop. No data means the LiDAR isn't
   spinning; start it from Terminal 1:
   `ros2 service call /start_motor std_srvs/srv/Empty "{}"`
4. **`No TF base_link <- rplidar_link yet` keeps repeating** — check the
   robot's transforms with `ros2 topic echo /tf_static --once`.
5. **Logs `Behavior: forward` but the robot doesn't move** — Ctrl+C, then
   `ros2 launch proj1 robot.launch.py stamped:=false`
6. **Robot backs away by itself after a bump** — reflexes are still on:

```bash
   ros2 param get /motion_control reflexes_enabled
   ros2 param set /motion_control reflexes_enabled false
```

7. **Robot stops at dark floor tape** — Ctrl+C, then
   `ros2 launch proj1 robot.launch.py halt_on_cliff:=false`
8. **Too fast or too slow** — Ctrl+C, then
   `ros2 launch proj1 robot.launch.py forward_speed:=0.08 turn_speed:=0.4`

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
