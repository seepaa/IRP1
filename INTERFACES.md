# Interfaces

The contract every node in `reactive_robot` codes against. If you change a
topic, type, frame or QoS here, update the nodes in the same PR.

Target: ROS 2 Jazzy, Gazebo Harmonic, TurtleBot 4 (`turtlebot4_gz_bringup`),
no namespace. All nodes run with `use_sim_time:=true`.

## Topics

| Topic | Type | Publisher | Subscribers | QoS | Frame |
|---|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `reactive_controller` | TB4 (Create 3 diff drive) | reliable, depth 10 | `base_link` |
| `/teleop/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `teleop_twist_keyboard` (remapped) | `reactive_controller` | reliable, depth 10 | `base_link` |
| `/hazard_detection` | `irobot_create_msgs/msg/HazardDetectionVector` | TB4 (Create 3) | `reactive_controller` | sensor data (best effort) | `base_link` |
| `/scan` | `sensor_msgs/msg/LaserScan` | TB4 lidar (ros_gz_bridge) | `reactive_controller`, `occupancy_mapper` | sensor data (best effort) | lidar frame, from `header.frame_id` |
| `/odom` | `nav_msgs/msg/Odometry` | TB4 (Create 3) | `reactive_controller`, `occupancy_mapper` | reliable, depth 10 | `odom` → `base_link` |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `occupancy_mapper` | RViz, anyone | reliable, **transient local**, depth 1 | `map_frame` param (default `odom`) |
| `/clock` | `rosgraph_msgs/msg/Clock` | `clock_bridge` (sim.launch.py) | all | default | – |

Subscribers use best-effort (`qos_profile_sensor_data`) for sensor topics so
they connect whether the publisher is reliable or best effort.

### Notes

- **`/cmd_vel` is `TwistStamped`, not `Twist`.** The Jazzy TurtleBot 4 is
  expected to take stamped commands. Confirm on first launch with
  `ros2 topic info /cmd_vel`; if it reports `Twist`, change this table and
  the publisher/subscriber types in `reactive_controller.py` together.
- **Bumper:** the TB4 has no plain bumper topic. A bump is any entry in
  `HazardDetectionVector.detections` with
  `type == HazardDetection.BUMP`; the entry's `header.frame_id` says which
  part of the bumper was hit.
- **Teleop:** keyboard commands go to `/teleop/cmd_vel`, never directly to
  `/cmd_vel`, so `reactive_controller` stays the only `/cmd_vel` publisher
  and bumper halt still overrides teleop:

  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -p stamped:=true -p frame_id:=base_link \
    -r cmd_vel:=/teleop/cmd_vel
  ```

## Frames

- `odom` → `base_link`: from the TB4 (Create 3) odometry, on `/tf`.
- `base_link` → lidar frame: from `robot_state_publisher`, on `/tf_static`.
- There is no `map` → `odom` transform (no localization). The mapper
  therefore builds its grid in `odom` by default. If a `map` frame is added
  later, set the mapper's `map_frame` parameter.
- The Gazebo world origin is the centre of the 15 ft × 20 ft outer walls.
  `odom` starts at the spawn pose (default `x=-0.762, y=0.762, yaw=0`, the
  room centre, facing the doorway).

## Units and shared constants

All values are SI (m, rad, s). The spec is in feet: 1 ft = 0.3048 m.

| Constant | Value | Used by |
|---|---|---|
| Obstacle trigger distance | 1 ft = 0.3048 m | escape, avoid |
| Escape turn | 180° ± 30° (π ± 0.524 rad) | escape |
| Random-turn interval | every 1 ft = 0.3048 m of forward travel | random turn |
| Random-turn range | ±15° (±0.262 rad) | random turn |

Walls are 0.15 m thick and centred on the drawing lines, so free space is
about 0.15 m smaller than the nominal room and hallway dimensions.

## Nodes

| Executable | Node name | Owner |
|---|---|---|
| `reactive_controller` | `reactive_controller` | behaviors owner |
| `occupancy_mapper` | `occupancy_mapper` | mapping owner |

## Running

```bash
# in the workspace root, with this repo in src/
colcon build --packages-select reactive_robot && source install/setup.bash
ros2 launch reactive_robot sim.launch.py            # x:= y:= yaw:= to move the start pose
ros2 run reactive_robot reactive_controller --ros-args -p use_sim_time:=true
ros2 run reactive_robot occupancy_mapper --ros-args -p use_sim_time:=true
```
