# Project 2 Change Documentation (draft)

Changes made to the code and launch files between Project 1 (Gazebo
simulation) and Project 2 (physical TurtleBot 4 in REPF B4), and why.

## Launch files

**New `robot.launch.py`.** Project 1's `simulation.launch.py` is kept for
the simulator; Project 2 uses a new launch file instead of editing it.

- *Removed:* Gazebo, the `/clock` bridge, robot spawning, Gazebo resource
  paths and the world / start-pose arguments. The real robot runs its own
  drivers and already publishes `/scan`, `/odom`, `/hazard_detection` and
  `/tf`, so there is nothing to simulate, spawn or bridge.
- *`use_sim_time` is now false.* Without Gazebo nothing publishes `/clock`,
  so with sim time the nodes' clocks would stay at zero. The teleop timeout
  would then never expire, and one key press would keep control forever.
- *Starts the lidar motor* (`/start_motor`) at launch and *stops it*
  (`/stop_motor`) on Ctrl+C. The real RPLIDAR doesn't spin until it is told
  to, and lab rules require it to be off before the robot is docked.
- *Kept the reflex-disable step.* The Create 3 turns its bump reflexes back
  on after every reboot, and they would fight our halt behavior.
- *Mapper is now optional* (`mapper:=false` by default). Project 2 doesn't
  require mapping.
- *New tuning arguments:* `forward_speed`, `turn_speed` and `halt_on_cliff`,
  so the robot can be tuned in the lab without editing code.

## Parameter changes

| Parameter | Project 1 | Project 2 | Reason |
|---|---|---|---|
| `forward_speed` | 0.2 m/s | 0.1 m/s | OU lab guide recommends 0.1; safer in a shared room |
| `turn_speed` | 0.8 rad/s | 0.5 rad/s | Less overshoot on real wheels |
| `heading_tolerance_deg` | 3 | 5 | Real odometry is noisier; avoids hunting around the target heading |
| `min_valid_range` | (new) | 0.15 m | Ignore lidar returns from the robot's own tower posts |

## Controller (`reactive_node.py`)

- *`/odom` subscribed with sensor-data (best-effort) QoS.* A reliable
  subscriber receives nothing from a best-effort publisher. The controller
  would then never get its first pose and would sit idle without any error.
- *No more guessing which way the lidar faces.* Project 1 assumed the lidar
  faced forward when its TF wasn't available. The real TurtleBot 4's lidar is
  mounted rotated, so that guess would swap front and side. Scans are now
  ignored until the TF arrives.
- *The robot waits for both odometry and a usable scan before moving.* If
  the lidar isn't spinning, the robot stays still instead of driving blind.
- *Halt also covers cliff and wheel-drop hazards* (`halt_on_cliff`, on by
  default). On the physical robot, these hazards can damage it.
- *Halt releases if hazard messages stop arriving* (`hazard_timeout`,
  0.5 s). Over Wi-Fi a lost "released" message could otherwise leave the
  robot halted forever.
- *`min_valid_range` parameter* (see table above).

## Mapper (`occupancy_mapper.py`)

- *`/odom` subscribed with sensor-data QoS,* for the same reason as the
  controller.

## Package and documentation

- `package.xml`: added `std_srvs` (lidar start/stop service type). Marked the
  Gazebo packages as needed only for the simulation launch. Updated the
  descriptions in `package.xml` and `setup.py`.
- `README.md`: added the Project 2 procedure (SSH check, `robot-setup.sh`
  discovery-server setup in every desktop terminal, launch, teleop,
  shutdown and docking). The Project 1 `ROS_DOMAIN_ID=42` setup conflicts
  with the robot's network, so it is now only in the simulation section.
