"""
Reactive (subsumption-style) controller for a TurtleBot 4 (real or sim).

Behaviors, highest priority first. Each behavior is its own method that
returns a velocity command when it wants control this cycle, or ``None`` to
let a lower-priority behavior act (the first non-``None`` command wins):

1. halt        -- stop while any bumper is pressed (and, by default, on a
                  cliff or wheel drop).
2. teleop      -- keyboard commands from ``/teleop_cmd``. The keyboard node
                  publishes once per key press, so the last command is held
                  for ``teleop_timeout`` seconds before autonomy resumes.
3. escape      -- (roughly) symmetric obstacles within 1 ft in front: turn to
                  face away, 180 +/- 30 deg. Fixed action pattern: once
                  started it finishes even if the obstacle disappears.
4. avoid       -- asymmetric obstacles within 1 ft in front: turn away from
                  the closer side. Reflex: only while the obstacle is there.
5. random_turn -- after every 1 ft of forward travel, turn by an angle drawn
                  uniformly from [-15, +15] deg.
6. forward     -- drive straight ahead.
"""

import math
import random

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import HazardDetection
from irobot_create_msgs.msg import HazardDetectionVector
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

FOOT = 0.3048  # m; the brief is written in feet, ROS uses SI units.


def normalize_angle(angle):
    """Wrap an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    """Return the yaw (rotation about z) of a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def lookup_sensor_transform(tf_buffer, base_frame, sensor_frame):
    """
    Look up ``base_frame <- sensor_frame``, or None if not available.

    Gazebo stamps scans with its scoped sensor name, e.g.
    ``turtlebot4/rplidar_link/rplidar``. If that frame is not in TF, fall
    back to the link part (``rplidar_link``), which robot_state_publisher
    always provides.
    """
    candidates = [sensor_frame]
    parts = sensor_frame.split('/')
    if len(parts) >= 2:
        candidates.append(parts[-2])
    for frame in candidates:
        try:
            return tf_buffer.lookup_transform(base_frame, frame, Time())
        except TransformException:
            continue
    return None


class SubsumptionController(Node):
    """Priority-arbitrated reactive controller (see module docstring)."""

    def __init__(self):
        """Declare parameters, then create publishers, subscribers, timer."""
        super().__init__('subsumption_controller')

        # Jazzy TurtleBot 4 takes TwistStamped on /cmd_vel; set stamped:=false
        # if `ros2 topic info /cmd_vel` shows geometry_msgs/msg/Twist.
        self.stamped = self.declare_parameter('stamped', True).value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_link').value
        self.forward_speed = self.declare_parameter(
            'forward_speed', 0.2).value                          # m/s
        self.turn_speed = self.declare_parameter(
            'turn_speed', 0.8).value                             # rad/s
        self.obstacle_distance = self.declare_parameter(
            'obstacle_distance', 1.0 * FOOT).value               # m
        self.front_half_angle = math.radians(self.declare_parameter(
            'front_half_angle_deg', 30.0).value)
        self.symmetry_tolerance = self.declare_parameter(
            'symmetry_tolerance', 0.08).value                    # m
        self.escape_angle = math.radians(self.declare_parameter(
            'escape_angle_deg', 180.0).value)
        self.escape_spread = math.radians(self.declare_parameter(
            'escape_spread_deg', 30.0).value)
        self.random_turn_interval = self.declare_parameter(
            'random_turn_interval', 1.0 * FOOT).value            # m
        self.random_turn_max = math.radians(self.declare_parameter(
            'random_turn_max_deg', 15.0).value)
        self.heading_tolerance = math.radians(self.declare_parameter(
            'heading_tolerance_deg', 3.0).value)
        # Readings closer than this are ignored (on top of the driver's own
        # range_min), so parts of the robot itself never count as obstacles.
        self.min_valid_range = self.declare_parameter(
            'min_valid_range', 0.0).value                        # m
        # The halt reflex stays on only while hazard messages keep arriving;
        # without this a dropped "released" message would halt us forever.
        self.hazard_timeout = self.declare_parameter(
            'hazard_timeout', 0.5).value                         # s
        self.halt_on_cliff = self.declare_parameter(
            'halt_on_cliff', True).value
        # teleop_twist_keyboard publishes once per key press, so keep the
        # last key in charge this long before handing back to autonomy.
        self.teleop_timeout = self.declare_parameter(
            'teleop_timeout', 5.0).value                         # s
        rate = self.declare_parameter('control_rate_hz', 10.0).value

        cmd_type = TwistStamped if self.stamped else Twist
        self.cmd_pub = self.create_publisher(cmd_type, '/cmd_vel', 10)

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        # Sensor-data QoS (best effort) matches either a reliable or a
        # best-effort /odom publisher; a reliable subscriber would get
        # nothing from the real Create 3 if it publishes best effort.
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            HazardDetectionVector, '/hazard_detection',
            self.hazard_callback, qos_profile_sensor_data)
        self.create_subscription(
            cmd_type, '/teleop_cmd', self.teleop_callback, 10)

        # Used to find where the lidar points relative to the robot base.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Sensor state.
        self.halt_hazard = False
        self.hazard_time = None
        self.scan_ready = False
        self.front_left_dist = math.inf
        self.front_right_dist = math.inf
        self.scan_yaw_offset = None

        # Odometry state.
        self.current_yaw = None
        self.last_x = None
        self.last_y = None
        self.forward_travel = 0.0

        # Teleop state.
        self.teleop_twist = None
        self.teleop_time = None

        # Latched turn used by escape (fixed action pattern) and random turn.
        self.turn_target = None
        self.turn_owner = None

        # Highest priority first; see module docstring.
        self.behaviors = (
            ('halt', self.halt),
            ('teleop', self.teleop),
            ('escape', self.escape),
            ('avoid', self.avoid),
            ('random_turn', self.random_turn),
            ('forward', self.forward),
        )
        self.active_behavior = None

        self.create_timer(1.0 / rate, self.control_loop)

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def hazard_callback(self, msg):
        """Record whether any halting hazard is currently detected."""
        halt_types = {HazardDetection.BUMP}
        if self.halt_on_cliff:
            halt_types |= {HazardDetection.CLIFF, HazardDetection.WHEEL_DROP}
        self.halt_hazard = any(d.type in halt_types for d in msg.detections)
        self.hazard_time = self.get_clock().now()

    def teleop_callback(self, msg):
        """Store the latest keyboard command and when it arrived."""
        self.teleop_twist = msg.twist if self.stamped else msg
        self.teleop_time = self.get_clock().now()

    def odom_callback(self, msg):
        """Track heading and distance travelled forward (for random turn)."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.last_x is not None:
            # Only motion along the heading counts; spinning or backing up
            # does not add to the 1 ft counter.
            forward = ((x - self.last_x) * math.cos(self.current_yaw)
                       + (y - self.last_y) * math.sin(self.current_yaw))
            if forward > 0.0:
                self.forward_travel += forward
        self.last_x = x
        self.last_y = y

    def scan_callback(self, msg):
        """Find the closest obstacle in the front-left and front-right arcs."""
        offset = self.lidar_yaw_offset(msg.header.frame_id)
        if offset is None:
            return  # Don't know which way the lidar faces yet.
        min_range = max(msg.range_min, self.min_valid_range)
        left = math.inf
        right = math.inf
        for i, r in enumerate(msg.ranges):
            if not (min_range < r < msg.range_max):
                continue
            # Beam angle in the robot's frame (0 = straight ahead, + = left),
            # so this works however the lidar is mounted or configured.
            angle = normalize_angle(
                msg.angle_min + i * msg.angle_increment + offset)
            if 0.0 <= angle <= self.front_half_angle:
                left = min(left, r)
            elif -self.front_half_angle <= angle < 0.0:
                right = min(right, r)
        self.front_left_dist = left
        self.front_right_dist = right
        self.scan_ready = True

    def lidar_yaw_offset(self, lidar_frame):
        """Return the lidar's yaw relative to the base frame, or None."""
        if self.scan_yaw_offset is not None:
            return self.scan_yaw_offset
        if lidar_frame in ('', self.base_frame):
            self.scan_yaw_offset = 0.0
            return 0.0
        tf = lookup_sensor_transform(
            self.tf_buffer, self.base_frame, lidar_frame)
        if tf is None:
            # The real TurtleBot 4's lidar is mounted rotated, so guessing
            # "forward" would swap front and side; wait for TF instead.
            self.get_logger().warn(
                f'No TF {self.base_frame} <- {lidar_frame} yet; '
                'ignoring scans until it arrives.',
                throttle_duration_sec=5.0)
            return None
        self.scan_yaw_offset = yaw_from_quaternion(tf.transform.rotation)
        self.get_logger().info(
            f'Lidar yaw offset: {math.degrees(self.scan_yaw_offset):.1f} deg')
        return self.scan_yaw_offset

    # ------------------------------------------------------------------
    # Arbitration
    # ------------------------------------------------------------------

    def control_loop(self):
        """Publish the command of the highest-priority active behavior."""
        if self.current_yaw is None or not self.scan_ready:
            # Never drive blind: wait for odometry and a usable scan (the
            # lidar motor must be running and its TF known).
            self.get_logger().info(
                'Waiting for /odom and /scan...', throttle_duration_sec=5.0)
            return
        for name, behavior in self.behaviors:
            cmd = behavior()
            if cmd is not None:
                if name != self.active_behavior:
                    self.get_logger().info(f'Behavior: {name}')
                    self.active_behavior = name
                self.cmd_pub.publish(cmd)
                return

    # ------------------------------------------------------------------
    # Behaviors (highest priority first)
    # ------------------------------------------------------------------

    def halt(self):
        """1. Stop while any bumper is pressed (or cliff / wheel drop)."""
        if not self.halt_hazard:
            return None
        age = self.get_clock().now() - self.hazard_time
        if age > Duration(seconds=self.hazard_timeout):
            self.halt_hazard = False  # Hazard messages stopped: released.
            return None
        return self.make_cmd(0.0, 0.0)

    def teleop(self):
        """2. Hold the last keyboard command for ``teleop_timeout`` s."""
        if self.teleop_twist is None:
            return None
        age = self.get_clock().now() - self.teleop_time
        if age > Duration(seconds=self.teleop_timeout):
            return None
        return self.make_cmd(
            self.teleop_twist.linear.x, self.teleop_twist.angular.z)

    def escape(self):
        """
        3. Turn away (180 +/- 30 deg) from symmetric obstacles within 1 ft.

        Fixed action pattern: once triggered, the turn runs to completion
        even if the obstacles are no longer seen.
        """
        if self.turn_owner == 'escape':
            return self.continue_turn()
        left = self.front_left_dist
        right = self.front_right_dist
        if (left < self.obstacle_distance and right < self.obstacle_distance
                and abs(left - right) < self.symmetry_tolerance):
            angle = self.escape_angle + random.uniform(
                -self.escape_spread, self.escape_spread)
            self.start_turn('escape', angle)
            return self.continue_turn()
        return None

    def avoid(self):
        """
        4. Turn away from the closer side of an asymmetric obstacle.

        Reflex: acts only while an obstacle is within 1 ft in front.
        """
        left = self.front_left_dist
        right = self.front_right_dist
        if left < self.obstacle_distance or right < self.obstacle_distance:
            if self.turn_owner == 'random_turn':
                # A pending random turn is stale once we have had to avoid.
                self.turn_owner = None
                self.turn_target = None
            # Closer on the left -> turn right (negative z), and vice versa.
            angular = -self.turn_speed if left < right else self.turn_speed
            return self.make_cmd(0.0, angular)
        return None

    def random_turn(self):
        """5. After every 1 ft of forward travel, turn U(-15, +15) deg."""
        if self.turn_owner == 'random_turn':
            return self.continue_turn()
        if self.forward_travel >= self.random_turn_interval:
            self.forward_travel = 0.0
            self.start_turn('random_turn', random.uniform(
                -self.random_turn_max, self.random_turn_max))
            return self.continue_turn()
        return None

    def forward(self):
        """6. Drive straight ahead."""
        return self.make_cmd(self.forward_speed, 0.0)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def start_turn(self, owner, angle):
        """Latch an in-place turn of ``angle`` radians from current heading."""
        self.turn_owner = owner
        self.turn_target = normalize_angle(self.current_yaw + angle)

    def continue_turn(self):
        """Rotate toward the latched target; return None once reached."""
        error = normalize_angle(self.turn_target - self.current_yaw)
        if abs(error) <= self.heading_tolerance:
            self.turn_owner = None
            self.turn_target = None
            return None
        return self.make_cmd(0.0, math.copysign(self.turn_speed, error))

    def make_cmd(self, linear, angular):
        """Build a Twist or TwistStamped velocity command."""
        if self.stamped:
            cmd = TwistStamped()
            # Leave header.stamp at zero so the receiver uses its own clock.
            # In sim our clock can lag Gazebo's; on the real robot the
            # desktop's and the Pi's clocks can disagree. Either way a real
            # stamp can look stale and get the command rejected.
            cmd.header.frame_id = self.base_frame
            twist = cmd.twist
        else:
            cmd = Twist()
            twist = cmd
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        return cmd


def main(args=None):
    """Run the subsumption controller node."""
    rclpy.init(args=args)
    node = SubsumptionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
