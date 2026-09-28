"""
Reactive controller for the TurtleBot 4.

Behaviors are arbitrated by fixed priority, highest first; the first behavior
that returns a command wins the control cycle and lower ones are suppressed:

1. halt on bumper
2. keyboard teleop
3. escape (symmetric obstacle ahead, fixed action pattern)
4. avoid (asymmetric obstacle ahead, reflexive turn)
5. random turn (every 1 ft travelled)
6. drive forward

Topics and message types are specified in INTERFACES.md.
"""

import math

from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import HazardDetectionVector
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

FOOT = 0.3048  # m; the spec is written in feet, ROS works in SI units.


class ReactiveController(Node):
    """
    Priority-arbitrated reactive controller.

    Each behavior method takes no arguments, reads the latest sensor state
    cached by the callbacks, and returns either a ``TwistStamped`` command
    (it wants control this cycle) or ``None`` (defer to lower priorities).
    """

    def __init__(self):
        """Declare parameters and create the publisher, subscriptions, timer."""
        super().__init__('reactive_controller')

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('forward_speed', 0.2)            # m/s
        self.declare_parameter('turn_speed', 1.0)               # rad/s
        self.declare_parameter('obstacle_distance', 1.0 * FOOT)  # m
        self.declare_parameter('escape_angle', math.pi)          # rad
        self.declare_parameter('escape_tolerance', math.radians(30.0))
        self.declare_parameter('random_turn_interval', 1.0 * FOOT)  # m
        self.declare_parameter('random_turn_max', math.radians(15.0))
        self.declare_parameter('teleop_timeout', 0.5)           # s

        self._cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.create_subscription(
            HazardDetectionVector, '/hazard_detection', self._on_hazard,
            qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, '/teleop/cmd_vel', self._on_teleop, 10)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        # Highest priority first; see module docstring.
        self._behaviors = (
            self._halt_on_bumper,
            self._keyboard_teleop,
            self._escape,
            self._avoid,
            self._random_turn,
            self._drive_forward,
        )

        rate = self.get_parameter('control_rate_hz').value
        self.create_timer(1.0 / rate, self._control_loop)

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _on_hazard(self, msg):
        """Cache bumper state from a ``HazardDetectionVector``."""
        # TODO(behaviors owner): record whether any detection has
        #   type == HazardDetection.BUMP (and optionally which side).

    def _on_teleop(self, msg):
        """Cache the latest keyboard teleop command and its arrival time."""
        # TODO(behaviors owner): store msg and self.get_clock().now() so
        #   _keyboard_teleop can tell whether teleop is still active.

    def _on_scan(self, msg):
        """Cache the latest lidar scan."""
        # TODO(behaviors owner): store msg (or pre-compute left/right/front
        #   minimum ranges within obstacle_distance).

    def _on_odom(self, msg):
        """Cache the latest odometry pose for distance/heading tracking."""
        # TODO(behaviors owner): store pose; used for distance travelled
        #   (random turn) and heading change (escape).

    # ------------------------------------------------------------------
    # Arbitration
    # ------------------------------------------------------------------

    def _control_loop(self):
        """Run the highest-priority active behavior and publish its command."""
        # TODO(behaviors owner): iterate self._behaviors in order; publish
        #   the first non-None command on /cmd_vel (stamp it, frame_id
        #   'base_link') and stop. Consider logging behavior transitions.

    # ------------------------------------------------------------------
    # Behaviors (highest priority first)
    # ------------------------------------------------------------------

    def _halt_on_bumper(self):
        """
        Stop immediately while any bumper is pressed.

        :return: zero-velocity command while a bump is active, else ``None``.
        """
        # TODO(behaviors owner): implement.
        return None

    def _keyboard_teleop(self):
        """
        Pass through keyboard teleop commands while teleop is active.

        Teleop is active if a ``/teleop/cmd_vel`` message arrived within
        ``teleop_timeout`` seconds.

        :return: the latest teleop command while active, else ``None``.
        """
        # TODO(behaviors owner): implement.
        return None

    def _escape(self):
        """
        Turn away from a roughly symmetric obstacle ahead.

        Triggered when obstacles lie within ``obstacle_distance`` on both
        sides of the front of the robot. This is a fixed action pattern: once
        triggered it rotates ``escape_angle`` +/- ``escape_tolerance``
        (180 +/- 30 deg) to completion, regardless of new sensor input.

        :return: rotation command while the pattern runs, else ``None``.
        """
        # TODO(behaviors owner): implement trigger test, pick a random target
        #   heading in [150, 210] deg, and latch until reached (via odom).
        return None

    def _avoid(self):
        """
        Reflexively turn away from an asymmetric obstacle ahead.

        Triggered when an obstacle within ``obstacle_distance`` is closer on
        one side of the front than the other; turns away from the closer
        side for as long as the condition holds (not latched).

        :return: turn command while triggered, else ``None``.
        """
        # TODO(behaviors owner): implement.
        return None

    def _random_turn(self):
        """
        Turn by a random angle in +/- ``random_turn_max`` (15 deg).

        Triggered after every ``random_turn_interval`` (1 ft) of forward
        travel, measured from odometry.

        :return: rotation command while the turn runs, else ``None``.
        """
        # TODO(behaviors owner): implement distance accumulator + turn latch.
        return None

    def _drive_forward(self):
        """
        Drive straight ahead at ``forward_speed``.

        Lowest priority default; always returns a command.

        :return: forward velocity command.
        """
        # TODO(behaviors owner): implement.
        return None


def main(args=None):
    """Run the reactive controller node."""
    rclpy.init(args=args)
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
