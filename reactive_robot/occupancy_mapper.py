"""
Occupancy-grid mapper for the TurtleBot 4.

Builds a 2-D ``nav_msgs/OccupancyGrid`` from lidar scans and the robot pose
(odometry / tf). Mapping only: no localization correction, no navigation.

Topics, frames and QoS are specified in INTERFACES.md.
"""

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer
from tf2_ros import TransformListener


class OccupancyMapper(Node):
    """Accumulate lidar scans into an occupancy grid and publish it."""

    def __init__(self):
        """Declare parameters and create the publisher, subscriptions, tf."""
        super().__init__('occupancy_mapper')

        # Defaults cover the whole 15 ft x 20 ft world, with margin, as seen
        # from odom (odom origin = spawn pose; default -0.762, 0.762, yaw 0).
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('resolution', 0.05)      # m/cell
        self.declare_parameter('width', 8.0)            # m
        self.declare_parameter('height', 8.0)           # m
        self.declare_parameter('origin_x', -4.0)        # m, in map_frame
        self.declare_parameter('origin_y', -4.0)        # m, in map_frame
        self.declare_parameter('publish_rate_hz', 1.0)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_map)

        # TODO(mapping owner): allocate the grid storage (e.g. log-odds array
        #   sized from width/height/resolution).

    def _on_scan(self, msg):
        """Integrate one lidar scan into the grid."""
        # TODO(mapping owner): look up map_frame <- msg.header.frame_id at
        #   msg.header.stamp via self._tf_buffer, then call _integrate_scan.

    def _on_odom(self, msg):
        """Cache the latest odometry (fallback / sanity check for tf)."""
        # TODO(mapping owner): store pose if needed.

    def _integrate_scan(self, scan, sensor_pose):
        """
        Update cell occupancy along each beam of ``scan``.

        :param scan: the ``LaserScan`` to integrate.
        :param sensor_pose: lidar pose (x, y, yaw) in ``map_frame``.
        """
        # TODO(mapping owner): ray-trace each valid beam (e.g. Bresenham);
        #   mark traversed cells free and the end cell occupied (log-odds).

    def _world_to_cell(self, x, y):
        """
        Convert a point in ``map_frame`` to grid indices.

        :return: ``(col, row)``, or ``None`` if outside the grid.
        """
        # TODO(mapping owner): implement.
        return None

    def _publish_map(self):
        """Publish the current grid as a ``nav_msgs/OccupancyGrid``."""
        # TODO(mapping owner): convert to int8 [-1 unknown, 0..100], fill
        #   header (map_frame, now) and info (resolution, size, origin).


def main(args=None):
    """Run the occupancy mapper node."""
    rclpy.init(args=args)
    node = OccupancyMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
