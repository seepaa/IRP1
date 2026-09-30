"""
Occupancy-grid mapper for a simulated TurtleBot 4.

Builds a 2-D ``nav_msgs/OccupancyGrid`` from lidar scans and odometry and
publishes it on ``/map``. Mapping only: the robot does not navigate with it.

Each beam marks the cells it passes through as more likely free and the cell
it ends in as more likely occupied (log-odds update). Cells never seen by any
beam stay unknown (-1).
"""

import math

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

L_OCC = 0.85    # log-odds added to a cell a beam ends in
L_FREE = -0.4   # log-odds added to a cell a beam passes through
L_CLAMP = 5.0   # keep cells able to change their mind


def yaw_from_quaternion(q):
    """Return the yaw (rotation about z) of a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OccupancyMapper(Node):
    """Accumulate lidar scans into a log-odds grid and publish it."""

    def __init__(self):
        """Declare parameters, allocate the grid, create pubs and subs."""
        super().__init__('occupancy_mapper')

        self.base_frame = self.declare_parameter(
            'base_frame', 'base_link').value
        self.resolution = self.declare_parameter(
            'resolution', 0.05).value                            # m/cell
        # Default 20 m x 20 m centred on the start pose: several times the
        # project world, so a larger TA-modified world still fits.
        size_x = self.declare_parameter('size_x', 20.0).value     # m
        size_y = self.declare_parameter('size_y', 20.0).value     # m
        self.origin_x = self.declare_parameter('origin_x', -10.0).value
        self.origin_y = self.declare_parameter('origin_y', -10.0).value
        publish_rate = self.declare_parameter('publish_rate_hz', 1.0).value

        self.width = int(round(size_x / self.resolution))
        self.height = int(round(size_y / self.resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.seen = np.zeros((self.height, self.width), dtype=bool)

        self.map_frame = 'odom'
        self.robot_pose = None          # (x, y, yaw) in map_frame
        self.lidar_mount = None         # (x, y, yaw) in base_frame

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / publish_rate, self.publish_map)

    def odom_callback(self, msg):
        """Store the latest robot pose; the map uses odom's frame."""
        self.map_frame = msg.header.frame_id or 'odom'
        p = msg.pose.pose
        self.robot_pose = (
            p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def lookup_lidar_mount(self, lidar_frame):
        """Return the lidar pose in the base frame (cached once found)."""
        if self.lidar_mount is not None:
            return self.lidar_mount
        if lidar_frame in ('', self.base_frame):
            self.lidar_mount = (0.0, 0.0, 0.0)
            return self.lidar_mount
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, lidar_frame, Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'No TF {self.base_frame} <- {lidar_frame} yet ({ex}); '
                'skipping scan.', throttle_duration_sec=5.0)
            return None
        t = tf.transform
        self.lidar_mount = (
            t.translation.x, t.translation.y,
            yaw_from_quaternion(t.rotation))
        return self.lidar_mount

    def scan_callback(self, scan):
        """Integrate one scan into the grid."""
        if self.robot_pose is None:
            return
        mount = self.lookup_lidar_mount(scan.header.frame_id)
        if mount is None:
            return

        # Lidar pose in the map frame = robot pose composed with the mount.
        rx, ry, ryaw = self.robot_pose
        mx, my, myaw = mount
        sx = rx + mx * math.cos(ryaw) - my * math.sin(ryaw)
        sy = ry + mx * math.sin(ryaw) + my * math.cos(ryaw)
        syaw = ryaw + myaw

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = (scan.angle_min + syaw
                  + np.arange(ranges.size) * scan.angle_increment)
        valid = np.isfinite(ranges) & (ranges > scan.range_min)
        hit = valid & (ranges < scan.range_max)
        # Beams with no return clear space out to the maximum range.
        free_len = np.where(hit, ranges, scan.range_max)
        free_len[~(valid | np.isinf(ranges))] = 0.0

        cos_a = np.cos(angles)
        sin_a = np.sin(angles)

        # Sample each beam every half cell up to its end (exclusive).
        steps = np.arange(0.0, scan.range_max, self.resolution / 2.0)
        along = steps[None, :] < (free_len[:, None] - self.resolution / 2.0)
        fx = sx + steps[None, :] * cos_a[:, None]
        fy = sy + steps[None, :] * sin_a[:, None]
        free_cells = self.to_cells(fx[along], fy[along])

        hx = sx + ranges[hit] * cos_a[hit]
        hy = sy + ranges[hit] * sin_a[hit]
        occ_cells = self.to_cells(hx, hy)

        # Count each cell at most once per scan; hits win over misses.
        free_cells = np.setdiff1d(free_cells, occ_cells)
        flat = self.log_odds.reshape(-1)
        flat[free_cells] += L_FREE
        flat[occ_cells] += L_OCC
        np.clip(self.log_odds, -L_CLAMP, L_CLAMP, out=self.log_odds)
        seen = self.seen.reshape(-1)
        seen[free_cells] = True
        seen[occ_cells] = True

    def to_cells(self, xs, ys):
        """Convert map-frame points to unique flat indices inside the grid."""
        col = np.floor((xs - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((ys - self.origin_y) / self.resolution).astype(np.int64)
        inside = ((col >= 0) & (col < self.width)
                  & (row >= 0) & (row < self.height))
        return np.unique(row[inside] * self.width + col[inside])

    def publish_map(self):
        """Publish the grid: -1 unknown, 0..100 occupancy probability."""
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        data = np.where(self.seen, np.rint(prob * 100.0), -1).astype(np.int8)

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = float(self.resolution)
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = float(self.origin_x)
        grid.info.origin.position.y = float(self.origin_y)
        grid.info.origin.orientation.w = 1.0
        grid.data = data.reshape(-1).tolist()
        self.map_pub.publish(grid)


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
