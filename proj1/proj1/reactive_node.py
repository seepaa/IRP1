import math
import random
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header


class SubsumptionController(Node):
    def __init__(self):
        super().__init__('subsumption_controller')

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/teleop_cmd', self.teleop_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # Robot Pose (Odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_yaw = 0.0

        # Laser Scan Data
        self.front_left_dist = 10.0
        self.front_right_dist = 10.0
        self.bumper_hit = False

        # Behavior State Tracking
        self.teleop_cmd = None
        self.teleop_timer = 0
        
        # Priority 3: Escape (Fixed Action Pattern)
        self.is_escaping = False
        self.escape_target_yaw = 0.0

        # Priority 5: Wander Distance Tracking
        self.last_x = 0.0
        self.last_y = 0.0
        self.distance_traveled = 0.0

        # Occupancy Grid Parameters (10m x 10m grid, 0.05m resolution)
        self.map_res = 0.05
        self.map_width = 200
        self.map_height = 200
        self.map_origin_x = -5.0
        self.map_origin_y = -5.0
        self.grid_data = [-1] * (self.map_width * self.map_height)  # -1 = unknown

        # Main Loop Timer (10 Hz)
        self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg: LaserScan):
        num_samples = len(msg.ranges)
        if num_samples == 0:
            return

        mid = num_samples // 2
        span = int(num_samples * (30 / 360.0))  # 30-degree forward arc

        left_sector = [r for r in msg.ranges[mid:mid+span] if msg.range_min < r < msg.range_max]
        right_sector = [r for r in msg.ranges[mid-span:mid] if msg.range_min < r < msg.range_max]

        self.front_left_dist = min(left_sector) if left_sector else 10.0
        self.front_right_dist = min(right_sector) if right_sector else 10.0

        # Priority 1 Trigger: Very close physical contact (< 0.16m)
        min_overall = min([r for r in msg.ranges if msg.range_min < r < msg.range_max] or [10.0])
        self.bumper_hit = min_overall < 0.16

        # Perform Mapping update using latest scan
        self.update_occupancy_grid(msg)

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw angle from orientation quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Track accumulated travel distance for Priority 5 (Wander)
        step_dist = math.hypot(self.robot_x - self.last_x, self.robot_y - self.last_y)
        self.distance_traveled += step_dist
        self.last_x = self.robot_x
        self.last_y = self.robot_y

    def teleop_callback(self, msg: Twist):
        self.teleop_cmd = msg
        self.teleop_timer = 10  # Active for 1 second (10 x 0.1s cycles)

    def update_occupancy_grid(self, scan: LaserScan):
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                hit_x = self.robot_x + r * math.cos(self.current_yaw + angle)
                hit_y = self.robot_y + r * math.sin(self.current_yaw + angle)

                gx = int((hit_x - self.map_origin_x) / self.map_res)
                gy = int((hit_y - self.map_origin_y) / self.map_res)

                if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                    idx = gy * self.map_width + gx
                    self.grid_data[idx] = 100

            angle += scan.angle_increment

        grid_msg = OccupancyGrid()
        grid_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='odom')
        grid_msg.info.resolution = self.map_res
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.data = self.grid_data
        self.map_pub.publish(grid_msg)

    def control_loop(self):
        cmd = Twist()
        dist_threshold = 0.305  # 1 foot (~0.305 meters)

        # Priority 1: Halt on Collision (Bumper)
        if self.bumper_hit:
            self.get_logger().info('P1: Bumper/Collision detected! Halting.', throttle_duration_sec=1.0)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # Priority 2: Manual Keyboard Commands
        if self.teleop_timer > 0 and self.teleop_cmd is not None:
            self.get_logger().info('P2: User Teleop active.', throttle_duration_sec=1.0)
            self.teleop_timer -= 1
            self.cmd_pub.publish(self.teleop_cmd)
            return

        # Priority 3: Escape (Symmetric obstacles within 1ft)
        is_symmetric = abs(self.front_left_dist - self.front_right_dist) < 0.08

        if self.is_escaping:
            angle_diff = abs(self.current_yaw - self.escape_target_yaw)
            if angle_diff > math.radians(15):
                cmd.angular.z = 0.5
                self.cmd_pub.publish(cmd)
                return
            else:
                self.is_escaping = False

        elif (self.front_left_dist < dist_threshold and 
              self.front_right_dist < dist_threshold and is_symmetric):
            self.get_logger().info('P3: Symmetric obstacle within 1ft. Escaping...')
            self.is_escaping = True
            turn_rad = math.radians(180 + random.uniform(-30, 30))
            self.escape_target_yaw = (self.current_yaw + turn_rad) % (2 * math.pi)
            cmd.angular.z = 0.5
            self.cmd_pub.publish(cmd)
            return

        # Priority 4: Avoid (Asymmetric obstacles within 1ft)
        if self.front_left_dist < dist_threshold or self.front_right_dist < dist_threshold:
            self.get_logger().info('P4: Asymmetric obstacle within 1ft. Avoiding...')
            cmd.linear.x = 0.05
            if self.front_left_dist < self.front_right_dist:
                cmd.angular.z = -0.4
            else:
                cmd.angular.z = 0.4
            self.cmd_pub.publish(cmd)
            return

        # Priority 5: Random Turn (+/- 15 deg) per 1 ft traveled
        if self.distance_traveled >= dist_threshold:
            self.get_logger().info('P5: 1ft Traveled. Applying random heading jitter.')
            self.distance_traveled = 0.0
            cmd.linear.x = 0.15
            cmd.angular.z = math.radians(random.uniform(-15, 15))
            self.cmd_pub.publish(cmd)
            return

        # Priority 6: Drive Forward (Default)
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SubsumptionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
