#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

DESIRED_DISTANCE = 0.5  # meters
SPEED = 0.15            # max linear speed
ANGULAR_GAIN = 0.1      # for small steering corrections

class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        namespace = self.get_namespace()
        cmd_topic = f'{namespace}/cmd_vel' if namespace != '/' else '/cmd_vel'
        scan_topic = f'{namespace}/scan' if namespace != '/' else '/scan'

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.latest_dist = None
        self.latest_angle = 0.0

        self.get_logger().info(f"Follower controller started in namespace: {namespace}")

    def scan_callback(self, msg: LaserScan):
        ranges = np.nan_to_num(np.array(msg.ranges), nan=msg.range_max, posinf=msg.range_max)
        n = len(ranges)

        mid = n // 2
        window = int(n * 20 / 360)  # ±20° forward window
        start = max(0, mid - window)
        end = min(n, mid + window)

        front_ranges = ranges[start:end]
        front_angles = msg.angle_min + np.arange(n)[start:end] * msg.angle_increment

        valid_mask = (front_ranges > 0.05) & (front_ranges < 2.0)
        if not np.any(valid_mask):
            self.latest_dist = None
            return

        front_ranges = front_ranges[valid_mask]
        front_angles = front_angles[valid_mask]

        closest_index = int(np.argmin(front_ranges))
        self.latest_dist = float(front_ranges[closest_index])
        self.latest_angle = float(front_angles[closest_index])

        cmd = Twist()
        if self.latest_dist is None:
            # No leader detected — stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Linear proportional control
            dist_error = self.latest_dist - DESIRED_DISTANCE
            cmd.linear.x = np.clip(dist_error * 0.8, -SPEED, SPEED)

            # Small angular correction if the leader drifts left/right
            cmd.angular.z = -ANGULAR_GAIN * self.latest_angle

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
