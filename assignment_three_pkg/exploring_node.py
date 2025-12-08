#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')

        # Parameters
        self.forward_speed = 0.1   # m/s
        self.turn_speed = 0.3      # rad/s
        self.safe_dist = 0.45      # m, stop if obstacle closer than this

        # ROS I/O
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Simple Explorer started: forward or turn, never both.")

    def scan_callback(self, scan: LaserScan):
        # Replace inf/NaN with max range
        max_r = scan.range_max if scan.range_max > 0 else 4.0
        arr = np.array([
            r if (not math.isnan(r) and not math.isinf(r)) else max_r
            for r in scan.ranges
        ])

        # Front sector (-30° to +30°)
        front_angle = math.radians(30)
        inc = scan.angle_increment
        n = len(arr)
        i1 = int((-front_angle - scan.angle_min) / inc)
        i2 = int((front_angle - scan.angle_min) / inc)

        # Clamp indices to valid range
        i1 = max(0, min(n - 1, i1))
        i2 = max(0, min(n - 1, i2))

        # Ensure i1 <= i2
        if i2 < i1:
            i1, i2 = i2, i1

        # Safe min calculation
        sector = arr[i1:i2+1]
        if len(sector) == 0:
            front_dist = max_r
        else:
            front_dist = float(np.min(sector))

        cmd = Twist()

        if front_dist < self.safe_dist:
            # Obstacle too close → turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -self.turn_speed
            self.get_logger().info(f"Obstacle detected at {front_dist:.2f} m → turning right")
        else:
            # Path clear → go forward
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
