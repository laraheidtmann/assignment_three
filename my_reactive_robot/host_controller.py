import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np


DESIRED_DISTANCE_FROM_WALL = 0.3
LINEAR_SPEED = 0.12
FRONT_OBSTACLE_THRESHOLD = 0.35

KP = 3.5
KI = 0.02
KD = 0.8


class HostController(Node):
    def __init__(self):
        super().__init__('host_controller')

        namespace = self.get_namespace()
        cmd_topic = f'{namespace}/cmd_vel' if namespace != '/' else '/cmd_vel'
        scan_topic = f'{namespace}/scan' if namespace != '/' else '/scan'

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.integral = 0.0
        self.prev_error = 0.0

        self.get_logger().info("✅ Host controller started")

    def get_distance(self, scan, angle_deg):
        """Get distance at a given angle (in degrees) from LaserScan data."""
        ranges = np.array(scan.ranges, dtype=float)
        ranges = np.nan_to_num(ranges, nan=scan.range_max, posinf=scan.range_max, neginf=scan.range_max)
        
        def deg_to_index():
            num_points = len(ranges)
            angle = math.radians(angle_deg)

            # handle reversed scan
            if scan.angle_min > scan.angle_max:
                angle_min = scan.angle_max
                angle_max = scan.angle_min
                angle = -angle  # flip the angle as well
            else:
                angle_min = scan.angle_min
                angle_max = scan.angle_max

            angle_increment = (angle_max - angle_min) / (num_points - 1)
            idx = int((angle - angle_min) / angle_increment)
            idx = max(0, min(num_points - 1, idx))
            return idx
        
        def safe_mean(center, window=3):
            n = len(ranges)
            s = max(center - window, 0)
            e = min(center + window, n - 1)
            vals = np.array(ranges[s:e], dtype=float)
            vals = np.nan_to_num(vals, nan=np.inf, posinf=np.inf)
            return float(np.min(vals))
        
        index = deg_to_index()
        distance = safe_mean(index)
        return distance

    def scan_callback(self, msg: LaserScan):
        d90 = self.get_distance(msg, -90)
        d45 = self.get_distance(msg, -45)
        d0 = self.get_distance(msg, 0)
        
        # Obstacle avoidance first
        if d0 < FRONT_OBSTACLE_THRESHOLD:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0  # Turn left
            self.cmd_pub.publish(cmd)
            return
        
        # Compute wall angle and distance
        alpha = math.atan2(d90 * math.cos(math.radians(45)) - d45,
                            d90 * math.sin(math.radians(45)))
        dist = d45 * math.cos(alpha)
        
        # Control error
        error = DESIRED_DISTANCE_FROM_WALL - dist

        # PID controller
        self.integral += error
        derivative = error - self.prev_error
        omega = KP * error + KI * self.integral + KD * derivative
        self.prev_error = error
        
        # Compose Twist command
        cmd = Twist()
        cmd.linear.x = LINEAR_SPEED
        cmd.angular.z = float(np.clip(omega, -3.0, 3.0))
        self.cmd_pub.publish(cmd)
        
        #self.get_logger().info(f"d90={d90:.2f} d45={d45:.2f} α={math.degrees(alpha):.1f}° e={error:.3f} ω={omega:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = HostController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()