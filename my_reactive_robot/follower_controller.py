#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import numpy as np

DESIRED_DISTANCE = 0.3  # meters
SPEED = 0.15            # max linear speed
ANGULAR_GAIN = 0.1      # for small steering corrections
LINEAR_GAIN=0.6



class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        namespace = self.get_namespace()
        cmd_topic = f'{namespace}/cmd_vel' if namespace != '/' else '/cmd_vel'
        scan_topic = f'{namespace}/scan' if namespace != '/' else '/scan'

        self.leader_north_sub = self.create_subscription(Vector3, '/leader/leader/compass/north_vector', self.leader_north_cb, 10)
        self.follower_north_sub = self.create_subscription(Vector3, '/follower/follower/compass/north_vector', self.follower_north_cb, 10)

        self.leader_north=None
        self.follower_north=None
        
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback_1, 10)

        self.latest_dist = None
        self.latest_angle = 0.0

        self.get_logger().info(f"Follower controller started in namespace: {namespace}")


    def leader_north_cb(self, msg): 
        self.leader_north = np.array([msg.x, msg.y, msg.z]) 

    def follower_north_cb(self, msg): 
        self.follower_north = np.array([msg.x, msg.y, msg.z])

    def scan_callback_1(self,msg: LaserScan):
        """Estimate distance to object directly ahead""" 
        ranges = np.nan_to_num(np.array(msg.ranges), nan=msg.range_max, posinf=msg.range_max) 
        n = len(ranges) 
        mid = n // 2 
        window = 10 # samples around center 
        front_ranges = ranges[mid - window:mid + window] 
        self.latest_dist = float(np.min(front_ranges))


        if self.latest_dist is None or self.follower_north is None or self.leader_north is None: 
            self.get_logger().info("nothing came through unfortunately")
            return 
        
        # Compute heading error using compass north vectors (2D cross/dot) 

        f = self.follower_north[:2] / np.linalg.norm(self.follower_north[:2]) 
        l = self.leader_north[:2] / np.linalg.norm(self.leader_north[:2]) 

        cross = np.cross(np.append(f, 0), np.append(l, 0))[2] # z-component of cross product 
        dot = np.clip(np.dot(f, l), -1.0, 1.0)

        heading_error = np.arctan2(cross, dot) # Linear velocity based on distance 
        dist_error = self.latest_dist - DESIRED_DISTANCE 
        linear_speed = np.clip(LINEAR_GAIN * dist_error, -0.15, 0.15) # Angular velocity based on heading error 
        angular_speed = np.clip(ANGULAR_GAIN * heading_error, -1.0, 1.0) 

        cmd = Twist() 
        cmd.linear.x = linear_speed 
        cmd.angular.z = angular_speed 

        self.cmd_pub.publish(cmd) 

        self.get_logger().info( f"dist={self.latest_dist:.2f} m, heading_err={np.degrees(heading_error):.1f}°, " f"cmd=({linear_speed:.2f}, {angular_speed:.2f})" )


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
