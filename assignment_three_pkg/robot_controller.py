#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose2D
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from std_msgs.msg import Float32


DESIRED_DISTANCE_FROM_WALL = 0.35
LINEAR_SPEED = 0.18
MIN_LINEAR_SPEED = 0.03
FRONT_OBSTACLE_THRESHOLD = 0.35

K_LINEAR = 0.2    # gain for linear speed
K_ANGULAR = 1.5   # gain for angular speed
MAX_LINEAR = 0.15
MAX_ANGULAR = 1.0
GOAL_TOLERANCE = 0.05  # distance to goal to stop

class WallFollower(Node):
    def __init__(self):
        super().__init__('reactive_robot')
        self.current_position = [0.0, 0.0, 0.0]  # x, y, theta
        self.goal_position=[1.0, 1.0]

        namespace = self.get_namespace()
        cmd_topic = f'{namespace}/cmd_vel' if namespace != '/' else '/cmd_vel'
        scan_topic = f'{namespace}/scan' if namespace != '/' else '/scan'
        odom_topic = f'{namespace}/odom' if namespace != '/' else '/odom'



        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_sub= self.create_subscription(Pose2D,odom_topic,self.odom_callback,10)


        self.get_logger().info("Robot controller started!")

    def odom_callback(self,msg):
        #update current position based on odometry
        self.get_logger().info("Current position is updated.")
        self.current_position[0] = msg.x
        self.current_position[1] = msg.y
        self.current_position[2] = msg.theta






    def scan_callback(self,scan):
        cmd = Twist()

        # 1. Obstacle avoidance in front of the robot (highest priority)
        front = self.get_distance(scan, 0)
        if front < 0.18:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8
            self.cmd_pub.publish(cmd)
            return
        if front < FRONT_OBSTACLE_THRESHOLD:
            cmd.linear.x = MIN_LINEAR_SPEED
            cmd.angular.z = 0.5
            self.cmd_pub.publish(cmd)
            return

        # try to generate new velocity command taking into consideration: obstacles, goal position
        # strategy: 
        
        #   1) first generate a velocity command towards goal position.
        #   2) check for (apporaching) obstacles or things in the way: 
        #   --> if there's something in the way avoid it by turning left or right

        # Get current position and goal
        x, y, theta = self.current_position  # [x, y, theta]
        goal_x, goal_y = self.goal_position  # [goal_x, goal_y]

        # Compute vector to goal
        dx = goal_x - x
        dy = goal_y - y
        distance_to_goal = math.hypot(dx, dy)

        # Stop if close enough
        if distance_to_goal < GOAL_TOLERANCE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Compute heading
            desired_theta = math.atan2(dy, dx)
            heading_error = desired_theta - theta

            # Wrap heading_error to [-pi, pi]
            heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

            # Compute proportional velocities
            cmd.linear.x = min(K_LINEAR * distance_to_goal, MAX_LINEAR)
            cmd.angular.z = max(-MAX_ANGULAR, min(K_ANGULAR * heading_error, MAX_ANGULAR))

        # Publish command
        self.cmd_pub.publish(cmd)

        

########## Helpers ###########
    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        idx = int(round((angle_rad - angle_min) / angle_inc))
        return max(0, min(len(scan.ranges) - 1, idx))

    def get_distance(self, scan: LaserScan, angle_deg: float) -> float:
        """Return range at given angle (deg), clamped to range_max."""
        angle_rad = math.radians(angle_deg)
        idx = self.angle_to_index(scan, angle_rad)
        r = scan.ranges[idx]
        if math.isinf(r) or math.isnan(r):
            r = scan.range_max
        return float(r)
##############################


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()