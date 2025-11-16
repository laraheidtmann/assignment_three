#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import math
import time

# Initial position: x, y, theta
INITIAL_POS = [0.0, 0.0, 0.0]

class OdomCalc(Node):
    def __init__(self):
        super().__init__('odom_calc')

        # Robot pose
        self.x, self.y, self.theta = INITIAL_POS
        self.last_time = time.time()

        # ROS topics
        namespace = self.get_namespace()
        cmd_topic = f'{namespace}/cmd_vel' if namespace != '/' else '/cmd_vel'

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, cmd_topic, self.cmd_callback, 10)

        # Publisher
        # We will publish the integrated odometry as a Pose2D
        self.odom_pub = self.create_publisher(Pose2D, 'odom', 10)

        self.get_logger().info('Odometry calculator started!')

        # Current velocity commands
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0


    def cmd_callback(self, msg: Twist):
        """
        Store the latest commanded velocities and integrate position based on last velocity commands.

        """
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z

        now=time.time()

        dt = (now - self.last_time) # seconds
        #self.get_logger().info(f"Elapsed time for dt: {dt}")
        self.last_time = now

        # Simple differential-drive integration (assuming velocities are in robot frame)
        delta_x = self.current_linear_vel * math.cos(self.theta) * dt
        delta_y = self.current_linear_vel * math.sin(self.theta) * dt
        delta_theta = self.current_angular_vel * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Wrap theta between -pi and pi
        #self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Publish as Pose2D
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta

        self.odom_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCalc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
