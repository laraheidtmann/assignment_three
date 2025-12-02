import rclpy
from geometry_msgs.msg import Twist
import math
import numpy as np


HALF_DISTANCE_BETWEEN_WHEELS = 0.069
WHEEL_RADIUS = 0.033
MAX_VELOCITY = 6.67


class TurtlebotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestamp = int(self.__robot.getBasicTimeStep())
        
        rclpy.init(args=None)
        self.__node = rclpy.create_node('turtlebot_driver')

        # Motors
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)
        
        # LIDAR
        self.__lidar = self.__robot.getDevice('LDS-01')
        self.__lidar.enable(self.__timestamp)
        self.__lidar.enablePointCloud()
        
        self.__target_twist = Twist()
        robot_name=self.__robot.getName()
        cmd_topic=f'/{robot_name}/cmd_vel'

        self.__node.create_subscription(Twist, cmd_topic, self.__cmd_vel_callback, 1)
        
        self.__node.get_logger().info("TurtleBot driver initialized")

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z
        
        if math.isnan(forward_speed) or math.isnan(angular_speed):
            self.__node.get_logger().warn("NaN detected in velocity command — skipping this cycle.")
            return
        
        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        
        self.__left_motor.setVelocity(max(-MAX_VELOCITY, min(MAX_VELOCITY, command_motor_left)))
        self.__right_motor.setVelocity(max(-MAX_VELOCITY, min(MAX_VELOCITY, command_motor_right)))