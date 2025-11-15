import rclpy

class HostNode:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # Constants
        self.WHEEL_RADIUS = 0.0205  # adjust to your robot
        self.HALF_WHEEL_DISTANCE = 0.0825  # adjust to your robot
        self.KP = 0.05  # proportional gain for wall-following
        self.DESIRED_DIST = 80  # desired distance from the wall
        self.FRONT_THRESHOLD = 50  # distance to start avoiding obstacle

        # Motors
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        self.__left_motor.setPosition(float('inf'))
        self.__right_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0.0)
        self.__right_motor.setVelocity(0.0)

        # Distance sensors
        self.__ds_front = self.__robot.getDevice('ds_front')
        self.__ds_right = self.__robot.getDevice('ds_right')
        timestep = int(self.__robot.getBasicTimeStep())
        self.__ds_front.enable(timestep)
        self.__ds_right.enable(timestep)

        # ROS2 setup (still needed by Webots integration)
        rclpy.init(args=None)
        self.__node = rclpy.create_node('host_node')

    def step(self):
        # Read sensor values
        front_dist = self.__ds_front.getValue()
        right_dist = self.__ds_right.getValue()

        forward_speed = 0.2  # base forward speed
        angular_speed = 0.0
        
        # Compute error
        error = right_dist - self.DESIRED_DIST  # positive = too far, negative = too close
        angular_speed = -self.KP * error        # turn right if too far

        # Front obstacle
        if front_dist < self.FRONT_THRESHOLD:
            forward_speed = 0.0
            angular_speed = 2.0  # turn left
        else:
            forward_speed = 0.2
        
        # Prevent overly sharp turns (optional)    
        max_angular = 2.0
        angular_speed = max(min(angular_speed, max_angular), -max_angular)

        # Compute wheel speeds (rad/s)
        left_speed = (forward_speed - angular_speed * self.HALF_WHEEL_DISTANCE) / self.WHEEL_RADIUS
        right_speed = (forward_speed + angular_speed * self.HALF_WHEEL_DISTANCE) / self.WHEEL_RADIUS

        # Apply wheel speeds
        self.__left_motor.setVelocity(left_speed)
        self.__right_motor.setVelocity(right_speed)
