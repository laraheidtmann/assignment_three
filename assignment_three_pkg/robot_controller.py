#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose2D
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from std_msgs.msg import Float32
from assignment_three_pkg.obstacle import Obstacle

# Robot motion
LINEAR_SPEED = 0.18
MIN_LINEAR_SPEED = 0.03
MAX_LINEAR = 0.15
MAX_ANGULAR = 1.0

# Obstacle detection
FRONT_OBSTACLE_THRESHOLD = 0.35  # m
OBSTACLE_FRONT_MIN_ANGLE = -15   # deg
OBSTACLE_FRONT_MAX_ANGLE = 15    # deg
OBSTACLE_DETECTION_DELTA = 0.5   # threshold to detect discontinuity in scan

# Control gains
K_LINEAR = 0.2
K_ANGULAR = 1.5
GOAL_TOLERANCE = 0.05  # distance to goal to stop

# Avoidance
AVOIDANCE_ANGLE_OFFSET = 30  # degrees to steer around obstacle
Kp_TURN = 0.6  # angular proportional gain during avoidance

class TurtleRobot(Node):
    def __init__(self):
        super().__init__('reactive_robot')
        self.current_position = [0.0, 0.0, 0.0]  # x, y, theta
        self.goal_position=[1.0, 1.0]

        namespace = self.get_namespace()
        cmd_topic = f'{namespace}/cmd_vel' if namespace != '/' else '/cmd_vel'
        scan_topic = f'{namespace}/scan' if namespace != '/' else '/scan'
        odom_topic = f'{namespace}/odom' if namespace != '/' else '/odom'



        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub= self.create_subscription(Pose2D,odom_topic,self.odom_callback,10)


        self.get_logger().info("Robot controller started!")

    def odom_callback(self,msg):
        #update current position based on odometry
        self.current_position[0] = msg.x
        self.current_position[1] = msg.y
        self.current_position[2] = msg.theta


    
    def object_detection(self, scan: LaserScan):

        FRONT_MIN_ANGLE = -15
        FRONT_MAX_ANGLE = 15
        DIST_THRESHOLD = 0.5   # meters (set as needed)

        range_begin = -179
        range_end   = 180
        last_ray_distance = self.get_distance(scan, range_begin)
        obstacles = []
        detected_object = None

        for angle in range(range_begin, range_end):
            current_ray_distance = self.get_distance(scan, angle)

            if 0 < current_ray_distance < float('inf'):

                # object begins (large drop)
                if last_ray_distance - current_ray_distance > 0.5:
                    detected_object = Obstacle()
                    detected_object.set_beginning_border(current_ray_distance, angle)
                    #self.get_logger().info(f"object begin detected:{current_ray_distance}")


                # object ends (large rise)
                if current_ray_distance - last_ray_distance > 0.5:
                    if detected_object is not None:
                        detected_object.set_ending_border(current_ray_distance, angle)
                        obstacles.append(detected_object)
                        #self.get_logger().info(f"object end detected:{current_ray_distance}")

                    detected_object = None

                if detected_object != None:
                        detected_object.lidar_points.append((angle, current_ray_distance))
                
                last_ray_distance = current_ray_distance


        
        if obstacles:
            #self.get_logger().info("=== DETECTED OBSTACLES ===")
            for i, obs in enumerate(obstacles):
                min_dist = obs.min_distance()
                # self.get_logger().info(
                #     f"Obstacle {i}: "
                #     f"Begin(angle={obs.beginning_border_angle}, dist={obs.beginning_border_dist:.2f}), "
                #     f"End(angle={obs.ending_border_angle}, dist={obs.ending_border_dist:.2f}), "
                #     f"MinDist={min_dist:.2f} m, "
                #     f"Points={len(obs.lidar_points)}"
                #)
        else:
            self.get_logger().info("No obstacles detected.")
                

        for obstacle in obstacles:
            # 1) is this obstacle in front (any lidar point near 0°)?
            if not obstacle.is_in_front(FRONT_MIN_ANGLE, FRONT_MAX_ANGLE):
                continue

            # 2) is it closer than threshold?
            min_dist = obstacle.min_distance()
            if min_dist is None:
                continue

            if min_dist < DIST_THRESHOLD:
                self.get_logger().info(
                    f"Obstacle in front within threshold! min_dist={min_dist:.2f} m"
                )
                return obstacle   # return the obstacle object for avoidance

        # no obstacle in front needing avoidance
        return False

    def scan_callback(self, scan):
        cmd = Twist()

        # Detect obstacles in front
        obstacle = self.object_detection(scan)

        if obstacle:
            #self.get_logger().info("Obstacle detected in front. Avoiding...")

            # Compute center angle of obstacle
            if obstacle.lidar_points:
                angles = [a for a, d in obstacle.lidar_points]
                center_angle = sum(angles) / len(angles)
            else:
                center_angle = (obstacle.beginning_border_angle + obstacle.ending_border_angle) / 2

            # Decide avoidance direction
            # steer away from the obstacle
            if center_angle >= 0:
                # obstacle slightly right -> turn left
                avoid_angle_deg = center_angle - 30
            else:
                # obstacle slightly left -> turn right
                avoid_angle_deg = center_angle + 30

            # Convert to radians and compute angular velocity
            error_angle = math.radians(avoid_angle_deg)
            Kp_turn = 0.6  # tuning parameter
            cmd.angular.z = max(-MAX_ANGULAR, min(Kp_turn * error_angle, MAX_ANGULAR))

            # Slow linear speed when obstacle is near
            min_dist = obstacle.min_distance()
            if min_dist:
                cmd.linear.x = max(MIN_LINEAR_SPEED, LINEAR_SPEED * (min_dist / FRONT_OBSTACLE_THRESHOLD))
            else:
                cmd.linear.x = LINEAR_SPEED * 0.5

            self.cmd_pub.publish(cmd)
            return  # skip goal navigation while avoiding obstacle

        # Goal navigation if no immediate obstacle
        x, y, theta = self.current_position
        goal_x, goal_y = self.goal_position
        dx = goal_x - x
        dy = goal_y - y
        distance_to_goal = math.hypot(dx, dy)

        if distance_to_goal < GOAL_TOLERANCE:
            self.get_logger().info("Goal reached!")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            desired_theta = math.atan2(dy, dx)
            heading_error = desired_theta - theta
            heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

            cmd.linear.x = min(K_LINEAR * distance_to_goal, MAX_LINEAR)
            cmd.angular.z = max(-MAX_ANGULAR, min(K_ANGULAR * heading_error, MAX_ANGULAR))

        self.cmd_pub.publish(cmd)


    def scan_callback_old(self,scan):
        cmd = Twist()
        obstacle= self.object_detection(scan)
        if obstacle != False:
            self.get_logger().info("Obstacle detected. Trying to avoid it")
            begin_dist, begin_angle = obstacle.get_beginning_border()
            end_dist, end_angle     = obstacle.get_ending_border()

            #navigate to the closer obstacle boundary
            if begin_dist<end_dist:
                goal_dist=begin_dist
                goal_angle=begin_angle-10
            else:
                goal_dist=end_dist
                goal_angle=end_angle+10 #extra angle s.t. robot does not crash into object BUT how does it knoe if its -/+?

            error_angle = math.radians(goal_angle)

            Kp_turn = 0.4   # tune this value
            cmd.angular.z = Kp_turn * error_angle

            #how do i formulate the cmd??
            cmd.linear.x = 0.1   # slow forward speed
            self.cmd_pub.publish(cmd)
            return


        #2. if the robot is closer than a certain threshold to an obstacle, turn right or left 
        # d

        # try to generate new velocity command taking into consideration: obstacles, goal position
        # strategy: 
        
        #   1) first generate a velocity command towards goal position. later, perhaps a trajectory
        #   2) check for (apporaching) obstacles or things in the way: 
        #   --> if there's something in the way avoid it by turning left or right

        # Get current position and goal
        x, y, theta = self.current_position  # [x, y, theta] obtained with odometry
        goal_x, goal_y = self.goal_position  # [goal_x, goal_y]  fixed position defined at the beginning 

        # Compute vector to goal
        dx = goal_x - x
        dy = goal_y - y
        distance_to_goal = math.hypot(dx, dy)

        # Stop if close enough
        if distance_to_goal < GOAL_TOLERANCE:
            self.get_logger().info("Goal has been reached successfully!")
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
    node = TurtleRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()