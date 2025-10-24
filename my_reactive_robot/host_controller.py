#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from std_msgs.msg import Float32



DESIRED_DISTANCE_FROM_WALL = 0.35
LINEAR_SPEED = 0.18
MIN_LINEAR_SPEED = 0.03
FRONT_OBSTACLE_THRESHOLD = 0.35

# PID parameters
KP = 0.6
KI = 0.0
KD = 0.1
K_ALPHA = 0.8

MAX_OMEGA = 2.5  # max turning rate in rad/s
SLOW_SCALE_FACTOR = 0.75

# Angles (deg) used for fitting wall
SAMPLE_START_DEG = -100
SAMPLE_END_DEG = -20
SAMPLE_STEP_DEG = 5

#Distance to second bot
DISTANCE_TO_BOT=0.8


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_reactive')

         # --- Configuration Parameters ---
        self.TURTLEBOT_RADIUS = 0.14  # Approx radius/diameter for size check (m)
        self.MIN_OBJECT_SIZE = 0.10   # Minimum cluster width (m)
        self.MAX_OBJECT_SIZE = 0.30   # Maximum cluster width (m)
        self.DISTANCE_THRESHOLD = 0.05 # Max distance between consecutive points in a cluster (m)
        self.LIDAR_MAX_RANGE = 3.5    # Max range of the simulated TurtleBot3 LDS-01


        namespace = self.get_namespace()
        cmd_topic = f'{namespace}/cmd_vel' if namespace != '/' else '/cmd_vel'
        scan_topic = f'{namespace}/scan' if namespace != '/' else '/scan'

        self.wait_for_follower=False
        self.distance_to_follower=None



        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        # Only PID memory allowed
        self.integral = 0.0
        self.prev_error = 0.0

        self.get_logger().info("Host controller started!")



    def _polar_to_cartesian(self, ranges, angle_min, angle_increment):
        """Converts polar Lidar readings to 2D Cartesian coordinates (x, y)."""
        
        # Filter out invalid readings (inf or 0.0)
        valid_indices = (ranges < self.LIDAR_MAX_RANGE) & (ranges > 0.05)
        valid_ranges = ranges[valid_indices]
        
        # Calculate angles for valid points
        angles = angle_min + np.where(valid_indices)[0] * angle_increment
        
        # Convert to Cartesian
        x = valid_ranges * np.cos(angles)
        y = valid_ranges * np.sin(angles)
        
        # Return a list of points (x, y)
        return np.vstack((x, y)).T

    def _cluster_by_distance(self, points):
        """
        Groups adjacent Lidar points into clusters if they are close enough.
        This is a simplified approach to clustering, replacing full DBSCAN.
        """

        self.get_logger().info("in cluster by distance")

        if len(points) < 2:
            return []

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            # Calculate Euclidean distance between current point and previous point
            distance = np.linalg.norm(points[i] - points[i-1])

            if distance < self.DISTANCE_THRESHOLD:
                # Points are close, add to current cluster
                current_cluster.append(points[i])
            else:
                # Gap is too large, finalize previous cluster and start a new one
                if len(current_cluster) > 1: # Require at least 2 points to form an object
                    clusters.append(np.array(current_cluster))
                current_cluster = [points[i]]

        # Add the last cluster if it's valid
        if len(current_cluster) > 1:
            clusters.append(np.array(current_cluster))

        return clusters

    def _filter_cluster_by_size(self, cluster):
        """
        Checks if a cluster's size and dimensions match a TurtleBot3.
        Size is estimated by the spread (bounding box) of points in the cluster.
        """
        self.get_logger().info("in filter cluster by size")
       
        # Calculate the extent (max span) of the cluster points
        min_x, max_x = np.min(cluster[:, 0]), np.max(cluster[:, 0])
        min_y, max_y = np.min(cluster[:, 1]), np.max(cluster[:, 1])

        width = max_x - min_x
        height = max_y - min_y
        
        # The TurtleBot3 is roughly circular, so its width and height should be similar
        # and fall within the defined size limits.
        
        # Check if either dimension falls within the expected range
        is_correct_size = (self.MIN_OBJECT_SIZE < width < self.MAX_OBJECT_SIZE) or \
                          (self.MIN_OBJECT_SIZE < height < self.MAX_OBJECT_SIZE)
        
        # Also check for aspect ratio (optional but good for filtering non-robots)
        aspect_ratio = max(width, height) / (min(width, height) + 1e-6) # Add small epsilon to avoid div by zero
        is_reasonable_shape = aspect_ratio < 3.0 # Assuming the robot isn't severely elongated in the scan

        return is_correct_size and is_reasonable_shape


    def detect_follower(self, msg: LaserScan):
        """        
        Processes lidar data to find the target robot.
        """
        self.get_logger().info("in detect_follower")
        
        # Convert raw ranges to a NumPy array for processing
        ranges = np.array(msg.ranges)
        
        # 1. Convert Lidar data to Cartesian coordinates
        cartesian_points = self._polar_to_cartesian(
            ranges,
            msg.angle_min,
            msg.angle_increment
        )
        
        # 2. Cluster the points to identify distinct objects
        clusters = self._cluster_by_distance(cartesian_points)
        
        closest_target_distance = self.LIDAR_MAX_RANGE + 1.0 # Initialize to a very large distance
        
        # 3. Filter clusters by size and find the closest matching target
        for cluster in clusters:
            if self._filter_cluster_by_size(cluster):
                # We found a cluster that matches the expected size of a TurtleBot3
                
                # Calculate the minimum distance from Robot 1 (origin) to the cluster
                # Distance of a point (x, y) from origin is sqrt(x^2 + y^2)
                distances = np.linalg.norm(cluster, axis=1)
                min_cluster_distance = np.min(distances)
                
                # Track the closest detected target
                if min_cluster_distance < closest_target_distance:
                    closest_target_distance = min_cluster_distance

        # 4. Publish the closest distance found
        distance_msg = Float32()
        
        # If no robot was detected, publish a value greater than the max detection range 
        # (or an agreed-upon "not detected" distance)
        if closest_target_distance > self.LIDAR_MAX_RANGE:
            distance_msg.data = closest_target_distance 
            self.get_logger().debug("Target robot not detected.")
        else:
            distance_msg.data = float(closest_target_distance)
            self.get_logger().info(f"Target robot detected at {distance_msg.data:.3f} m.")
        
        self.distance_to_follower=closest_target_distance

        #self.distance_pub.publish(distance_msg)


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

    def get_sample_points(self, scan: LaserScan) -> np.ndarray:
        """Sample (x, y) points on the right side from LaserScan."""
        points = []
        for angle_deg in range(SAMPLE_START_DEG, SAMPLE_END_DEG + 1, SAMPLE_STEP_DEG):
            r = self.get_distance(scan, angle_deg)
            if 0.05 < r < scan.range_max - 0.05:
                angle_rad = math.radians(angle_deg)
                x = r * math.cos(angle_rad)
                y = r * math.sin(angle_rad)
                points.append((x, y))
        return np.array(points) if len(points) > 1 else np.empty((0, 2))

    def fit_line_pca(self, points: np.ndarray):
        """
        Fit a line to 2D points using PCA (total least squares) -> returns (alpha, dist, fit_error).
        - alpha: wall orientation [rad]
        - dist: perpendicular distance robot -> wall
        - fit_error: perpendicular residual std dev (for corner detection)
        """
        if len(points) < 2:
            return 0.0, float('inf'), 0.0

        centroid = points.mean(axis=0)
        Xc = points - centroid
        U, S, Vt = np.linalg.svd(Xc, full_matrices=False)
        dir_vec = Vt[0, :] / np.linalg.norm(Vt[0, :])
        perp = np.array([-dir_vec[1], dir_vec[0]])
        dist = abs(np.dot(centroid, perp))
        alpha = math.atan2(dir_vec[1], dir_vec[0])
        fit_error = np.std(Xc @ perp)
        return alpha, dist, fit_error


    def scan_callback(self, scan: LaserScan):
        #call method to calculate distance to follower
        detect_follower(scan)

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

        # 2. Estimate wall using multiple beams
        points = self.get_sample_points(scan)
        alpha, dist, fit_err = self.fit_line_pca(points)

        if not np.isfinite(dist) or dist > scan.range_max:
            # fallback: use single ray
            dist = self.get_distance(scan, -90)
            alpha = 0.0
            fit_err = 0.0

        # 3. Compute PID control
        error = DESIRED_DISTANCE_FROM_WALL - dist
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        pid_term = KP * error + KI * self.integral + KD * derivative
        ff = K_ALPHA * alpha
        omega = pid_term + ff
        omega = float(np.clip(omega, -MAX_OMEGA, MAX_OMEGA))

        # 4. Reactive speed scaling (optional???)
        corner_score = min(1.0, fit_err / 0.05)   # >0.05 m = strong curve
        turn_demand = abs(omega) / MAX_OMEGA
        combined_turn = max(corner_score, turn_demand)

        min_scale = MIN_LINEAR_SPEED / LINEAR_SPEED
        speed_scale = max(min_scale, 1.0 - SLOW_SCALE_FACTOR * combined_turn)

        cmd.linear.x = LINEAR_SPEED * speed_scale
        cmd.angular.z = omega

        #don't move if follower is too far away
        if self.distance_to_follower != None:
            if self.distance_to_follower> DISTANCE_TO_BOT:
                cmd.linear.x=0.0
                cmd.angular.z=0.0
                self.get_logger().info("Guest too far away so I wait.")
        else:
            self.get_logger().info("Distance to follower is None.")



        

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"dist={dist:.2f} err={error:.2f} alpha={math.degrees(alpha):.1f}° "
            f"fit_err={fit_err:.3f} v={cmd.linear.x:.2f}"
        )


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