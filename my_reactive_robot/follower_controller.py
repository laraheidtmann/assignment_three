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

#TODO: add a topic where the distance to the robot infront of it is measured and sent to that topic. 


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_reactive')

        namespace = self.get_namespace()
        cmd_topic = f'{namespace}/cmd_vel' if namespace != '/' else '/cmd_vel'
        scan_topic = f'{namespace}/scan' if namespace != '/' else '/scan'

        self.distance_pub=self.create_publisher(Float32,'/distance_topic',10)


        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        # Only PID memory allowed
        self.integral = 0.0
        self.prev_error = 0.0

        self.latest_front_distance = None
        self.declare_parameter("jump_threshold", 0.25) 
        self.declare_parameter("min_cluster_size", 3) 
        self.declare_parameter("leader_size_min", 0.1) # m (TurtleBot3 ~0.18 m diameter) 
        self.declare_parameter("leader_size_max", 0.8) 
        self.declare_parameter("forward_angle_limit", 180.0) 
        
        # Retrieve values 
        self.jump_threshold = self.get_parameter("jump_threshold").value 
        self.min_cluster_size = self.get_parameter("min_cluster_size").value 
        self.leader_size_min = self.get_parameter("leader_size_min").value 
        self.leader_size_max = self.get_parameter("leader_size_max").value 
        self.forward_angle_limit = self.get_parameter("forward_angle_limit").value

        self.get_logger().info("follower controller started!")




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
        cmd = Twist()
        ##### --- get distance to leader --- #####
        ranges = np.nan_to_num(np.array(scan.ranges), nan=scan.range_max, posinf=scan.range_max) 
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment 
        max_range = scan.range_max

         # --- Step 1: Cluster points sequentially ---
        clusters = []
        current = [0]
        for i in range(1, len(ranges)):
            if abs(ranges[i] - ranges[i - 1]) <= self.jump_threshold and \
            ranges[i] < max_range and ranges[i - 1] < max_range:
                current.append(i)
            else:
                if len(current) >= self.min_cluster_size:
                    clusters.append(current)
                current = [i]
        if len(current) >= self.min_cluster_size:
            clusters.append(current)

        if not clusters:
            return
        
        # --- Step 2: Filter and estimate cluster physical size ---
        potential_leader = None
        leader_distance = None
        for c in clusters:
            r = ranges[c]
            a = angles[c]
            if len(r) < 2:
                continue

            # Approximate angular width of cluster
            angle_span = abs(a[-1] - a[0])
            avg_range = np.mean(r)
            # Trigonometric width estimation (chord length)
            cluster_width = 2.0 * avg_range * np.sin(angle_span / 2.0)

            # Get centroid coordinates
            xs, ys = r * np.cos(a), r * np.sin(a)
            cx, cy = np.mean(xs), np.mean(ys)
            dist = np.hypot(cx, cy)
            theta = np.degrees(np.arctan2(cy, cx))

            # Filter by forward view and size range
            if self.leader_size_min <= cluster_width <= self.leader_size_max and \
            -self.forward_angle_limit <= theta <= self.forward_angle_limit:
                potential_leader = (cx, cy)
                leader_distance = dist
                self.get_logger().debug(
                    f"Cluster size={cluster_width:.2f} m, dist={dist:.2f} m, angle={theta:.1f}°"
                )
                break  # Stop after first matching cluster (closest in scan order)

        # --- Step 3: Publish distance to leader ---
        if potential_leader is not None and leader_distance is not None:
            msg = Float32()
            msg.data = float(leader_distance)
            self.distance_pub.publish(msg)
            self.get_logger().info(f"Leader detected at {leader_distance:.2f} m")
        else:
            self.get_logger().debug("No cluster matching leader size found.")

       

        ######### -- wall following starts here -- #########

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