#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import time


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

# Sinusoidal speed parameters
MAX_SIN_SPEED = 0.25  # Max linear speed (m/s)
MIN_SIN_SPEED = 0.10  # Min linear speed (m/s)
SIN_PERIOD_S = 4.0    # Period of the sine wave in seconds


class GuestController(Node):
    def __init__(self):
        super().__init__('guest_controller_reactive')

        namespace = self.get_namespace()
        cmd_topic = f'{namespace}/cmd_vel' if namespace != '/' else '/cmd_vel'
        scan_topic = f'{namespace}/scan' if namespace != '/' else '/scan'
        
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        # Only PID memory allowed
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.time_start = time.time()

        self.get_logger().info(f"✅ Guest controller started!")


    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        idx = int(round((angle_rad - angle_min) / angle_inc))
        return max(0, min(len(scan.ranges) - 1, idx))

    def get_distance(self, scan: LaserScan, angle_deg: float) -> float:
        """Return range at given angle (deg), clamped to range_max."""
        def deg_to_rad_wrapped(deg):
            rad = math.radians(deg)
            while rad > math.pi:
                rad -= 2*math.pi
            while rad < -math.pi:
                rad += 2*math.pi
            return rad
        
        angle_rad = math.radians(angle_deg)
        angle_rad = deg_to_rad_wrapped(angle_deg)
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
        
        # 5. Calculate time-varying base speed (sinusoidal)
        time_now = time.time()
        elapsed_time = time_now - self.time_start
        sin_value = math.sin(2 * math.pi * (elapsed_time / SIN_PERIOD_S))
        
        # Map sine wave (-1 to 1) to speed range (MIN_SIN_SPEED to MAX_SIN_SPEED)
        # Amplitude is (MAX_SIN_SPEED - MIN_SIN_SPEED) / 2
        # Center is (MAX_SIN_SPEED + MIN_SIN_SPEED) / 2
        amplitude = (MAX_SIN_SPEED - MIN_SIN_SPEED) / 2.0
        center = (MAX_SIN_SPEED + MIN_SIN_SPEED) / 2.0
        base_speed = center + amplitude * sin_value

        min_scale = MIN_LINEAR_SPEED / base_speed
        speed_scale = max(min_scale, 1.0 - SLOW_SCALE_FACTOR * combined_turn)

        cmd.linear.x = base_speed * speed_scale
        cmd.angular.z = omega

        self.cmd_pub.publish(cmd)

        # self.get_logger().info(
        #     f"dist={dist:.2f} err={error:.2f} alpha={math.degrees(alpha):.1f}° "
        #     f"fit_err={fit_err:.3f} v={cmd.linear.x:.2f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = GuestController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()