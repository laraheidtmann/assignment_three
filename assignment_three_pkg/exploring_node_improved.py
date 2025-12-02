#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time

class Explorer(Node):
    def __init__(self):
        super().__init__('smooth_explorer')

        # ---------- PARAMETERS (SAFE + SMOOTH) ----------
        self.goal_has_been_set = False

        # SMOOTH SLAM-FRIENDLY SPEEDS
        self.max_forward = 0.18        # gentle
        self.min_forward = 0.05        # never stand still unless necessary
        self.max_turn = 0.35           # soft turns

        # OBSTACLE THRESHOLDS (more conservative)
        self.safe_dist = 0.45          # begin avoidance early
        self.clear_dist = 0.60         # hysteresis

        self.front_width_deg = 70

        # STUCK HANDLING (no backward!)
        self.stuck_turn_time = 3.0     # try turning for this long
        self.unstuck_spiral_turn = 0.5
        self.unstuck_forward = 0.10
        self.unstuck_time = 3.0        # how long to spiral out

        self.mode = "FORWARD"          # FORWARD, AVOID, UNSTUCK
        self.turn_direction = +1       # default direction
        self.turn_start_time = None
        self.mode_start_time = time.time()

        # ROS
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Smooth SLAM-friendly Explorer started.")

    # ---------------------------------------------------
    def goal_callback(self, msg):
        self.goal_has_been_set = True
        self.get_logger().warn("Goal received → stopping exploration.")

    # ---------------------------------------------------
    def min_in_sector(self, arr, scan, a1, a2):
        inc = scan.angle_increment
        n = len(arr)
        i1 = int((a1 - scan.angle_min) / inc)
        i2 = int((a2 - scan.angle_min) / inc)
        i1 = max(0, min(n - 1, i1))
        i2 = max(0, min(n - 1, i2))
        if i1 > i2:
            i1, i2 = i2, i1
        return float(np.min(arr[i1:i2 + 1]))

    # ---------------------------------------------------
    def scan_callback(self, scan: LaserScan):
        if self.goal_has_been_set:
            return

        # clean ranges
        max_r = scan.range_max if scan.range_max > 0 else 4.0
        arr = np.array([
            r if (not math.isnan(r) and not math.isinf(r)) else max_r
            for r in scan.ranges
        ])

        half = math.radians(self.front_width_deg / 2)
        front = self.min_in_sector(arr, scan, -half, +half)
        left  = self.min_in_sector(arr, scan, +half, math.radians(120))
        right = self.min_in_sector(arr, scan, math.radians(-120), -half)

        now = time.time()
        cmd = Twist()

        # ===================================================
        #     MODE: UNSTUCK — gentle spiral outward
        # ===================================================
        if self.mode == "UNSTUCK":
            cmd.linear.x = self.unstuck_forward
            cmd.angular.z = self.unstuck_spiral_turn * self.turn_direction

            if now - self.mode_start_time > self.unstuck_time:
                self.mode = "FORWARD"
                self.get_logger().info("Recovered → FORWARD")
            self.cmd_pub.publish(cmd)
            return

        # ===================================================
        #     STUCK DETECTION (in AVOID mode too long)
        # ===================================================
        if self.mode == "AVOID":
            if self.turn_start_time is None:
                self.turn_start_time = now

            if now - self.turn_start_time > self.stuck_turn_time:
                self.get_logger().warn("Stuck detected → spiral UNSTUCK")
                self.mode = "UNSTUCK"
                self.mode_start_time = now
                self.turn_start_time = None
                self.cmd_pub.publish(cmd)
                return
        else:
            self.turn_start_time = None

        # ===================================================
        #     MODE: FORWARD
        # ===================================================
        if self.mode == "FORWARD":
            if front < self.safe_dist:
                # switch to avoidance
                self.mode = "AVOID"
                self.mode_start_time = now
                self.turn_direction = +1 if left > right else -1
                self.get_logger().info(f"Obstacle → turning {self.turn_direction}")
            else:
                # smooth forward motion (slow down if somewhat close)
                if front < self.clear_dist + 0.4:
                    scale = (front - self.safe_dist) / (self.clear_dist - self.safe_dist)
                    scale = max(0.0, min(1.0, scale))
                    cmd.linear.x = self.min_forward + scale * (self.max_forward - self.min_forward)
                else:
                    cmd.linear.x = self.max_forward

                self.cmd_pub.publish(cmd)
                return

        # ===================================================
        #     MODE: AVOID
        # ===================================================
        if self.mode == "AVOID":
            if front > self.clear_dist:
                self.mode = "FORWARD"
                self.get_logger().info("Clear → FORWARD")
                return

            # soft turning while slowly moving forward
            cmd.linear.x = self.min_forward
            cmd.angular.z = self.turn_direction * self.max_turn
            self.cmd_pub.publish(cmd)
            return


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
