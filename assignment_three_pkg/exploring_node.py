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
        super().__init__('exploring_node')

        # -------- PARAMETERS --------

        self.goal_has_been_set=False #as soon as a goal has been set, exploring stops

        self.safe_distance = 0.30          # start avoiding
        self.resume_distance = 0.40        # hysteresis, start moving forward again
        self.forward_speed = 0.22
        self.turn_speed = 0.5
        self.front_width_deg = 60.0        # field of view for front
        self.stuck_timeout = 4.0           # seconds turning before escape
        self.escape_turn = 1.2             # rad/s
        self.escape_time = 1.5             # seconds backing + turning

        # INTERNAL STATE
        self.mode = "FORWARD"              # FORWARD, AVOID, ESCAPE
        self.last_switch_time = time.time()

        self.turn_direction = 0.0          # +1 left, -1 right
        self.turning_start_time = None

        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.goal_sub=self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,10)


        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Improved Explorer node started!")
        
    def goal_callback(self, msg: PoseStamped):
        self.goal_has_been_set=True
        self.get_logger().info("Goal received, stopping exploration.")

    # -------------------------------------------------------
    #    HELPER: find minimum distance in angle range
    # -------------------------------------------------------
    def min_in_sector(self, clean_ranges, scan, angle_from_rad, angle_to_rad):
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        inc = scan.angle_increment
        n = len(clean_ranges)

        idx_from = int((angle_from_rad - angle_min) / inc)
        idx_to = int((angle_to_rad - angle_min) / inc)

        idx_from = max(0, min(n-1, idx_from))
        idx_to   = max(0, min(n-1, idx_to))

        if idx_from > idx_to:
            idx_from, idx_to = idx_to, idx_from

        return float(np.min(clean_ranges[idx_from:idx_to+1]))

    # -------------------------------------------------------
    #    MAIN SCAN CALLBACK
    # -------------------------------------------------------
    def scan_callback(self, scan: LaserScan):
        if self.goal_has_been_set:
            return  # stop exploring if a goal has been set
        # clean ranges
        max_range = scan.range_max if scan.range_max > 0 else 4.0
        clean_ranges = np.array([
            r if (not math.isnan(r) and not math.isinf(r)) else max_range
            for r in scan.ranges
        ], dtype=float)

        # sector angles (convert degrees to radians)
        half_front = math.radians(self.front_width_deg / 2.0)

        front_min = self.min_in_sector(clean_ranges, scan, -half_front, +half_front)
        left_min  = self.min_in_sector(clean_ranges, scan, +half_front, math.radians(110))
        right_min = self.min_in_sector(clean_ranges, scan, math.radians(-110), -half_front)

        cmd = Twist()

        # ---------------------------------------------------
        #   STATE MACHINE
        # ---------------------------------------------------

        now = time.time()

        # -------------------- ESCAPE MODE --------------------
        if self.mode == "ESCAPE":
            cmd.linear.x = -0.12     # small reverse
            cmd.angular.z = self.escape_turn * self.turn_direction

            # escape lasts fixed time
            if now - self.last_switch_time > self.escape_time:
                self.mode = "FORWARD"
                self.get_logger().info("ESCAPE complete → FORWARD")
            self.cmd_pub.publish(cmd)
            return

        # -------------------- STUCK DETECTION --------------------
        # If turning for too long → stuck → escape
        if self.mode == "AVOID":
            if self.turning_start_time is None:
                self.turning_start_time = now
            if now - self.turning_start_time > self.stuck_timeout:
                self.get_logger().warn("Detected STUCK → ESCAPE MANEUVER")
                self.mode = "ESCAPE"
                self.last_switch_time = now
                self.cmd_pub.publish(cmd)
                return
        else:
            self.turning_start_time = None

        # -------------------- FORWARD MODE --------------------
        if self.mode == "FORWARD":
            if front_min < self.safe_distance:
                self.mode = "AVOID"
                self.turn_direction = +1 if left_min > right_min else -1
                self.last_switch_time = now
                self.get_logger().info(f"Obstacle ahead → AVOID (turn {self.turn_direction})")
            else:
                # Move forward normally
                cmd.linear.x = self.forward_speed
                self.cmd_pub.publish(cmd)
                return

        # -------------------- AVOID MODE --------------------
        if self.mode == "AVOID":
            # Check if we can resume forward motion (hysteresis)
            if front_min > self.resume_distance:
                self.mode = "FORWARD"
                self.get_logger().info("Clear path → FORWARD")
                cmd.linear.x = self.forward_speed
                self.cmd_pub.publish(cmd)
                return

            # turn away from obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed * self.turn_direction
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
