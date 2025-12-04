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
        super().__init__('smooth_explorer_v2')

        # ---------- PARAMETERS ----------
        self.goal_has_been_set = False

        # Max / min speeds (very gentle for SLAM)
        self.max_forward = 0.20        # m/s
        self.min_forward = 0.06        # m/s
        self.max_turn = 0.30           # rad/s

        # Distances (meters)
        self.safe_dist = 0.45          # start slowing / turning
        self.clear_dist = 1.0          # considered "far enough"

        # Sector definitions
        self.front_width_deg = 60.0
        self.side_width_deg = 80.0

        # Smoothing (low-pass filter on cmd_vel)
        self.smooth_alpha = 0.25       # between 0 and 1; smaller = smoother
        self.prev_cmd = Twist()

        # For simple stuck detection (no aggressive unstuck!)
        self.last_progress_time = time.time()
        self.progress_timeout = 15.0   # s without "free" space → small wiggle

        # ROS I/O
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Smooth, SLAM-friendly Explorer v2 started.")

    # ---------------------------------------------------
    def goal_callback(self, msg):
        # When a goal is set (for Nav2 etc.), stop exploring
        self.goal_has_been_set = True
        self.get_logger().warn("Goal received → stopping exploration.")
        stop = Twist()
        self.prev_cmd = stop
        self.cmd_pub.publish(stop)

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
    def smooth_cmd(self, desired: Twist) -> Twist:
        """First-order low-pass filter on linear & angular velocity."""
        out = Twist()

        a = self.smooth_alpha
        prev = self.prev_cmd

        out.linear.x = prev.linear.x + a * (desired.linear.x - prev.linear.x)
        out.angular.z = prev.angular.z + a * (desired.angular.z - prev.angular.z)

        # Save and return
        self.prev_cmd = out
        return out

    # ---------------------------------------------------
    def scan_callback(self, scan: LaserScan):
        if self.goal_has_been_set:
            # Nav2 / other node should take over
            return

        # Clean ranges: replace inf/NaN with max range
        max_r = scan.range_max if scan.range_max > 0 else 4.0
        arr = np.array([
            r if (not math.isnan(r) and not math.isinf(r)) else max_r
            for r in scan.ranges
        ])

        # Define sectors in robot frame:
        #  0 rad = straight ahead, +left, -right
        half_front = math.radians(self.front_width_deg / 2.0)
        left_start = half_front
        left_end = left_start + math.radians(self.side_width_deg)
        right_start = -half_front - math.radians(self.side_width_deg)
        right_end = -half_front

        front = self.min_in_sector(arr, scan, -half_front, +half_front)
        left = self.min_in_sector(arr, scan, left_start, left_end)
        right = self.min_in_sector(arr, scan, right_start, right_end)

        # -------------- BASIC SAFETY --------------
        desired = Twist()

        # If everything is extremely close, just stop
        critical_dist = 0.25
        if front < critical_dist and left < critical_dist and right < critical_dist:
            self.get_logger().warn_once("Very close to obstacles → stopping.")
            # keep desired = 0 (full stop), but smoothed
            cmd = self.smooth_cmd(desired)
            self.cmd_pub.publish(cmd)
            return

        # -------------- FORWARD SPEED --------------
        # Map front distance to forward speed smoothly.
        # Below safe_dist: slow down strongly, maybe almost stop.
        if front <= self.safe_dist:
            speed_scale = 0.1  # almost stopping but not backward
        else:
            # Linearly scale between safe_dist and clear_dist
            d = max(0.0, min(self.clear_dist, front) - self.safe_dist)
            span = max(0.01, self.clear_dist - self.safe_dist)
            speed_scale = d / span  # 0..1

        forward = self.min_forward + speed_scale * (self.max_forward - self.min_forward)
        forward = max(0.0, min(self.max_forward, forward))
        desired.linear.x = forward

        # -------------- TURNING (SMOOTH) --------------
        # 1) Steer away from closer side (left vs right)
        # Larger distance = more free space = steer that way
        side_diff = left - right  # >0 → more free on left
        max_diff = 1.0  # meters; used for normalizing

        side_term = max(-1.0, min(1.0, side_diff / max_diff))
        side_turn_gain = 0.25 * self.max_turn
        turn_from_sides = side_turn_gain * side_term  # ± about 0.25*max_turn

        # 2) Stronger turn if front is close, toward the freer side
        if front < self.safe_dist:
            front_scale = 1.0 - (front / max(self.safe_dist, 0.01))
            front_scale = max(0.0, min(1.0, front_scale))
            dir_sign = +1.0 if left > right else -1.0
            front_turn_gain = 0.75 * self.max_turn
            turn_from_front = dir_sign * front_turn_gain * front_scale
        else:
            turn_from_front = 0.0

        turn = turn_from_sides + turn_from_front

        # If we are very free ahead and sides balanced, keep turn small
        if front > self.clear_dist and abs(side_diff) < 0.1:
            turn *= 0.3

        # Final clamp
        desired.angular.z = max(-self.max_turn, min(self.max_turn, turn))

        # -------------- SIMPLE "STUCK" HANDLING --------------
        # If we haven't seen "good" free space in a while, add a gentle bias
        now = time.time()
        if front > 0.9 * self.clear_dist:
            self.last_progress_time = now

        if now - self.last_progress_time > self.progress_timeout:
            # Tiny wiggle to break symmetry (won't spin aggressively)
            self.get_logger().info("Likely stuck in a local pattern → gentle bias turn.")
            desired.angular.z += 0.15  # small constant left bias
            desired.angular.z = max(-self.max_turn, min(self.max_turn, desired.angular.z))

        # -------------- SMOOTH & PUBLISH --------------
        cmd = self.smooth_cmd(desired)
        self.cmd_pub.publish(cmd)


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
