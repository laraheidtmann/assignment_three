#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus
import csv
import os
import math
import time

class MetricLogger(Node):

    def __init__(self):
        super().__init__('metric_logger')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('scenario_id', 'default')
        self.scenario_id = self.get_parameter('scenario_id').value

        # ---------------- STATE ----------------
        self.trial_active = False
        self.goal_pose = None
        self.start_time = None
        self.path_length = 0.0
        self.last_position = None
        self.replans = 0
        self.min_clearance = float('inf')

        # ---------------- CSV ----------------
        log_dir = os.path.expanduser('~/nav_metrics')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, 'metrics.csv')

        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'scenario_id',
                    'goal_x',
                    'goal_y',
                    'success',
                    'time_to_goal',
                    'path_length',
                    'replans',
                    'min_obstacle_clearance'
                ])

        # ---------------- SUBSCRIPTIONS ----------------
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(PoseStamped, '/plan', self.plan_cb, 10)

        # ---------------- ACTION CLIENT ----------------
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.get_logger().info('Metric logger initialized')

    # ==================================================
    # GOAL HANDLING
    # ==================================================

    def goal_cb(self, msg: PoseStamped):
        if self.trial_active:
            self.get_logger().warn('Previous trial unfinished — marking failed')
            self.end_trial(success=False)

        self.goal_pose = msg
        self.start_trial()
        self.send_nav_goal(msg)

    def send_nav_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            self.end_trial(success=False)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending goal to Nav2: ({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f})"
        )

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self.end_trial(success=False)
            return

        self.get_logger().info('Goal accepted by Nav2')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        if not self.trial_active:
            return

        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 reports goal SUCCEEDED')
            self.end_trial(success=True)
        else:
            self.get_logger().warn(f'Nav2 reports goal FAILED (status={status})')
            self.end_trial(success=False)

    # ==================================================
    # METRICS
    # ==================================================

    def start_trial(self):
        self.trial_active = True
        self.start_time = time.time()
        self.path_length = 0.0
        self.last_position = None
        self.replans = 0
        self.min_clearance = float('inf')

        self.get_logger().info(
            f"Trial started: goal ({self.goal_pose.pose.position.x:.2f}, "
            f"{self.goal_pose.pose.position.y:.2f})"
        )

    def odom_cb(self, msg: Odometry):
        if not self.trial_active:
            return

        pos = msg.pose.pose.position
        if self.last_position is not None:
            dx = pos.x - self.last_position.x
            dy = pos.y - self.last_position.y
            self.path_length += math.hypot(dx, dy)
        self.last_position = pos

    def scan_cb(self, msg: LaserScan):
        if not self.trial_active:
            return

        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            self.min_clearance = min(self.min_clearance, min(valid))

    def plan_cb(self, msg):
        if self.trial_active:
            self.replans += 1

    # ==================================================
    # END TRIAL
    # ==================================================

    def end_trial(self, success: bool):
        if not self.trial_active:
            return

        duration = time.time() - self.start_time

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.scenario_id,
                self.goal_pose.pose.position.x,
                self.goal_pose.pose.position.y,
                success,
                round(duration, 2),
                round(self.path_length, 2),
                self.replans,
                round(self.min_clearance, 3)
            ])

        self.get_logger().info(
            f"Trial ended | Success={success} "
            f"Time={duration:.2f}s PathLen={self.path_length:.2f} "
            f"Replans={self.replans} MinClearance={self.min_clearance:.2f}"
        )

        self.trial_active = False


def main(args=None):
    rclpy.init(args=args)
    node = MetricLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
