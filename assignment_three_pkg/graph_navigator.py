#!/usr/bin/env python3
import math
import heapq
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
import numpy as np
from rclpy.time import Time
import matplotlib.pyplot as plt
import os

GridIndex = Tuple[int, int]


class GraphNavigator(Node):
    def __init__(self):
        super().__init__('graph_navigator')

        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('obstacle_threshold', 50)  # >= -> obstacle
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('waypoint_dist', 0.10)

        self.map_topic = self.get_parameter('map_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.obstacle_threshold = int(self.get_parameter('obstacle_threshold').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.waypoint_dist = float(self.get_parameter('waypoint_dist').value)

        # Map storage
        self.map_received = False
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_data: Optional[np.ndarray] = None  # 2D numpy array indexed [y,x]
        self.map_resolution: Optional[float] = None
        self.map_origin: Optional[Tuple[float, float]] = None
        self.map_width: Optional[int] = None
        self.map_height: Optional[int] = None

        # Path & control state
        self.path: List[Tuple[float, float]] = []
        self.current_waypoint_index = 0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers / Publishers / Timer
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"GraphNavigator started (map={self.map_topic}, goal={self.goal_topic}, cmd_vel={self.cmd_vel_topic})"
        )


      
   

    # ---------------- Map handling ----------------
    def map_callback(self, msg: OccupancyGrid):
        # Store message and convert to 2D numpy array (shape: [height, width])
        self.map_msg = msg
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)
        self.map_resolution = float(msg.info.resolution)
        self.map_origin = (float(msg.info.origin.position.x), float(msg.info.origin.position.y))

        # OccupancyGrid.data is a flat list in row-major order (y first). Reshape accordingly.
        arr = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        self.map_data = arr  # map_data[y, x]
        self.map_received = True
        self.get_logger().info(f"Received map: {self.map_width} x {self.map_height}, res={self.map_resolution}")
        known_cells=self.count_known_cells(msg)
        self.get_logger().info(f"Known cells in map: {known_cells}")
        self.get_logger().info(f"Percentage of explored map: {known_cells / (self.map_width * self.map_height) * 100:.2f}%")
    
    def count_known_cells(self,occ_grid: OccupancyGrid) -> int:
        """Return number of cells that are not -1 (i.e., known)."""
        data = np.array(occ_grid.data, dtype=np.int8)
        return int(np.sum(data != -1))

    def get_map(self) -> Optional[np.ndarray]:
        if not self.map_received:
            return None
        return self.map_data

    # ---------------- Coordinate helpers ----------------
    def world_to_grid(self, x: float, y: float) -> Optional[GridIndex]:
        """Convert world coordinates (map frame) to grid indices (ix, iy)."""
        if not self.map_received or self.map_origin is None:
            return None
        ox, oy = self.map_origin
        mx = x - ox
        my = y - oy
        ix = int(math.floor(mx / self.map_resolution))
        iy = int(math.floor(my / self.map_resolution))
        if ix < 0 or iy < 0 or ix >= self.map_width or iy >= self.map_height:
            return None
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        """Convert grid indices (ix,iy) to world coordinates (center of cell)."""
        if not self.map_received or self.map_origin is None:
            return 0.0, 0.0
        ox, oy = self.map_origin
        x = ox + (ix + 0.5) * self.map_resolution
        y = oy + (iy + 0.5) * self.map_resolution
        return x, y

    def is_free(self, ix: int, iy: int) -> bool:
        """Return True if cell is passable (not an obstacle). Unknown (-1) treated as obstacle by default."""
        if not self.map_received:
            return False
        if ix < 0 or iy < 0 or ix >= self.map_width or iy >= self.map_height:
            return False
        v = int(self.map_data[iy, ix])
        # Treat unknown as obstacle for safe planning:
        if v == -1:
            return False
        return v < self.obstacle_threshold

    # Prevent cutting corners: for diagonal movement ensure adjacent orthogonal cells are free
    def diagonal_allowed(self, cx: int, cy: int, nx: int, ny: int) -> bool:
        dx = nx - cx
        dy = ny - cy
        if abs(dx) == 1 and abs(dy) == 1:
            # require both orthogonal neighbors free
            return self.is_free(cx + dx, cy) and self.is_free(cx, cy + dy)
        return True

    # ---------------- Robot pose helper ----------------
    def get_robot_grid_index(self) -> Optional[GridIndex]:
        if not self.map_received:
            return None
        try:
            # lookup latest transform map <- base_frame_id
            tf = self.tf_buffer.lookup_transform(
                'map', self.base_frame_id, Time(), timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        return self.world_to_grid(x, y)

    # ---------------- Goal handling ----------------
    def goal_callback(self, msg: PoseStamped):
        if not self.map_received:
            self.get_logger().warn("No map yet, cannot plan.")
            return

        # Transform goal into map frame if necessary
        goal_in_map = msg
        if msg.header.frame_id != 'map':
            try:
                goal_in_map = self.tf_buffer.transform(msg, 'map', timeout=Duration(seconds=0.5))
            except Exception as e:
                self.get_logger().warn(f"Could not transform goal into 'map' frame: {e}")
                return

        gx = goal_in_map.pose.position.x
        gy = goal_in_map.pose.position.y

        start_idx = self.get_robot_grid_index()
        goal_idx = self.world_to_grid(gx, gy)

        if start_idx is None:
            self.get_logger().warn("Could not get robot start grid index (TF or map missing).")
            return
        if goal_idx is None:
            self.get_logger().warn("Goal is outside the map or invalid.")
            return

        self.get_logger().info(f"Planning A* from {start_idx} to {goal_idx}...")
        cell_path = self.a_star(start_idx, goal_idx)
        if not cell_path:
            self.get_logger().warn("A* failed to find a path.")
            self.path = []
            return

        # Convert cell indices to world coordinates (center of cell)
        self.path = [self.grid_to_world(ix, iy) for (ix, iy) in cell_path]
        self.current_waypoint_index = 0
        self.get_logger().info(f"Path planned with {len(self.path)} waypoints.")

    # ---------------- A* implementation ----------------
    def a_star(self, start: GridIndex, goal: GridIndex) -> List[GridIndex]:
        grid = self.get_map()
        if grid is None:
            self.get_logger().info("No map data available for A*.")
            return []

        def h(a: GridIndex, b: GridIndex) -> float:
            # Euclidean distance (in cells)
            return math.hypot(a[0] - b[0], a[1] - b[1])

        neighbors = [
            (+1, 0), (-1, 0), (0, +1), (0, -1),
            (+1, +1), (+1, -1), (-1, +1), (-1, -1)
        ]

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: h(start, goal)}
        closed = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            if current == goal:
                # reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            closed.add(current)

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)

                # bounds & obstacle check
                if not self.is_free(nx, ny):
                    continue

                # prevent cutting corners diagonally
                if not self.diagonal_allowed(current[0], current[1], nx, ny):
                    continue

                tentative_g = g_score[current] + math.hypot(dx, dy)

                if neighbor in closed:
                    continue
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + h(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    # ---------------- Path following ----------------
    def control_loop(self):
        # If no path, nothing to do
        if not self.path:
            return

        # get robot current pose in map frame
        try:
            tf = self.tf_buffer.lookup_transform('map', self.base_frame_id, Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

        # If finished
        if self.current_waypoint_index >= len(self.path):
            self.path = []
            self.cmd_pub.publish(Twist())  # stop
            self.get_logger().info("Reached goal.")
            return

        wx, wy = self.path[self.current_waypoint_index]
        dx = wx - x
        dy = wy - y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - yaw)

        # Next waypoint if close enough
        if dist < self.waypoint_dist:
            self.current_waypoint_index += 1
            return

        twist = Twist()
        # If heading error large, rotate in place
        if abs(angle_error) > math.radians(20.0):
            twist.angular.z = self.angular_speed * (1.0 if angle_error > 0 else -1.0)
            twist.linear.x = 0.0
        else:
            # proportional angular velocity and constant linear
            twist.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.0 * angle_error))
            twist.linear.x = self.linear_speed

        self.cmd_pub.publish(twist)

    # ---------------- Utilities ----------------
    @staticmethod
    def quat_to_yaw(x, y, z, w) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = GraphNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
