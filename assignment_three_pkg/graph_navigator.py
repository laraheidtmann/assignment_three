#!/usr/bin/env python3
import math
import heapq
import tf2_geometry_msgs
from typing import List, Tuple, Optional
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, PointStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os
GridIndex = Tuple[int, int]

# graph navigator using A* to navigate through the world. It inflates the obstacles by 
# a safety distance self.safety_distance.
class GraphNavigator(Node):
    def __init__(self):
        super().__init__('graph_navigator')
        
        ros_distro = os.environ.get("ROS_DISTRO", "")
        self.use_twist_stamped = ros_distro in ["rolling", "jazzy", "kilted"]

        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('waypoint_dist', 0.10)
        self.declare_parameter('safety_distance', 0.25)  # meters

        self.map_topic = self.get_parameter('map_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.obstacle_threshold = int(self.get_parameter('obstacle_threshold').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.waypoint_dist = float(self.get_parameter('waypoint_dist').value)
        self.safety_distance = float(self.get_parameter('safety_distance').value)

        # ---------------- Map storage ----------------
        self.map_received = False
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_data: Optional[np.ndarray] = None
        self.inflated_map: Optional[np.ndarray] = None
        self.map_resolution: Optional[float] = None
        self.map_origin: Optional[Tuple[float, float]] = None
        self.map_width: Optional[int] = None
        self.map_height: Optional[int] = None
        self.inflation_radius_cells: int = 0
        self.current_goal: Optional[PoseStamped] = None
        self.current_scan=LaserScan()

        # ---------------- Path state ----------------
        self.path: List[Tuple[float, float]] = []
        self.current_waypoint_index = 0

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )


        # Subscribers / Publishers / Timer
        self.lidar_sub=self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, qos_map)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        if self.use_twist_stamped:
            self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.scan_timer=self.create_timer(1,self.scan_callback)

        self.get_logger().info("GraphNavigator with dynamic obstacle tracking started")
        # ---------------- Replan hysteresis ----------------
        self.block_counter = 0
        self.block_counter_threshold = 2   # number of consecutive detections required
        # -------- Dynamic obstacle tracking --------
        self.tracked_obstacles = {}  
        # id -> {
        #   'centroid': (x, y),
        #   'velocity': (vx, vy),
        #   'last_seen': time
        # }
        self.next_obstacle_id = 0
        self.max_tracking_distance = 0.5  # meters



    def lidar_callback(self, scan: LaserScan):
        self.current_scan=scan
    
    def predict_dynamic_obstacles(self, horizon=1.5, dt=0.2):
        """
        Predict future positions of tracked obstacles and update dynamic_map.

        Each obstacle should have:
            - 'centroid': (x, y) in map frame
            - 'velocity': (vx, vy)
            - 'last_seen': timestamp
        """
        if not hasattr(self, 'dynamic_map'):
            self.dynamic_map = np.zeros_like(self.map_data, dtype=np.uint8)
        else:
            self.dynamic_map.fill(0)

        predicted_cells = set()

        for obs in self.tracked_obstacles.values():
            x, y = obs['centroid']
            vx, vy = obs['velocity']


            # Predict positions along the horizon
            for t in np.arange(0, horizon + dt, dt):
                x_pred = x + vx * t
                y_pred = y + vy * t

                # Inflate using a circle around the predicted centroid
                inflation= self.safety_distance+ 0.1
                num_points = max(8, int(2 * math.pi * inflation / self.map_resolution))
                for theta in np.linspace(0, 2 * math.pi, num_points, endpoint=False):
                    xr = x_pred + inflation * math.cos(theta)
                    yr = y_pred + inflation * math.sin(theta)
                    grid = self.world_to_grid(xr, yr)
                    if grid is not None:
                        predicted_cells.add(grid)

        # Fill predicted cells into dynamic map
        for ix, iy in predicted_cells:
            self.dynamic_map[iy, ix] = 100  # mark as occupied


    def update_tracked_obstacles(self, centroids, stamp):
        """
        Update self.tracked_obstacles with new centroids, estimate velocity.
        centroids: list of (x, y) in map frame
        stamp: ROS time of the scan
        """
        max_dist = 0.5  # meters, max distance to match previous obstacle
        new_tracked = {}

        # Keep track of which old IDs have been matched
        matched_ids = set()

        # --- 1. Try to match each centroid to existing obstacles ---
        for cx, cy in centroids:
            best_id = None
            best_dist = float('inf')

            for oid, obs in self.tracked_obstacles.items():
                ox, oy = obs['centroid']
                dist = math.hypot(cx - ox, cy - oy)
                if dist < max_dist and dist < best_dist:
                    best_dist = dist
                    best_id = oid

            if best_id is not None:
                # Matched an existing obstacle → update velocity
                obs = self.tracked_obstacles[best_id]
                dt = (stamp.sec + stamp.nanosec * 1e-9) - (obs['last_seen'].sec + obs['last_seen'].nanosec * 1e-9)
                dt = max(dt, 1e-3)  # avoid division by zero

                vx = (cx - obs['centroid'][0]) / dt
                vy = (cy - obs['centroid'][1]) / dt

                new_tracked[best_id] = {
                    'centroid': (cx, cy),
                    'velocity': (vx, vy),
                    'last_seen': stamp
                }
                matched_ids.add(best_id)

            else:
                # New obstacle → assign new ID
                oid = self.next_obstacle_id
                self.next_obstacle_id += 1
                new_tracked[oid] = {
                    'centroid': (cx, cy),
                    'velocity': (0.0, 0.0),
                    'last_seen': stamp
                }

        # --- 2. Keep unmatched old obstacles for short time ---
        now = stamp.sec + stamp.nanosec * 1e-9
        for oid, obs in self.tracked_obstacles.items():
            if oid in matched_ids:
                continue
            obs_time = obs['last_seen'].sec + obs['last_seen'].nanosec * 1e-9
            if now - obs_time < 1.0:  # keep for 1 second if lost
                new_tracked[oid] = obs

        self.tracked_obstacles = new_tracked


    def compute_centroid(self, cluster):
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    def cluster_points(self, points, cluster_dist=0.25):
        """
        Simple distance-based clustering.
        points: List[(x, y)] in map frame
        cluster_dist: max distance between points in same cluster (meters)
        """
        clusters = []
        used = [False] * len(points)

        for i, (x_i, y_i) in enumerate(points):
            if used[i]:
                continue

            cluster = [(x_i, y_i)]
            used[i] = True

            changed = True
            while changed:
                changed = False
                for j, (x_j, y_j) in enumerate(points):
                    if used[j]:
                        continue
                    for (cx, cy) in cluster:
                        if math.hypot(x_j - cx, y_j - cy) <= cluster_dist:
                            cluster.append((x_j, y_j))
                            used[j] = True
                            changed = True
                            break

            clusters.append(cluster)

        return clusters

    # ---------------- Scan handling ----------------
    def scan_callback(self):
        if not self.map_received or not hasattr(self, 'current_scan'):
            return

        if not self.tf_buffer.can_transform('map', self.current_scan.header.frame_id,
                                            self.current_scan.header.stamp,
                                            timeout=Duration(seconds=1.0)):
            self.get_logger().warn(f"Transform from {self.current_scan.header.frame_id} to map not available yet")
            return

        dynamic_cells = []
        points_map = []

        max_dyn_range = 2.0  # maximum distance to consider dynamic obstacles (meters)

        for i, r in enumerate(self.current_scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < self.current_scan.range_min or r > self.current_scan.range_max:
                continue
            if r > max_dyn_range:  # ignore points beyond max dynamic range
                continue

            angle = self.current_scan.angle_min + i * self.current_scan.angle_increment
            x_l = r * math.cos(angle)
            y_l = r * math.sin(angle)

            point_lidar = PointStamped()
            point_lidar.header.frame_id = self.current_scan.header.frame_id
            point_lidar.header.stamp = self.current_scan.header.stamp
            point_lidar.point.x = x_l
            point_lidar.point.y = y_l
            point_lidar.point.z = 0.0

            try:
                point_map = self.tf_buffer.transform(point_lidar, 'map', timeout=Duration(seconds=0.1))
            except Exception as e:
                self.get_logger().error(f"TF transform failed: {e}")
                continue

            points_map.append((point_map.point.x, point_map.point.y))


            grid = self.world_to_grid(point_map.point.x, point_map.point.y)
            if grid is None:
                continue
            ix, iy = grid

            # Only mark if the static map cell is free
            if self.map_data[iy, ix] < self.obstacle_threshold:
                dynamic_cells.append((ix, iy))

        #self.update_dynamic_map(dynamic_cells)

        # ---- CLUSTER POINTS ----
        clusters = self.cluster_points(points_map, cluster_dist=0.25)

        centroids = []
        for cluster in clusters:
            if len(cluster) < 5:
                continue  # reject noise
            centroids.append(self.compute_centroid(cluster))
        self.update_tracked_obstacles(centroids, self.current_scan.header.stamp)

        self.predict_dynamic_obstacles(horizon=1.5, dt=0.2)


        # ---------------- Replan hysteresis logic ----------------
        if self.dynamic_obstacle_blocks_path():
            self.block_counter += 1
            self.get_logger().debug(
                f"Dynamic obstacle on path ({self.block_counter}/"
                f"{self.block_counter_threshold})"
            )
        else:
            self.block_counter = 0

        # Trigger replanning only after persistent blockage
        if self.block_counter >= self.block_counter_threshold:
            self.get_logger().warn("Persistent dynamic obstacle → replanning")
            self.block_counter = 0

            self.path = []  # invalidate current path
            if self.current_goal:
                self.goal_callback(self.current_goal)
    

    def dynamic_obstacle_blocks_path(self) -> bool:
       #Returns True if any dynamic obstacle lies on the remaining path.

        if not self.path or not hasattr(self, 'dynamic_map'):
            return False

        # Only check future waypoints
        for wx, wy in self.path[self.current_waypoint_index:]:
            grid = self.world_to_grid(wx, wy)
            if grid is None:
                continue
            ix, iy = grid

            # Dynamic obstacle present
            if self.dynamic_map[iy, ix] > 0:
                return True

        return False


    def update_dynamic_map(self, cells: List[GridIndex]):
        """Update the dynamic map with current laser scan points."""
        if not hasattr(self, 'dynamic_map'):
            self.dynamic_map = np.zeros_like(self.map_data, dtype=np.uint8)

        # Reset dynamic map every scan
        self.dynamic_map.fill(0)

        for ix, iy in cells:
            self.dynamic_map[iy, ix] = 100  # mark as occupied

        # Inflate dynamic obstacles with smaller radius than static map
        dyn_inflation_cells = max(4, int(self.safety_distance / self.map_resolution))
        self.inflate_dynamic_obstacles(dyn_inflation_cells)


    def inflate_dynamic_obstacles(self, inflation_radius_cells: int):
        """Inflate dynamic obstacles by a small radius."""
        if inflation_radius_cells <= 0:
            return

        inflated = np.copy(self.dynamic_map)
        ys, xs = np.where(self.dynamic_map > 0)

        for cy, cx in zip(ys, xs):
            for dy in range(-inflation_radius_cells, inflation_radius_cells + 1):
                for dx in range(-inflation_radius_cells, inflation_radius_cells + 1):
                    nx = cx + dx
                    ny = cy + dy
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        if math.hypot(dx, dy) <= inflation_radius_cells:
                            inflated[ny, nx] = 100

        self.dynamic_map = inflated



    # ---------------- Map handling ----------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (
            msg.info.origin.position.x,
            msg.info.origin.position.y
        )
        self.dynamic_map = np.zeros(
            (self.map_height, self.map_width),
            dtype=np.uint8
        )


        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width)
        )

        self.inflation_radius_cells = int(
            math.ceil(self.safety_distance / self.map_resolution)
        )

        self.inflate_obstacles()
        self.map_received = True

        self.get_logger().info(
            f"Map received ({self.map_width}x{self.map_height}), "
            f"inflation radius = {self.inflation_radius_cells} cells"
        )

    def inflate_obstacles(self):
        """Inflate obstacles by safety distance."""
        inflated = np.copy(self.map_data)

        obstacle_indices = np.argwhere(
            (self.map_data >= self.obstacle_threshold) | (self.map_data == -1)
        )

        for cy, cx in obstacle_indices:
            for dy in range(-self.inflation_radius_cells, self.inflation_radius_cells + 1):
                for dx in range(-self.inflation_radius_cells, self.inflation_radius_cells + 1):
                    nx = cx + dx
                    ny = cy + dy
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        if math.hypot(dx, dy) <= self.inflation_radius_cells:
                            inflated[ny, nx] = self.obstacle_threshold

        self.inflated_map = inflated

    # ---------------- Coordinate helpers ----------------
    def world_to_grid(self, x: float, y: float) -> Optional[GridIndex]:
        ox, oy = self.map_origin
        ix = int((x - ox) / self.map_resolution)
        iy = int((y - oy) / self.map_resolution)
        if 0 <= ix < self.map_width and 0 <= iy < self.map_height:
            return ix, iy
        return None

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        ox, oy = self.map_origin
        return (
            ox + (ix + 0.5) * self.map_resolution,
            oy + (iy + 0.5) * self.map_resolution
        )

    def is_free(self, ix: int, iy: int) -> bool:
        if not self.map_received:
            return False
        if ix < 0 or iy < 0 or ix >= self.map_width or iy >= self.map_height:
            return False

        static_free = self.inflated_map[iy, ix] < self.obstacle_threshold
        dynamic_free =  self.dynamic_map[iy, ix] == 0

        return static_free and dynamic_free

 

    def diagonal_allowed(self, cx, cy, nx, ny) -> bool:
        dx = nx - cx
        dy = ny - cy
        if abs(dx) == 1 and abs(dy) == 1:
            return self.is_free(cx + dx, cy) and self.is_free(cx, cy + dy)
        return True

    # ---------------- Robot pose ----------------
    def get_robot_grid_index(self) -> Optional[GridIndex]:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.base_frame_id, Time(),
                timeout=Duration(seconds=0.5)
            )
            return self.world_to_grid(
                tf.transform.translation.x,
                tf.transform.translation.y
            )
        except Exception:
            return None

    # ---------------- Goal handling ----------------
    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info("New goal received or replanning initiated.")
        self.current_goal = msg
        if not self.map_received:
            return

        if msg.header.frame_id == 'map':
            goal = msg
        else:
            try:
                goal = self.tf_buffer.transform(msg, 'map')
            except Exception as e:
                self.get_logger().error(
                    f"TF transform failed (from {msg.header.frame_id} to map): {e}"
                )
                return



        start = self.get_robot_grid_index()
        goal_idx = self.world_to_grid(
            goal.pose.position.x,
            goal.pose.position.y
        )

        if start is None or goal_idx is None:
            return

        # check to see if goal is reachable
        if not self.is_free(goal_idx[0], goal_idx[1]):
            self.get_logger().info("Goal is in an obstacle or unreachable.")
            return
            
        cell_path = self.a_star(start, goal_idx)
        self.path = [self.grid_to_world(ix, iy) for ix, iy in cell_path]
        self.get_logger().info("Path planned")
        self.current_waypoint_index = 0

    # ---------------- A* ----------------
    def a_star(self, start: GridIndex, goal: GridIndex) -> List[GridIndex]:
        self.get_logger().info(f"Starting A* from {start} to {goal}")
        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        neighbors = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

        open_set = [(0, start)]
        came_from = {}
        g = {start: 0}
        closed = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            closed.add(current)

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                if not self.is_free(nx, ny):
                    continue
                if not self.diagonal_allowed(current[0], current[1], nx, ny):
                    continue

                neighbor = (nx, ny)
                tentative = g[current] + math.hypot(dx, dy)
                if neighbor in closed and tentative >= g.get(neighbor, float('inf')):
                    continue

                if tentative < g.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g[neighbor] = tentative
                    heapq.heappush(open_set, (tentative + h(neighbor, goal), neighbor))

        return []

    # ---------------- Control ----------------
    def control_loop(self):
        if not self.path:
            return

        try:
            tf = self.tf_buffer.lookup_transform('map', self.base_frame_id, Time())
        except Exception as e:
            self.get_logger().info("TF lookup failed in control loop.")
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = self.quat_to_yaw(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w
        )
        # If finished
        if self.current_waypoint_index >= len(self.path):
            self.path = []
            if self.use_twist_stamped:
                self.cmd_pub.publish(TwistStamped())  # stop
            else:
                self.cmd_pub.publish(Twist())  # stop
            
            self.get_logger().info("Reached goal.")
            return

        wx, wy = self.path[self.current_waypoint_index]
        dx, dy = wx - x, wy - y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - yaw)


        if dist < self.waypoint_dist:
            self.current_waypoint_index += 1
            return

        # If heading error large, rotate in place
        angular_z = 0.0
        linear_x = 0.0
        if abs(angle_error) > math.radians(20.0):
            angular_z = self.angular_speed * (1.0 if angle_error > 0 else -1.0)
            linear_x = 0.0
        else:
            # proportional angular velocity and constant linear
            angular_z = max(-self.angular_speed, min(self.angular_speed, 2.0 * angle_error))
            linear_x = self.linear_speed
        
        if self.use_twist_stamped:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.twist.linear.x = linear_x
            twist_stamped.twist.angular.z = angular_z
            self.cmd_pub.publish(twist_stamped)
        else:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.cmd_pub.publish(twist)
        # Decay dynamic map over time
        if hasattr(self, 'dynamic_map'):
            self.dynamic_map = (self.dynamic_map * 0.9).astype(np.uint8)
            self.dynamic_map[self.dynamic_map < 5] = 0



    # ---------------- Utilities ----------------
    @staticmethod
    def quat_to_yaw(x, y, z, w):
        return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    @staticmethod
    def normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = GraphNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
