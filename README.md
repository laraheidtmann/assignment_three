# Graph-Based Navigation and Dynamic Obstacle Avoidance in ROS 
This repository contains a ROS 2–based autonomous mobile robot navigation framework developed and evaluated in **Webots** using a **TurtleBot3 Burger** equipped with a **2D LiDAR** sensor. The system supports **SLAM-based mapping**, **graph-based A*** navigation, **dynamic obstacle handling**, and **quantitative performance evaluation**.

The project implements and compares two navigation approaches:
1. Standard Nav2 navigation
2. A custom graph-based global planner with online replanning

Both approaches are evaluated on:
- A **static map** (no dynamic obstacles)
- A **dynamic environment** (with moving obstacles)

This enables a controlled and fair comparison of navigation performance under different environmental assumptions.



## 🚀 Project Overview
The system is structured around two operational phases:
1. Mapping & Exploration Phase
    - Autonomous exploration of an unknown indoor environment
    - **Cartographer SLAM** used to build a 2D occupancy grid map
    - Smooth, conservative motion to improve SLAM consistency
    - Generated maps are stored and reused for navigation
2. Navigation Phase
    - **Adaptive Monte Carlo Localization (AMCL)** for pose estimation
    - Interchangeable navigation backends:
      - Nav2 stack
      - Custom graph-based navigation
    - Quantitative evaluation via a shared metric logging pipeline



## 🧠 System Architecture
**Core technologies**
- ROS 2 Humble
- Webots simulator
- TurtleBot3 Burger
- 2D LiDAR (sole perception sensor)
- Cartographer SLAM
- AMCL localization

**Design focus**
- Explicit comparison between reactive local planning and global replanning
- Minimal sensing modalities
- Deterministic and reproducible experiments



## 📁 Repository Structure
```
├── assignment_three_pkg
│   ├── __init__.py
│   ├── exploring_node.py
│   ├── exploring_node_improved.py
│   ├── graph_navigator_advanced.py
│   ├── graph_navigator_simple.py
│   ├── metric_logger.py
│   ├── odom_calculator.py
│   ├── set_initial_pose.py
│   └── turtlebot_driver.py
│
├── controllers
│   └── dynamic_obstacles_controller
│       └── dynamic_obstacles_controller.py
│
├── launch
│   ├── launch_nav2.py
│   ├── launch_nav_custom.py
│   └── launch_slam.py
│
├── rqt_graph
│   ├── rosgraph.png
│   ├── rosgraph_exploration_mode.png
│   └── rosgraph_navigation_mode.png
│
├── slam_maps
│   ├── example_map.pgm
│   ├── example_map.png
│   ├── example_map.yaml
│   ├── game_map.pgm
│   ├── game_map.png
│   └── game_map.yaml
│
├── worlds
│   ├── game_world.wbt
│   ├── game_world_static.wbt
│   └── turtlebot3_burger_example.wbt
│
├── .gitignore
├── LICENSE
├── README.md
├── package.xml
├── setup.cfg
└── setup.py
```



## 🧩 Node Descriptions
**`graph_navigator_simple.py`:** Custom graph-based navigation node using A* on an inflated occupancy grid. Detects dynamic obstacles by projecting LiDAR data into the map and performs hysteresis-based global replanning. Executes motion via waypoint tracking.

**`graph_navigator_advanced.py`:** Experimental extension of the graph-based planner for testing alternative planning or optimization strategies. Not used in the final evaluation.

**`exploring_node.py`:** Simple reactive exploration controller used for initial SLAM testing.

**`exploring_node_improved.py`:** Improved exploration node optimized for SLAM quality, featuring smooth motion, stuck detection, and controlled obstacle avoidance.

**`metric_logger.py`:** Logs navigation performance metrics for both Nav2 and custom navigation, enabling direct quantitative comparison.

**`odom_calculator.py`:** Computes traveled distance from odometry data for evaluation purposes.

**`set_initial_pose.py`:** Publishes a fixed initial pose to AMCL to ensure consistent experiment initialization.

**`turtlebot_driver.py`:** Interfaces ROS 2 velocity commands with the Webots TurtleBot3 robot.

**`dynamic_obstacles_controller.py`:** Webots supervisor controller that spawns and moves dynamic obstacles during navigation experiments.



## 🛠️ Build & Run Instructions
This project is developed and tested using ROS 2 in a workspace named `ros2_ws`.

### 1. Important: Clean Up Old ROS Processes
Before building or launching the system, ensure that no leftover ROS nodes are running:
```bash
pkill -9 -f ros
```
This prevents conflicts caused by orphaned nodes or stale DDS participants.

### 2. Build the Workspace
1. Navigate to the ROS 2 workspace:
   ```bash
   cd ~/ros2_ws
   ```
2. Build the package:
   ```bash
   colcon build --packages-select assignment_three_pkg --symlink-install
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```
   💡 You must source the workspace in every new terminal before running the project.

### 3. SLAM & Map Building
To generate a map using autonomous exploration and Cartographer SLAM:
```bash
ros2 launch assignment_three_pkg launch_slam.py
```

Once exploration is complete, save the generated map using:
```bash
ros2 run nav2_map_server map_saver_cli -f /home/<user>/ros2_ws/src/assignment_three_pkg/slam_maps/<map_name>
```

This will create the corresponding `.pgm` and `.yaml` files in the slam_maps directory.

### 4. Navigation (Static or Dynamic Environments)
After a map has been created and saved, navigation can be launched using one of the following modes. The project supports two interchangeable navigation pipelines, selectable via launch files. Both pipelines use the same map, robot model, sensors, and simulator, ensuring fair comparison.

#### Mode 1: Nav2-Based Navigation (Baseline)
Launch the standard Nav2 navigation stack:
```bash
ros2 launch assignment_three_pkg launch_nav2.py
```

**Architecture**
- Localization: AMCL
- Global planning: NavFn (grid-based Dijkstra)
- Local planning: DWB (Dynamic Window Approach)
- Obstacle handling: Costmap-based inflation + reactive control
- Recovery behaviors: spin, backup, wait

**Characteristics**
- Continuous reactive avoidance
- Short-horizon optimization
- Widely used industry-standard approach

Dynamic obstacles are handled implicitly via LiDAR updates to the costmaps; no explicit obstacle tracking or prediction is performed.

#### Mode 2: Custom Graph-Based Navigation
Launch the custom A*-based navigation system:
```bash
ros2 launch assignment_three_pkg launch_nav_custom.py
```

**Architecture**
- Localization: AMCL (shared with Nav2)
- Planning: Custom A* over an inflated occupancy grid
- Control: Waypoint-based velocity control
- Replanning: Global replanning with hysteresis

**Global Planning**
- Occupancy grid converted into an 8-connected graph
- Obstacles inflated by a configurable safety distance
- Diagonal motion allowed with corner-cutting prevention
- Euclidean-distance heuristic (A*)

**Dynamic Obstacle Handling**
- LiDAR points are:
  - Transformed into the map frame
  - Projected into grid cells
  - Compared against the static map
- Points that intrude into free space are treated as dynamic obstacles
- A separate dynamic occupancy layer is maintained
- Dynamic obstacles are inflated more conservatively than static ones

**Replanning Strategy**
- Planned paths are continuously checked for blockage
- Hysteresis-based replanning:
  - Replanning is triggered only after repeated blockage detections
  - Prevents oscillations due to sensor noise
- Dynamic obstacle information decays over time

**Motion Execution**
- Waypoint-by-waypoint tracking
- Heading correction prioritized over forward motion
- Simple, stable control law suitable for frequent replanning



## 🔀 Usage
### Publishing Navigation Goals
Navigation goals can be sent to the system in two different ways:

#### Option 1: RViz2
1. Launch the navigation system (Nav2 or custom navigation).
2. Open **RViz2** (for Nav2, it opens automatically).
3. Configure RViz2 (only for custom navigation):
   - Set the fixed frame to `map`
   - Add the topic `/map`. Under Topic, set `Durability Policy = Transient Local`
   - Add more topics, e.g. `/scan` to vizualize the LaserScan, `/initialpose` or `/goal_pose`
4. Select the **“2D Goal Pose”** tool.
5. Click on the map to set the goal position and orientation.

This method was used for most experiments and allows intuitive goal placement.

#### Option 2: Command Line
Goals can also be published manually via the command line:
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 1.0
    y: 4.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
" -1
```



## 🧪 Evaluation
Navigation performance was evaluated through a sequence-based goal experiment. A fixed set of six goals was defined on the map, and the robot was commanded to navigate to each goal sequentially in a single run. The same goal sequence, start pose, and map were used for all experiments to ensure fair comparison.

### Static Map
In the static scenario, no dynamic obstacles were present.

**Purpose**
- Establish a baseline for navigation performance
- Evaluate path efficiency and execution stability
- Verify correct operation of both navigation approaches

Both Nav2 and the custom graph-based navigation were tested under identical static conditions.

**Map:** `game_world_static.wbt`

### Dynamic Environment (Main Focus)
In the dynamic scenario, moving obstacles were introduced that frequently intersected the robot’s planned path.

**Purpose**
- Evaluate robustness to environmental changes
- Compare recallable local avoidance (Nav2) with explicit global replanning (custom planner)
- Analyze replanning frequency and stability

This scenario represents the main focus of the evaluation.

**Map:** `game_world.wbt`

### Metrics
For each run, the following metrics were logged automatically:
- Success or failure of the full six-goal sequence
- Total navigation time
- Traveled distance
- Number of replans
- Minimum obstacle clearance

Each navigation method was evaluated on both static and dynamic maps, resulting in four experimental configurations. All experiments are evaluated using a unified logging node: `metric_logger.py`.

Metrics are stored automatically in:
```bash
~/nav_metrics/metrics.csv
```



## ⚠️ Limitations
- Dynamic obstacles are treated conservatively (no velocity estimation)
- Grid-based planning may become computationally expensive at high resolutions
- No semantic classification of obstacles
- Assumes reasonably accurate localization
