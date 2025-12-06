# Assignment three 
0. When running the project multiple times, nodes quite often get stuck. So run this to kill leftover nodes:
```
pkill -9 -f ros
```

1. Navigate to ROS2 workspace:
```
cd ~/ros2_ws
```

2. Build:
```
 colcon build --packages-select assignment_three_pkg --symlink-install
```
3. Source the installation
```
source install/setup.bash
```
4. Run slam_toolbox in a different terminal (only if you want to create a new map which we are currently not doing):
```
ros2 launch slam_toolbox online_async_launch.py
```
5. Run the simulation:
```
ros2 launch assignment_three_pkg robot_launch_full.py
```
6. Set the initial position (needed for AMCL)
```
 ros2 topic pub -r 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]" -1
```
7. Publish a goal:

for example:
```
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 1.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
" -1
```
8. save the maps for debugging using:
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/lara/ros2_ws/src/assignment_three_pkg/slam_maps'}}"

```

9. To start navigation, AMCL needs an initial pose. It can either be set in RVIZ or manually by publishing on the initial pose topic: 
```
ros2 topic pub -r 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position:
      x: 6.36
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]" -1
```


## Start RViz Visualization
1. Start RViz using the command `rviz2`
2. Under Global Options, set Fixed Frame = map
3. Add the topic /map. Under Topic, set Durability Policy = Transient Local -> the map will appear
4. Add more topics, e.g. /scan to vizualize the LaserScan, /initialpose or /goal_pose

-> If there are new initial poses or goal poses are published, it will show up



## Summary of what the project does:
Exploration phase:
-reactive controller lets robot drive around environment randomly
-slam_toolbox builds a map while robot drives around
-as soon as a goal pose is published, A* plans a route there based on the map. 

## TODO: 
-Static Map Completed:
A full static map of the environment has been successfully generated using SLAM. The resulting map, named map_best_run, can be saved, reloaded, and used independently of the SLAM process.

Localization Progress (AMCL):
Monte Carlo Localization is partially working. AMCL receives the map_best_run map and laser data, but there are still timing and TF-related issues causing inconsistent initialization and message filtering problems.

Navigation Not Functional Yet:
The navigation stack (Nav2) cannot run reliably at this stage because localization is not fully stable. Path planning and controller functions do not work yet due to the remaining bugs in AMCL and TF.

Dynamic Obstacles Not Implemented:
No dynamic obstacle layers or real-time obstacle detection have been added yet. Navigation currently depends only on the static map_best_run map.