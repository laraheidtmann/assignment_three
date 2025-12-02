# Assignment three 

1. Navigate to ROS2 workspace:
```
cd ~/ros2_ws
```

2. Build:
```
colcon build
```
3. Source the installation
```
source install/setup.bash
```
4. Run slam_toolbox in a different terminal:
```
ros2 launch slam_toolbox online_async_launch.py
```
5. Run the simulation:
```
ros2 launch assignment_three_pkg robot_launch_full.py
```
6. After ~5min publish a goal:
```
for example:

ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: -2.9
    y: -3.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
" -1
```
7. save the maps for debugging using:
```
 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/mnt/c/Users/joerg/my_slam_map/my_map'}}"
