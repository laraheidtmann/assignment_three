#!/usr/bin/env python

import os
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('assignment_three_pkg')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')


    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world_test.wbt')
    )

    # Webots ROS2 driver for the TurtleBot3 Burger
    turtle_driver = WebotsController(
        robot_name='turtlebot3_burger',
        namespace='turtlebot3_burger',
        parameters=[
         { 'robot_description': robot_description_path,
            'use_sim_time': True}
        ]
    )

    # Your reactive navigation node
    turtlebot_nav = Node(
        package='assignment_three_pkg',
        name='reactive_controller',
        executable='robot_controller',
        namespace='turtlebot3_burger',
        parameters=[{
            'use_sim_time': True}]
    )
    
    odom_calc = Node(
        package='assignment_three_pkg',
        name='odom_calculator',
        executable='odom_calculator',
        namespace='turtlebot3_burger',
        parameters=[{
            'use_sim_time': True}]
    )

    

    return LaunchDescription([
        webots,
        turtle_driver,
        turtlebot_nav,
        odom_calc,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
