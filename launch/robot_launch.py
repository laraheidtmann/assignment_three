#!/usr/bin/env python

import os
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('my_reactive_robot')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world_big.wbt')
    )

    leader_driver = WebotsController(
        robot_name='leader',
        namespace= 'leader',
        parameters=[
            { 'robot_description': robot_description_path,
            'robot_name': 'leader' }
        ],
    )
    follower_driver = WebotsController(
    robot_name='follower',
    namespace= 'follower',
    parameters=[
        { 'robot_description': robot_description_path,
        'robot_name': 'follower' }
    ],
    )

    leader = Node(
        package='my_reactive_robot',
        name='leader',
        executable='leader_controller',
        namespace='leader'
    )
    follower = Node(
        package='my_reactive_robot',
        name='follower',
        executable='follower_controller',
        namespace='follower'
    )

    return LaunchDescription([
        webots,
        leader_driver,
        leader,
        follower_driver,
        follower,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])