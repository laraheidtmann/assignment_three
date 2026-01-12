import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('assignment_three_pkg')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    obstacles_controller_script = os.path.join(
        package_dir, 'controllers', 'dynamic_obstacles_controller', 'dynamic_obstacles_controller.py'
    )
    obstacles_supervisor_controller = ExecuteProcess(
        output='screen',
        cmd=['python3', obstacles_controller_script],
        additional_env={
            'WEBOTS_CONTROLLER_URL': controller_url_prefix('1234') + 'ObstaclesSupervisor',
            'WEBOTS_HOME': get_package_prefix('webots_ros2_driver'),
        },
        respawn=True,
    )
    obstacles_supervisor_controller_delayed = TimerAction(
        period=2.0,
        actions=[obstacles_supervisor_controller],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # Navigation
    navigation_nodes = []
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    nav2_map = os.path.join(package_dir, 'slam_maps', 'my_map_game.yaml')
    nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')
    if 'turtlebot3_navigation2' in get_packages_with_prefixes():
        turtlebot_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('params_file', nav2_params),
                ('use_sim_time', use_sim_time),
            ])
        navigation_nodes.append(turtlebot_navigation)


    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=navigation_nodes + ros_control_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='assignment_three_world.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        obstacles_supervisor_controller_delayed,

        robot_state_publisher,
        footprint_publisher,

        turtlebot_driver,
        waiting_nodes,

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