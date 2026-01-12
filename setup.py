from setuptools import find_packages, setup

package_name = 'assignment_three_pkg'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch_full.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_map_building.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/nav2_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/assignment_three_world.wbt']))

data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot_webots.urdf',
    'resource/ros2control.yml',
    'resource/nav2_params.yaml'
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/turtlebot3_burger_example.wbt', 'worlds/.turtlebot3_burger_example.wbproj',
    'worlds/assignment_three_world.wbt', 'worlds/.assignment_three_world.wbproj',

]))

data_files.append(('share/' + package_name + '/slam_maps', ['slam_maps/my_map.yaml']))
data_files.append(('share/' + package_name + '/slam_maps', ['slam_maps/my_map.pgm']))
data_files.append(('share/' + package_name + '/slam_maps', ['slam_maps/my_map_game.yaml']))
data_files.append(('share/' + package_name + '/slam_maps', ['slam_maps/my_map_game.pgm']))



data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_driver = assignment_three_pkg.turtlebot_driver:main',
            'robot_controller = assignment_three_pkg.robot_controller:main',
            'odom_calculator = assignment_three_pkg.odom_calculator:main',
            'occupancy_grid = assignment_three_pkg.occupancy_grid_node:main',
            'exploring_node = assignment_three_pkg.exploring_node_improved:main',
            'navigating_node = assignment_three_pkg.graph_navigator:main',
            'set_initial_pose = assignment_three_pkg.set_initial_pose:main',
        ],
    },
)