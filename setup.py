import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'assignment_three_pkg'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

data_files.append(('share/' + package_name + '/launch', [
    'launch/launch_slam.py',
    'launch/launch_nav_custom.py',
    'launch/launch_nav2.py'
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot_webots.urdf',
    'resource/ros2control.yml',
    'resource/nav2_params.yaml'
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/game_world.wbt',
    'worlds/game_world_static.wbt',
    'worlds/turtlebot3_burger_example.wbt',
]))
data_files.append(('share/' + package_name + '/slam_maps', [
    'slam_maps/game_map.yaml',
    'slam_maps/game_map.pgm',
    'slam_maps/example_map.yaml',
    'slam_maps/example_map.pgm',
]))

data_files.append(('share/' + package_name, ['package.xml']))


def _add_tree(data_files_list, root_dir: str) -> None:
    """Install a directory tree under share/<package_name>/<root_dir> preserving layout."""
    for path in glob(os.path.join(root_dir, '**', '*'), recursive=True):
        if not os.path.isfile(path):
            continue
        install_dir = os.path.join('share', package_name, os.path.dirname(path))
        data_files_list.append((install_dir, [path]))


# Webots expects controllers at <project>/controllers/<controller_name>/...
_add_tree(data_files, 'controllers')

# Some setups resolve the Webots project root as the `worlds/` directory.
# In that case Webots looks for controllers under `worlds/controllers/...`.
_add_tree(data_files, os.path.join('worlds', 'controllers'))

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
            'exploring_node = assignment_three_pkg.exploring_node_improved:main',
            'navigating_node = assignment_three_pkg.graph_navigator_simple:main',
            'set_initial_pose = assignment_three_pkg.set_initial_pose:main',
            'metric_logger = assignment_three_pkg.metric_logger:main',
        ],
    },
)