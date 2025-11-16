from setuptools import find_packages, setup

package_name = 'assignment_three_pkg'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world_big.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world_test.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/tilde.obj']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/tilde_big.obj']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))

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

        ],
    },
)