from setuptools import find_packages, setup

package_name = 'ros2_traj_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/trajectory_launch.py',
            'launch/velocity_filter_launch.py',
            'launch/complete_system_launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/trajectory_params.yaml',
            'config/velocity_filter_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='venugopal reddy kollan',
    maintainer_email='venugopal.reddy.kollan@gmail.com',
    description='Trajectory planning using polynomial interpolation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "trajectory_planner_node = ros2_traj_pkg.trajectory_planner_node:main",
            "velocity_filter_node = ros2_traj_pkg.filtervelocity_subscriber_node:main",
        ],
    },
)
