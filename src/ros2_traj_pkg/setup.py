# If you wish not to use the generate_parameter_library
# Comment the import generate_parameter_module and module declarations
from setuptools import find_packages, setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = "ros2_traj_pkg"

generate_parameter_module(
    "trajectory_planner_parameters",  # python module name for parameter library
    "config/publisher_params_manager.yaml", # path to input yaml file
)

generate_parameter_module(
    "velocity_filter_parameters", # python module name for parameter library
    "config/subscriber_params_manager.yaml", # path to input yaml file
)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/trajectory_launch.py",
                "launch/velocity_filter_launch.py",
                "launch/complete_system_launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/publisher_params_manager.yaml",
                "config/subscriber_params_manager.yaml",
                "config/trajectory_params.yaml",
                "config/velocity_filter_params.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="venugopal reddy kollan",
    maintainer_email="venugopal.reddy.kollan@gmail.com",
    description="Trajectory planning using polynomial interpolation",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectory_planner_node = ros2_traj_pkg.trajectory_planner_node:main",
            "filtervelocity_subscriber_node = ros2_traj_pkg.filtervelocity_subscriber_node:main",
        ],
    },
)
