from setuptools import setup
import os
from glob import glob

package_name = 'uav_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf'))
    ],
    install_requires=['setuptools', 'shapely'],
    zip_safe=True,
    maintainer='greg',
    maintainer_email='greg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transforms = uav_launch.transforms:main',
            'ch02_reconfigure_state = uav_launch.ch02_reconfigure_state:main',
            'ch03_kinematic_mav = uav_launch.ch03_kinematic_mav_node:main',
            'ch04_dynamic_mav = uav_launch.ch04_dynamic_mav_node:main',
            'ch04_wind = uav_launch.ch04_wind_node:main',
            'ch05_trim = uav_launch.ch05_trim_traj_node:main',
            'ch06_autopilot = uav_launch.ch06_autopilot_node:main',
            'ch06_autopilot_dynamics = uav_launch.ch06_autopilot_dynamics_node:main',
            'ch07_sensors = uav_launch.ch07_sensors_node:main',
            'ch07_sensors_auto_dyn = uav_launch.ch07_sensors_autopilot_dynamics_node:main',
            'ch10_path_follower = uav_launch.ch10_path_follower_node:main',
            'ch11_path_manager = uav_launch.ch11_path_manager_node:main',
            'ch11_waypoints_publisher = uav_launch.ch11_waypoints_publisher:main',
            'ch11_waypoints_relay = uav_launch.ch11_waypoints_relay:main',
            'ch12_path_planner = uav_launch.ch12_path_planner_node:main',
            'final_path_planner = uav_launch.final_path_planner_node:main',
            'ch12_world_manager = uav_launch.ch12_world_manager_node:main',
            'diagnotic_aggregator = uav_launch.diagnostics_aggregator_node:main',
            'uav_plotter = uav_launch.uav_plotting_node:main',
            'gps_denied_monitor = uav_launch.gps_denied_region_monitor:main',
        ],
    },
)
