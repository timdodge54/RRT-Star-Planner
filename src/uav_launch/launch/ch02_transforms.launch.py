from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Create the package directory
    uav_launch_dir = get_package_share_directory('uav_launch')

    # Define the urdf file for visualizing the uav
    urdf_file_name = 'fixed_wing_uav.urdf'
    urdf = os.path.join(
        get_package_share_directory('uav_launch'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        ################ Dynamics and kinematics #####################################
        Node(
            package='uav_launch',
            executable='transforms',
            name='transforms',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf],
            parameters=[{
                'use_sim_time': False
            }]),

        ################# Tools for Interacting with the Sim #########################
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(uav_launch_dir, 'ch02_transforms.rviz')]],
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])