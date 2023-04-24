from launch import LaunchDescription
from launch_ros.actions import Node
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

    # Flag for enabling/disabling use of simulation time instead of wall clock
    use_sim_time = True

    return LaunchDescription([

        ################ Dynamics and kinematics #####################################
        Node(
            package='uav_launch',
            executable='transforms',
            name='transforms',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf],
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        # Dynamics combined with autopilot
        Node(
            package='uav_launch',
            executable='ch06_autopilot_dynamics',
            name='autopilot_dynamic_mav',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='uav_launch',
            executable='ch04_wind',
            name='wind',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='uav_launch',
            executable='ch07_sensors',
            name="sensors",
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        ################# UAV Control Software #######################################
        # State Estimator
        Node(
            package='topic_tools',
            executable='relay',
            name="state_estimator_relay",
            parameters=[
                {"input_topic": "uav_state"},
                {"output_topic": "uav_state_estimate"},
                {"use_sim_time": use_sim_time}
            ]
        ),

        # Path follower
        Node(
            package='uav_launch',
            executable='ch10_path_follower',
            name='path_follower',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        # Path follower
        Node(
            package='uav_launch',
            executable='ch11_path_manager',
            name='path_manager',
            #emulate_tty=True,
            #output="screen",
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        ################# Tools for Interacting with the Sim #########################
        # Temporary waypoint relay
        Node(
            package='uav_launch',
            executable='ch11_waypoints_relay',
            name='waypoint_relay',
            remappings=[
            ('waypoints_in', '/wp_panel/waypoint_cmd'),
            ('waypoints_out', '/waypoints'),
         ],
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(uav_launch_dir, 'ch11_path_creation.rviz')]],
            parameters=[{
                'use_sim_time': False
            }]
        ),
        Node(
            package='rqt_robot_monitor',
            executable='rqt_robot_monitor',
            name='rqt_robot_monitor',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        Node(
            package='uav_launch',
            executable='diagnotic_aggregator',
            name='diagnotic_aggregator',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='uav_launch',
            executable='uav_plotter',
            name='uav_plotter',
            parameters=[{
                'use_sim_time': True,
                't_horizon': 100.,
                'plot_sensors': True
            }]
        )
    ])
