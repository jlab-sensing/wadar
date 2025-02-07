import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import launch.actions

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('nav_farm_walker')
    # Path to the parameter file
    params_file = os.path.join(pkg_share, 'config', 'dual_ekf_navsat_params.yaml')

    return LaunchDescription([
        # Declare launch arguments
        launch.actions.DeclareLaunchArgument(
            "output_final_position", default_value="false"
        ),
        launch.actions.DeclareLaunchArgument(
            "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
        ),
        
        # EKF node for odometry (local)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),
        
        # EKF node for map frame (global)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),
        
        # Navsat transform node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}],
            remappings=[
                ('imu/data', 'imu/data'),
                ('gps/fix', 'gps/fix'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global'),
            ],
        ),
    ])