import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory('nav_farm_walker')
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    use_mapviz = LaunchConfiguration('use_mapviz')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start mapviz')

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot_localization.launch.py'))
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",  # Set use_sim_time to False for real-life scenario
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz)
    )

    urdf_file = os.path.join(gps_wpf_dir, 'urdf', 'walker.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    # robot state publisher launch
    ld.add_action(start_robot_state_publisher_cmd)

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # viz launch
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(mapviz_cmd)

    return ld