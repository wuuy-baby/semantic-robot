import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('robot_description')

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg_share, 'maps', 'simple_map.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_yaml,
                'use_sim_time': True
            }
        ]
    )

    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom',
        output='screen',
        arguments=[
            '--x', '0.6',
            '--y', '0.6',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ]
            }
        ]
    )

    return LaunchDescription([
        map_server,
        static_map_odom,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,

        TimerAction(
            period=2.0,
            actions=[lifecycle_manager]
        ),
    ])