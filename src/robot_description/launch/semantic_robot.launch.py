import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    simulation_launch = os.path.join(
        pkg_share,
        'launch',
        'simulation.launch.py'
    )
    nav2_launch = os.path.join(
        pkg_share,
        'launch',
        'nav2.launch.py'
    )
    ekf_params = os.path.join(
        pkg_share,
        'config',
        'ekf.yaml'
    )

    vmware_mode = LaunchConfiguration('vmware_mode')

    declare_vmware_mode = DeclareLaunchArgument(
        'vmware_mode',
        default_value='true',
        description=(
            'Enable software-rendering environment variables used by the '
            'VMware development environment.'
        )
    )

    vmware_environment = [
        SetEnvironmentVariable(
            'QT_QPA_PLATFORM',
            'xcb',
            condition=IfCondition(vmware_mode)
        ),
        SetEnvironmentVariable(
            'LIBGL_DRI3_DISABLE',
            '1',
            condition=IfCondition(vmware_mode)
        ),
        SetEnvironmentVariable(
            'LIBGL_ALWAYS_SOFTWARE',
            '1',
            condition=IfCondition(vmware_mode)
        ),
    ]

    # Stage 1: simulator, robot model, base bridges and LaserScan republisher.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch)
    )

    # Camera and IMU are kept in a separate bridge because the existing
    # simulation.launch.py intentionally only handles the base robot topics.
    camera_imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_imu_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    # Stage 2: state estimation and perception. A short delay gives Gazebo
    # enough time to spawn the robot and start publishing sensor topics.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params],
        output='screen'
    )

    bottle_detector = Node(
        package='robot_control',
        executable='bottle_detector',
        name='bottle_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    delayed_state_estimation_and_perception = TimerAction(
        period=2.0,
        actions=[
            ekf,
            bottle_detector,
        ]
    )

    # Stage 3: localization + navigation stack.
    delayed_nav2 = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch)
            )
        ]
    )

    # Stage 4: task-level semantic mission controller.
    semantic_controller = Node(
        package='robot_control',
        executable='semantic_search_controller',
        name='semantic_search_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    delayed_semantic_controller = TimerAction(
        period=7.0,
        actions=[semantic_controller]
    )

    return LaunchDescription([
        declare_vmware_mode,
        *vmware_environment,
        simulation,
        camera_imu_bridge,
        delayed_state_estimation_and_perception,
        delayed_nav2,
        delayed_semantic_controller,
    ])
