from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    # robot_description 包路径
    pkg_path = get_package_share_directory('robot_description')

    # ros_gz_sim 包路径
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    # =============================
    # Xacro -> URDF
    # =============================

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'robot.urdf.xacro'
    )

    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {
        'robot_description': robot_description_config.toxml()
    }

    # =============================
    # Robot State Publisher
    # =============================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # =============================
    # Gazebo World
    # =============================

    world_file = os.path.join(
        pkg_path,
        'worlds',
        'empty.sdf'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_path,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r ' + world_file
        }.items()
    )

    # =============================
    # Spawn Robot
    # =============================



    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'semantic_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/empty_world/model/semantic_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/world/empty_world/model/semantic_robot/joint_state', '/joint_states'),
            ('/scan', '/scan_raw'),
        ],
        output='screen'
    )

    # =============================
    # Odom TF Broadcaster (odom -> base_footprint)
    # =============================

    odom_tf_broadcaster = Node(
        package='robot_control',
        executable='odom_tf_broadcaster',
        output='screen'
    )

    # =============================
    # Scan Frame Republisher (/scan_raw -> /scan, frame_id=laser_link)
    # =============================

    scan_frame_republisher = Node(
        package='robot_control',
        executable='scan_frame_republisher',
        output='screen'
    )

    # =============================
    # Launch
    # =============================

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        odom_tf_broadcaster,
        scan_frame_republisher
    ])