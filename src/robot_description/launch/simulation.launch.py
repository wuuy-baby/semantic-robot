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
    # Scan Frame Republisher (/scan_raw -> /scan, frame_id=laser_link)
    # =============================

    scan_frame_republisher = Node(
        package='robot_control',
        executable='scan_frame_republisher',
        output='screen'
    )

    # =============================
    # IMU sensor frame 静态 TF
    # Gazebo IMU 插件发布的消息 frame_id 为
    #   semantic_robot/imu_link/imu_sensor
    # 但 URDF TF 树中只有 imu_link（imu_joint: base_link -> imu_link）。
    # gazebo.xacro 中 <sensor name="imu_sensor"> 没有 <pose> 元素，
    # 即传感器光心与 imu_link 同原点同朝向（xyz=0, rpy=0）。
    # 这里补一条静态 TF 使 imu 消息 frame_id 在 TF 树中存在。
    # =============================

    imu_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_sensor_static_tf',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'imu_link',
            '--child-frame-id', 'semantic_robot/imu_link/imu_sensor'
        ],
        output='screen'
    )

    # =============================
    # Launch
    # =============================
    # 注意：odom -> base_footprint 由 robot_localization ekf_filter_node 发布
    # （publish_tf: true），此处不再启动旧的 odom_tf_broadcaster，避免 TF 双发布。

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        scan_frame_republisher,
        imu_sensor_tf
    ])