from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wuuy-baby',
    maintainer_email='241547421+wuuy-baby@users.noreply.github.com',
    description=(
        'ROS 2 control, perception, active localization, and semantic mission '
        'nodes for the Semantic Robot project.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = robot_control.odom_tf_broadcaster:main',
            'move_distance = robot_control.move_distance:main',
            'rotate_angle = robot_control.rotate_angle:main',
            'square_motion = robot_control.square_motion:main',
            'go_to_goal = robot_control.go_to_goal:main',
            'go_to_goal_p = robot_control.go_to_goal_p:main',
            'waypoint_follower = robot_control.waypoint_follower:main',
            'navigate_to_pose_simple = robot_control.navigate_to_pose_simple:main',
            'scan_frame_republisher = robot_control.scan_frame_republisher:main',
            'slam_explorer = robot_control.slam_explorer:main',
            'bottle_detector = robot_control.bottle_detector:main',
            'semantic_search_controller = robot_control.semantic_search_controller:main',
        ],
    },
)
