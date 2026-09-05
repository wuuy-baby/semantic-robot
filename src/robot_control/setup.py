from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ast',
    maintainer_email='ast@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
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
        ],
    },
)
