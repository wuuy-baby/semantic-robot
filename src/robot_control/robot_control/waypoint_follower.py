import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )


        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/robot_path',
            10
        )   
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 多个目标点
        self.waypoints = [
            (2.0, 1.0),
            (3.0, 3.0),
            (1.0, 4.0),
            (0.0, 2.0)
        ]

        # 当前目标点索引
        self.current_index = 0

        # P控制参数
        self.kp_linear = 0.5
        self.kp_angular = 1.5

        # 最大速度
        self.max_linear_speed = 0.4
        self.max_angular_speed = 1.0

        # 到目标多近算到达
        self.distance_tolerance = 0.10

        self.finished = False

        self.get_logger().info(
            f'WaypointFollower started, '
            f'total waypoints: {len(self.waypoints)}'
        )

    def publish_waypoints(self,stamp):

        marker_array = MarkerArray()

        for i, (x, y) in enumerate(self.waypoints): 

            marker = Marker()

            marker.header.frame_id = 'odom'
            marker.header.stamp = stamp

            marker.ns = 'waypoints'
            marker.id = i

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.10

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.20
            marker.scale.y = 0.20
            marker.scale.z = 0.20

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0    

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)    


    def quaternion_to_yaw(self, x, y, z, w):

        siny_cosp = 2.0 * (w * z + x * y)

        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )


    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


    def clamp(self, value, min_value, max_value):

        return max(
            min_value,
            min(value, max_value)
        )


    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


    def odom_callback(self, msg):
        self.publish_waypoints(msg.header.stamp)

        if self.finished:
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        pose = PoseStamped()

        pose.header = msg.header
        pose.header.frame_id = 'odom'

        pose.pose = msg.pose.pose

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)

        self.path_pub.publish(self.path)

        q = msg.pose.pose.orientation

        current_yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        # 当前目标点
        target_x, target_y = self.waypoints[
            self.current_index
        ]

        dx = target_x - current_x
        dy = target_y - current_y

        distance_error = math.sqrt(
            dx * dx + dy * dy
        )

        target_yaw = math.atan2(
            dy,
            dx
        )

        angle_error = self.normalize_angle(
            target_yaw - current_yaw
        )

        # 到达当前 waypoint
        if distance_error < self.distance_tolerance:

            self.stop_robot()

            self.get_logger().info(
                f'Waypoint {self.current_index + 1} reached: '
                f'({target_x:.2f}, {target_y:.2f})'
            )

            self.current_index += 1

            # 所有 waypoint 完成
            if self.current_index >= len(self.waypoints):

                self.finished = True

                self.get_logger().info(
                    'All waypoints reached'
                )

            return

        # P控制
        linear_cmd = (
            self.kp_linear * distance_error
        )

        angular_cmd = (
            self.kp_angular * angle_error
        )

        # 限幅
        linear_cmd = self.clamp(
            linear_cmd,
            0.0,
            self.max_linear_speed
        )

        angular_cmd = self.clamp(
            angular_cmd,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        # 方向差太大先转
        if abs(angle_error) > math.radians(60.0):
            linear_cmd = 0.0

        cmd = Twist()

        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Waypoint={self.current_index + 1}, '
            f'Target=({target_x:.1f},{target_y:.1f}), '
            f'Distance={distance_error:.2f}, '
            f'Angle={math.degrees(angle_error):.2f}, '
            f'v={linear_cmd:.2f}, '
            f'w={angular_cmd:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = WaypointFollower()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()