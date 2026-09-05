import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class GoToGoalP(Node):

    def __init__(self):
        super().__init__('go_to_goal_p')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 参数
        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 1.0)

        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)

        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.0)

        self.declare_parameter('distance_tolerance', 0.10)

        self.target_x = self.get_parameter(
            'target_x'
        ).get_parameter_value().double_value

        self.target_y = self.get_parameter(
            'target_y'
        ).get_parameter_value().double_value

        self.kp_linear = self.get_parameter(
            'kp_linear'
        ).get_parameter_value().double_value

        self.kp_angular = self.get_parameter(
            'kp_angular'
        ).get_parameter_value().double_value

        self.max_linear_speed = self.get_parameter(
            'max_linear_speed'
        ).get_parameter_value().double_value

        self.max_angular_speed = self.get_parameter(
            'max_angular_speed'
        ).get_parameter_value().double_value

        self.distance_tolerance = self.get_parameter(
            'distance_tolerance'
        ).get_parameter_value().double_value

        self.finished = False

        self.get_logger().info(
            f'GoToGoalP started: target=({self.target_x:.2f}, '
            f'{self.target_y:.2f})'
        )


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

        if self.finished:
            return

        # 当前坐标
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # 当前姿态
        q = msg.pose.pose.orientation

        current_yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        # 目标相对位置
        dx = self.target_x - current_x
        dy = self.target_y - current_y

        # 距离误差
        distance_error = math.sqrt(
            dx * dx + dy * dy
        )

        # 目标方向
        target_yaw = math.atan2(
            dy,
            dx
        )

        # 角度误差
        angle_error = self.normalize_angle(
            target_yaw - current_yaw
        )

        # 到达目标
        if distance_error < self.distance_tolerance:

            self.stop_robot()

            self.finished = True

            self.get_logger().info(
                f'Goal reached: '
                f'x={current_x:.2f}, '
                f'y={current_y:.2f}'
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

        # 如果方向偏得太多，先转向
        if abs(angle_error) > math.radians(60.0):
            linear_cmd = 0.0

        cmd = Twist()

        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Distance={distance_error:.2f} m, '
            f'Angle={math.degrees(angle_error):.2f} deg, '
            f'v={linear_cmd:.2f}, '
            f'w={angular_cmd:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GoToGoalP()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()