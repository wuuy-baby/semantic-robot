import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        self.cmd_pub=self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_sub=self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 2.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.4)

        self.target_x = (
            self.get_parameter('target_x')
            .get_parameter_value()
            .double_value
        )

        self.target_y = (
            self.get_parameter('target_y')
            .get_parameter_value()
            .double_value
        )

        self.linear_speed = (
            self.get_parameter('linear_speed')
            .get_parameter_value()
            .double_value
        )

        self.angular_speed = (
            self.get_parameter('angular_speed')
            .get_parameter_value()
            .double_value
        ) 

        self.distance_tolerance=0.10
        self.angle_tolerance = math.radians(5.0)
        self.finished=False
        self.get_logger().info(
            f'GoToGoal started: target=({self.target_x:.2f}, '
            f'{self.target_y:.2f})'

        )
    def quaternion_to_yaw(self, x, y, z, w):

        siny_cosp = 2.0 * (w * z + x * y)

        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)


    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


    def odom_callback(self, msg):

        if self.finished:
            return

        # 1. 当前机器人位置
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # 2. 当前机器人姿态
        q = msg.pose.pose.orientation

        current_yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        # 3. 当前点 → 目标点
        dx = self.target_x - current_x
        dy = self.target_y - current_y

        # 4. 到目标还有多远
        distance = math.sqrt(
            dx * dx + dy * dy
        )

        # 5. 目标位于哪个方向
        target_yaw = math.atan2(
            dy,
            dx
        )

        # 6. 机器人还需要转多少
        angle_error = self.normalize_angle(
            target_yaw - current_yaw
        )

        cmd = Twist()

        # 7. 到达目标
        if distance < self.distance_tolerance:

            self.stop_robot()

            self.finished = True

            self.get_logger().info(
                f'Goal reached: '
                f'x={current_x:.2f}, '
                f'y={current_y:.2f}'
            )

            return

        # 8. 朝向目标
        if abs(angle_error) > self.angle_tolerance:

            cmd.linear.x = 0.0

            if angle_error > 0:
                cmd.angular.z = self.angular_speed
            else:
                cmd.angular.z = -self.angular_speed

        # 9. 已经基本对准，向前移动
        else:

            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Distance={distance:.2f} m, '
            f'Angle error={math.degrees(angle_error):.2f} deg'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GoToGoal()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
