import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class MoveDistance(Node):

    def __init__(self):
        super().__init__('move_distance')

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

        self.start_x = None
        self.start_y = None

        self.declare_parameter('target_distance', 2.0)
        self.declare_parameter('speed', 0.2)

        self.target_distance = (
            self.get_parameter('target_distance')
            .get_parameter_value()
            .double_value
        )

        self.speed = (
            self.get_parameter('speed')
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info(
            f'Target distance: {self.target_distance:.2f} m'
            f'Speed: {self.speed:.2f} m/s'
        )


        self.finished = False
        self.get_logger().info('MoveDistance node started')

    def odom_callback(self, msg):

        if self.finished:
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = current_x
            self.start_y = current_y

            self.get_logger().info(
                f'Start position: x={self.start_x:.2f}, y={self.start_y:.2f}'
            )

            return

        dx = current_x - self.start_x
        dy = current_y - self.start_y

        distance = math.sqrt(dx * dx + dy * dy)

        cmd = Twist()

        if distance < self.target_distance:

            cmd.linear.x = self.speed
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f'Distance: {distance:.2f} m'
            )

        else:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.finished = True

            self.get_logger().info(
                f'Target reached: {distance:.2f} m'
            )


def main(args=None):
    rclpy.init(args=args)

    node = MoveDistance()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()