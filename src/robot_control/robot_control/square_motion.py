import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SquareMotion(Node):

    def __init__(self):
        super().__init__('square_motion')

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

        # 当前状态
        self.state = 'MOVE'

        # 已经完成了几条边
        self.side_count = 0

        # 直线参数
        

        # 每个动作的起始状态
        self.declare_parameter('side_length',2.0)
        self.declare_parameter('linear_speed',0.2)
        self.declare_parameter('turn_angle_deg',90.0)
        self.declare_parameter('angular_speed',0.3)

        self.target_distance=(
            self.get_parameter('side_length')
            .get_parameter_value()
            .double_value
        )

        self.linear_speed=(
            self.get_parameter('linear_speed')
            .get_parameter_value()
            .double_value
        )

        turn_angle_deg = (
            self.get_parameter('turn_angle_deg')
            .get_parameter_value()
            .double_value
        )

        self.target_angle = math.radians(turn_angle_deg)

        self.angular_speed = (
            self.get_parameter('angular_speed')
            .get_parameter_value()
            .double_value
        )

        # 每个动作的起始状态
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.get_logger().info('SquareMotion node started')


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

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        current_yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        # -------------------------
        # MOVE 状态
        # -------------------------
        if self.state == 'MOVE':

            if self.start_x is None:

                self.start_x = current_x
                self.start_y = current_y

                self.get_logger().info(
                    f'Start moving side {self.side_count + 1}'
                )

                return

            dx = current_x - self.start_x
            dy = current_y - self.start_y

            distance = math.sqrt(
                dx * dx + dy * dy
            )

            if distance < self.target_distance:

                cmd = Twist()

                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

                self.cmd_pub.publish(cmd)

            else:

                self.stop_robot()

                self.get_logger().info(
                    f'Side {self.side_count + 1} finished'
                )

                # 清空直线起点
                self.start_x = None
                self.start_y = None

                # 准备进入旋转
                self.start_yaw = current_yaw

                self.state = 'ROTATE'

        # -------------------------
        # ROTATE 状态
        # -------------------------
        elif self.state == 'ROTATE':

            rotated_angle = self.normalize_angle(
                current_yaw - self.start_yaw
            )

            if abs(rotated_angle) < self.target_angle:

                cmd = Twist()

                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed

                self.cmd_pub.publish(cmd)

            else:

                self.stop_robot()

                self.side_count += 1

                self.get_logger().info(
                    f'Rotation finished, completed sides: {self.side_count}'
                )

                if self.side_count >= 4:

                    self.state = 'FINISHED'

                    self.get_logger().info(
                        'Square motion finished'
                    )

                else:

                    self.state = 'MOVE'

        # -------------------------
        # FINISHED 状态
        # -------------------------
        elif self.state == 'FINISHED':

            self.stop_robot()


def main(args=None):

    rclpy.init(args=args)

    node = SquareMotion()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()