import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class RotateAngle(Node):

    def __init__(self):
        super().__init__('rotate_angle')

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

        self.start_yaw = None

        self.declare_parameter('target_angle_deg', 90.0)
        self.declare_parameter('angular_speed', 0.3)

        target_angle_deg = (
            self.get_parameter('target_angle_deg')
    .get_parameter_value()
    .double_value
        )

        self.target_angle = math.radians(target_angle_deg)

        self.angular_speed = (
            self.get_parameter('angular_speed')
    .get_parameter_value()
    .double_value
        )

        self.get_logger().info(
            f'Target angle: {target_angle_deg:.2f} deg'
            f'Angular speed: {self.angular_speed:.2f} rad/s'
        )


    
        self.finished = False

        self.get_logger().info('RotateAngle node started')


    def quaternion_to_yaw(self, x, y, z, w):

        siny_cosp = 2.0 * (w * z + x * y)

        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw


    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


    def odom_callback(self, msg):

        if self.finished:
            return

        q = msg.pose.pose.orientation

        current_yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        if self.start_yaw is None:

            self.start_yaw = current_yaw

            self.get_logger().info(
                f'Start yaw: {math.degrees(self.start_yaw):.2f} deg'
            )

            return

        rotated_angle = self.normalize_angle(
            current_yaw - self.start_yaw
        )

        cmd = Twist()

        if abs(rotated_angle) < self.target_angle:

            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f'Rotated: {math.degrees(rotated_angle):.2f} deg'
            )

        else:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.finished = True

            self.get_logger().info(
                f'Target reached: {math.degrees(rotated_angle):.2f} deg'
            )


def main(args=None):

    rclpy.init(args=args)

    node = RotateAngle()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()