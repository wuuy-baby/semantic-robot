import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('Odom TF Broadcaster started')

    def odom_callback(self, msg):

        transform = TransformStamped()

        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    node = OdomTFBroadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()