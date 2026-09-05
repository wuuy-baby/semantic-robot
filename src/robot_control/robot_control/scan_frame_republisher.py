import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class ScanFrameRepublisher(Node):

    def __init__(self):
        super().__init__('scan_frame_republisher')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile_sensor_data
        )

    def scan_callback(self, msg):

        msg.header.frame_id = 'laser_link'

        self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ScanFrameRepublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()