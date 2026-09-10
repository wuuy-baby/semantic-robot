import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SlamExplorer(Node):
    """
    SLAM 建图辅助巡航节点（简单状态机，不做自主探索、不接 Nav2）。

    安全路线（按 6m x 6m 房间 + 障碍物布局重新设计）：

        房间内边界 x,y ∈ [-0.5, 5.5]
        障碍物:
          box1  : x∈[1.6,2.4] y∈[1.4,2.2]
          box2  : x∈[4.0,4.6] y∈[3.2,4.4]
          pillar: x∈[2.0,2.4] y∈[4.3,4.7]
          table : x∈[4.4,5.2] y∈[1.2,1.8]

        机器人: 0.54 x 0.46, 外接半径 0.355, 内接半宽 x=0.27 y=0.23

        路线（6 段前进 + 5 次转弯）：
          F1: 东 0.9m  y=0   (spawn 脱离段)
          T1: 左转 90°  → 朝北
          F2: 北 0.6m  x=0.9 (脱离南墙)
          T2: 右转 90°  → 朝东
          F3: 东 2.3m  y=0.6 (南侧车道，box1 下方)
          T3: 左转 90°  → 朝北
          F4: 北 2.9m  x=3.2 (中央走廊，box1 右 / box2 左)
          T4: 左转 90°  → 朝西
          F5: 西 2.3m  y=3.5 (北侧返回，pillar 下方)
          T5: 左转 90°  → 朝南
          F6: 南 2.9m  x=0.9 (西侧车道，box1 左)
          → DONE

        逐段安全余量（除 spawn 脱离段外均 ≥ 0.30m）：
          F1: 南墙余量 0.27m（spawn 脱离，不可避免）
          T1: 南墙余量 0.145m（spawn 旋转，0.355<0.5 无碰撞）
          F2: 西墙余量 1.13m ✓
          T2: box1 角余量 0.708m ✓
          F3: box1 余量 0.57m ✓
          T3: box1 角余量 1.434m ✓
          F4: box2 余量 0.53m / box1 余量 0.53m ✓
          T4: box2 角余量 0.499m / pillar 余量 0.776m ✓
          F5: pillar 余量 0.57m ✓
          T5: pillar 余量 1.007m / box1 余量 1.122m ✓
          F6: box1 余量 0.43m / 西墙余量 1.13m ✓

    每段用 /odom 相对位姿反馈（距离/偏航角）控制，
    误差不跨段累积，比纯定时更可靠。
    """

    def __init__(self):
        super().__init__('slam_explorer')

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

        # 六段前进距离 (m)
        self.leg_distances = [0.9, 0.6, 2.3, 2.9, 2.3, 2.9]

        # 每段前进后的转弯方向: +1=左转, -1=右转 (最后一段后不转)
        self.turn_directions = [1, -1, 1, 1, 1]

        # 运动参数
        self.linear_speed = 0.12        # m/s
        self.angular_speed = 0.25       # rad/s (幅值保持不变)
        self.turn_angle = math.pi / 2.0  # 90 度

        # 状态机: FORWARD -> TURN -> FORWARD -> ... -> DONE
        self.state = 'FORWARD'
        self.leg_index = 0      # 当前前进段索引 (0..5)
        self.turn_index = 0     # 当前转弯索引 (0..4)

        # 当前动作的起始位姿（相对基准）
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.get_logger().info('[SLAM Explorer] Started')

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

        if self.state == 'DONE':
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        current_yaw = self.quaternion_to_yaw(
            q.x, q.y, q.z, q.w
        )

        # 每个新动作的第一帧 odom：记录起始位姿
        if self.start_x is None:

            self.start_x = current_x
            self.start_y = current_y
            self.start_yaw = current_yaw

            self.get_logger().info(
                f'[SLAM Explorer] Leg {self.leg_index + 1}/'
                f'{len(self.leg_distances)}: Moving forward'
            )

            return

        cmd = Twist()

        # -------------------------
        # FORWARD 状态：走完当前边
        # -------------------------
        if self.state == 'FORWARD':

            dx = current_x - self.start_x
            dy = current_y - self.start_y

            distance = math.sqrt(dx * dx + dy * dy)

            if distance < self.leg_distances[self.leg_index]:

                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

                self.cmd_pub.publish(cmd)

            else:

                self.stop_robot()

                self.get_logger().info(
                    f'[SLAM Explorer] Leg {self.leg_index + 1} '
                    f'forward done: {distance:.2f} m'
                )

                # 进入转弯
                self.state = 'TURN'
                self.start_x = current_x
                self.start_y = current_y
                self.start_yaw = current_yaw

                direction = self.turn_directions[self.turn_index]
                turn_name = 'left' if direction > 0 else 'right'

                self.get_logger().info(
                    f'[SLAM Explorer] Turn {self.turn_index + 1}/'
                    f'{len(self.turn_directions)}: Turning {turn_name}'
                )

        # -------------------------
        # TURN 状态：转 90 度
        # -------------------------
        elif self.state == 'TURN':

            rotated_angle = self.normalize_angle(
                current_yaw - self.start_yaw
            )

            if abs(rotated_angle) < self.turn_angle:

                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed * \
                    self.turn_directions[self.turn_index]

                self.cmd_pub.publish(cmd)

            else:

                self.stop_robot()

                self.get_logger().info(
                    f'[SLAM Explorer] Turn done: '
                    f'{math.degrees(rotated_angle):.1f} deg'
                )

                self.turn_index += 1
                self.leg_index += 1

                if self.leg_index >= len(self.leg_distances):

                    self.state = 'DONE'

                    self.stop_robot()

                    self.get_logger().info(
                        '[SLAM Explorer] Finished - robot stopped'
                    )

                else:

                    # 继续下一条边
                    self.state = 'FORWARD'
                    self.start_x = current_x
                    self.start_y = current_y
                    self.start_yaw = current_yaw

                    self.get_logger().info(
                        f'[SLAM Explorer] Leg {self.leg_index + 1}/'
                        f'{len(self.leg_distances)}: Moving forward'
                    )

    def destroy_node(self):

        # 退出前确保机器人停止
        self.stop_robot()

        return super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = SlamExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C 或正常结束时都发布一次零速度
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
