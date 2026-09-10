import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from std_msgs.msg import Float32, Bool

import cv2
import numpy as np

from cv_bridge import CvBridge


class BottleDetector(Node):
    """
    红色 bottle 检测节点（OpenCV + HSV 颜色分割）。

    数据流：
        /camera (sensor_msgs/msg/Image)
          -> cv_bridge 转 OpenCV BGR
          -> HSV 红色双区间分割 (H 0~10, H 170~180)
          -> 形态学开+闭运算去噪
          -> 轮廓查找 + 面积过滤 (>= 100 px)
          -> 取最大轮廓作为 bottle_1
          -> 输出 bounding box / center_u / center_v / area
          -> 调试图像发布到 /bottle_detection/image
    """

    def __init__(self):
        super().__init__('bottle_detector')

        self.bridge = CvBridge()

        # 订阅相机图像（sensor_data QoS 适合图像流）
        self.cam_sub = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            qos_profile_sensor_data
        )

        # 订阅相机标定信息（用于水平偏角计算）
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        # 订阅 LiDAR（用于 bottle 距离融合）
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 最近一次 LaserScan 消息
        self.latest_scan = None

        # 调试图像发布
        self.debug_pub = self.create_publisher(
            Image,
            '/bottle_detection/image',
            10
        )

        # 检测状态发布（供 semantic_search_controller 订阅）
        self.angle_pub = self.create_publisher(
            Float32,
            '/bottle_detection/angle',
            10
        )

        self.distance_pub = self.create_publisher(
            Float32,
            '/bottle_detection/distance',
            10
        )

        self.occluded_pub = self.create_publisher(
            Bool,
            '/bottle_detection/occluded',
            10
        )

        # 语义修正：LiDAR range < threshold 不代表视觉被遮挡，
        # 只代表该 bearing 上的 range 可能来自前景障碍物而非 bottle
        self.range_uncertain_pub = self.create_publisher(
            Bool,
            '/bottle_detection/range_uncertain',
            10
        )

        # 相机内参（来自 CameraInfo.k）
        self.fx = None
        self.cx = None

        # HSV 红色双区间（红色在 HSV 色环两端）
        self.red_lower_1 = np.array([0, 120, 70])
        self.red_upper_1 = np.array([10, 255, 255])

        self.red_lower_2 = np.array([170, 120, 70])
        self.red_upper_2 = np.array([180, 255, 255])

        # 面积过滤阈值（pixel）
        self.min_contour_area = 100

        # 遮挡判断阈值：bottle 距离小于此值视为被遮挡
        self.occlusion_distance_threshold = 1.0

        self.get_logger().info('[Bottle Detector] Started')

    def scan_callback(self, msg):

        # 只保存最近一帧，供图像回调做距离融合
        self.latest_scan = msg

    def camera_info_callback(self, msg):

        # 只需记录一次内参（固定标定，不随帧变化）
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]

            self.get_logger().info(
                f'[Bottle Detector] camera_info received: '
                f'fx={self.fx:.1f}, cx={self.cx:.1f}'
            )

    def image_callback(self, msg):

        # ROS Image -> OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge convert failed: {e}')
            return

        # BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 红色双区间掩码
        mask_1 = cv2.inRange(hsv, self.red_lower_1, self.red_upper_1)
        mask_2 = cv2.inRange(hsv, self.red_lower_2, self.red_upper_2)
        mask = cv2.bitwise_or(mask_1, mask_2)

        # 形态学处理：开运算去小噪点，闭运算填充孔洞
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 轮廓查找
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 过滤面积过小的区域，保留最大轮廓
        valid = [c for c in contours
                 if cv2.contourArea(c) >= self.min_contour_area]

        if not valid:

            # 未检测到：不输出日志（避免刷屏），仍发布调试图像
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            )
            return

        best = max(valid, key=cv2.contourArea)
        area = cv2.contourArea(best)

        # bounding box 与中心
        x, y, w, h = cv2.boundingRect(best)
        center_u = x + w // 2
        center_v = y + h // 2

        # 水平偏角：需要已收到 camera_info
        # angle_rad = atan2(center_u - cx, fx)
        # < 0 目标在左侧, > 0 在右侧, ≈ 0 正前方
        angle_rad = None
        angle_deg = None

        if self.fx is not None:
            angle_rad = math.atan2(center_u - self.cx, self.fx)
            angle_deg = math.degrees(angle_rad)

        # LiDAR 距离融合
        # 注意角度符号约定：Camera <0=左, >0=右
        # ROS LaserScan 机器人坐标 +角=左, -角=右
        # 因此 lidar 角 = -camera 角
        bottle_distance = None

        scan = self.latest_scan

        if angle_rad is not None and scan is not None:

            lidar_angle_rad = -angle_rad

            index = round(
                (lidar_angle_rad - scan.angle_min) / scan.angle_increment
            )

            # 以目标角为中心取 ±2 个点（最多 5 点），过滤无效值
            valid_ranges = []

            for i in range(index - 2, index + 3):

                if 0 <= i < len(scan.ranges):

                    r = scan.ranges[i]

                    if (math.isfinite(r)
                            and scan.range_min <= r
                            and r <= scan.range_max):
                        valid_ranges.append(r)

            if valid_ranges:
                # 中位数抗单点噪声
                bottle_distance = float(np.median(valid_ranges))

        dist_text = (
            f'{bottle_distance:.2f} m'
            if bottle_distance is not None
            else 'N/A'
        )

        # 遮挡判断：距离有效时按阈值判定；无效时为 None
        if bottle_distance is not None:
            occluded = bottle_distance < self.occlusion_distance_threshold
        else:
            occluded = None

        if occluded is None:
            occ_text = 'N/A'
        elif occluded:
            occ_text = 'True'
        else:
            occ_text = 'False'

        # 发布检测状态（供 semantic_search_controller 订阅）
        # 仅在对应字段有效时发布
        if angle_deg is not None:
            angle_msg = Float32()
            angle_msg.data = float(angle_deg)
            self.angle_pub.publish(angle_msg)

        if bottle_distance is not None:
            dist_msg = Float32()
            dist_msg.data = float(bottle_distance)
            self.distance_pub.publish(dist_msg)

        if occluded is not None:
            occ_msg = Bool()
            occ_msg.data = bool(occluded)
            self.occluded_pub.publish(occ_msg)
            # range_uncertain 与 occluded 同值（均为 range<threshold），
            # 但语义不同：range_uncertain 不驱动主动换视角
            ru_msg = Bool()
            ru_msg.data = bool(occluded)
            self.range_uncertain_pub.publish(ru_msg)

        if angle_deg is not None:

            self.get_logger().info(
                f'[Bottle Detector] bottle detected: '
                f'center=({center_u}, {center_v}), area={area:.0f}, '
                f'angle={angle_deg:.1f} deg, distance={dist_text}, '
                f'occluded={occ_text}',
                throttle_duration_sec=1.0
            )

        else:

            self.get_logger().info(
                f'[Bottle Detector] bottle detected: '
                f'center=({center_u}, {center_v}), area={area:.0f} '
                f'(waiting for camera_info)',
                throttle_duration_sec=1.0
            )

        # 遮挡状态文字（用于调试图像）
        if occluded is None:
            status_text = 'N/A'
        elif occluded:
            status_text = 'OCCLUDED'
        else:
            status_text = 'CLEAR'

        # 调试图像：bounding box + 中心点 + 文本
        debug_img = frame.copy()

        cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(debug_img, (center_u, center_v), 4, (0, 0, 255), -1)
        cv2.putText(
            debug_img,
            f'bottle  {angle_deg:.1f} deg  {dist_text}  {status_text}'
            if angle_deg is not None
            else 'bottle',
            (x, max(y - 6, 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA
        )

        # 保留原 header（timestamp / frame_id）
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        debug_msg.header = msg.header

        self.debug_pub.publish(debug_msg)


def main(args=None):

    rclpy.init(args=args)

    node = BottleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
