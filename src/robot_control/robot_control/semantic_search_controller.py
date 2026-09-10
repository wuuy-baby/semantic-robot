"""
语义目标搜索控制器（状态机版本）

职责：
    订阅 bottle_detector 发布的检测结果（angle / distance / occluded），
    根据视觉+LiDAR 融合状态在五个状态间转移：
        SEARCHING            -> 没看到 bottle
        TARGET_FOUND         -> 视觉发现 bottle
        REPOSITION_REQUIRED  -> 看到 bottle 但该方向被遮挡
        MOVING_TO_OBSERVATION -> Nav2 正在移动到主动换视角观察点
        TARGET_CLEAR         -> 看到 bottle 且方向畅通

    REPOSITION_REQUIRED 时通过 TF + costmap 安全过滤选择扇区观察点，
    然后通过 Nav2 NavigateToPose 移动到观察点，
    到达后回到 SEARCHING 重新感知，形成主动换视角闭环。

数据来源（bottle_detector 发布）：
    /bottle_detection/angle    std_msgs/Float32   相机水平偏角(deg)
    /bottle_detection/distance std_msgs/Float32   LiDAR 融合距离(m)
    /bottle_detection/occluded std_msgs/Bool      是否被遮挡

对外输出：
    /semantic_search/state     std_msgs/String     当前状态字符串
"""

from enum import Enum
import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

from std_msgs.msg import Float32, Bool, String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, ComputePathToPose

from tf2_ros import Buffer, TransformListener, TransformException


# -----------------------------
# 状态枚举（Python Enum）
# -----------------------------
class SearchState(Enum):
    SEARCHING = 'SEARCHING'
    TARGET_FOUND = 'TARGET_FOUND'
    REPOSITION_REQUIRED = 'REPOSITION_REQUIRED'
    TARGET_CLEAR = 'TARGET_CLEAR'
    MOVING_TO_OBSERVATION = 'MOVING_TO_OBSERVATION'
    MOVING_TO_OBSERVATION_B = 'MOVING_TO_OBSERVATION_B'
    WAITING_FOR_OBSERVATION_B = 'WAITING_FOR_OBSERVATION_B'
    # Stage 5.2/5.3：向 approach pose 导航 + 到达后最终视觉确认
    NAVIGATING_TO_APPROACH = 'NAVIGATING_TO_APPROACH'
    APPROACH_REACHED = 'APPROACH_REACHED'
    FINAL_CONFIRMATION = 'FINAL_CONFIRMATION'


class SemanticSearchController(Node):
    """语义目标搜索状态机节点。"""

    def __init__(self):
        super().__init__('semantic_search_controller')

        # =====================
        # 参数
        # =====================
        # 检测超时：超过此时间未收到 angle，视为 bottle 消失
        self.declare_parameter('detection_timeout', 1.5)
        self.detection_timeout = self.get_parameter(
            'detection_timeout').value

        # 观察候选点偏移参数（单位 m）
        # forward_offset: 沿目标方向前移距离
        # lateral_offset: 垂直目标方向的左右偏移距离
        self.declare_parameter('observation_forward_offset', 0.3)
        self.observation_forward_offset = self.get_parameter(
            'observation_forward_offset').value

        self.declare_parameter('observation_lateral_offset', 0.8)
        self.observation_lateral_offset = self.get_parameter(
            'observation_lateral_offset').value

        # 多个侧向偏移候选（m）：沿左右侧向方向逐个尝试
        # 第一版硬编码列表，不做复杂 ROS 数组参数
        self.observation_lateral_offsets = [0.8, 1.0, 1.2, 1.4]

        # 前向扇区候选搜索配置
        # search_radii: 候选距离（m）
        # search_angle_offsets_deg: 相对目标视线的角度（deg）
        #   不含 0°，因为正前方已知有近障碍物
        self.search_radii = [0.6, 0.9, 1.2]
        self.search_angle_offsets_deg = [-90, -60, -30, 30, 60, 90]

        # 候选点区域统计半径（m）：评估候选点周围圆形区域 cost
        self.declare_parameter('candidate_check_radius', 0.25)
        self.candidate_check_radius = self.get_parameter(
            'candidate_check_radius').value

        # =====================
        # 订阅 bottle_detector 输出
        # =====================
        self.angle_sub = self.create_subscription(
            Float32,
            '/bottle_detection/angle',
            self.angle_callback,
            10
        )

        self.distance_sub = self.create_subscription(
            Float32,
            '/bottle_detection/distance',
            self.distance_callback,
            10
        )

        self.occluded_sub = self.create_subscription(
            Bool,
            '/bottle_detection/occluded',
            self.occluded_callback,
            10
        )

        # 语义修正：订阅 range_uncertain 代替 occluded 驱动状态机
        # range_uncertain=True 只表示 LiDAR bearing 上有近障碍，
        # 不等于 Camera 视线被遮挡，不再触发自动换视角
        self.range_uncertain_sub = self.create_subscription(
            Bool,
            '/bottle_detection/range_uncertain',
            self.range_uncertain_callback,
            10
        )

        # =====================
        # 订阅 Nav2 global costmap（用于查询候选点 cost）
        # 使用 TRANSIENT_LOCAL QoS：global_costmap 默认用此 QoS 发布
        # =====================
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            costmap_qos
        )

        # =====================
        # 状态发布
        # TRANSIENT_LOCAL QoS：晚加入的订阅者也能收到最后一条状态
        # =====================
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.state_pub = self.create_publisher(
            String,
            '/semantic_search/state',
            state_qos
        )

        # =====================
        # TF2：用于在 REPOSITION_REQUIRED 时读取机器人 map 坐标
        # =====================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =====================
        # 内部状态
        # =====================
        self.state = SearchState.SEARCHING

        # 最近一次各字段值（None 表示未知/未收到）
        self.latest_angle = None
        self.latest_distance = None
        self.latest_occluded = None
        self.latest_range_uncertain = None  # 语义修正：代替 occluded 驱动状态机

        # 最后一次收到 angle 的时间（纳秒）
        self.last_detection_time_ns = None

        # TF 查询重试状态（仅在 REPOSITION_REQUIRED 时使用）
        # pending=True 表示仍在等待 TF 查询成功
        # warning_shown=True 表示首次失败 warning 已打印，避免 0.2s 重复刷屏
        self.pose_query_pending = False
        self.pose_query_warning_shown = False

        # Global costmap 缓存与候选点 cost 查询状态
        self.latest_global_costmap = None
        # 保存最近一次计算的左右候选点
        self.left_candidate = None
        self.right_candidate = None
        # 防止每次 costmap 更新都重复打印候选 cost
        self.candidate_costs_printed = False
        # 防止每次 costmap 更新都重复打印区域统计
        self.candidate_region_printed = False
        # 防止每次 costmap 更新都重复打印扇区搜索结果
        self.sector_search_printed = False

        # =====================
        # 双视点视觉定位 - Observation A
        # 第一条视觉 bearing ray：在 TARGET_FOUND + range_uncertain 时记录
        # 使用 5 帧 camera_angle median 抑制单帧抖动
        # =====================
        self.observation_a = None
        self.observation_a_pending = False
        self.observation_a_warning_shown = False
        # temporal bearing filtering：连续收集多帧取 median
        self.bearing_sample_count = 5
        self.observation_a_angle_samples = []

        # =====================
        # 双视点视觉定位 - Observation B goal
        # 基于 Ray A 垂直方向选择 triangulation baseline 位置
        # observation_b_goal 是目标位置，observation_b 是未来真实观测
        # =====================
        self.observation_b_goal = None
        self.observation_b_selection_done = False
        # baseline 候选距离：0.40m 对 ~5m 远 bottle 视差不足，
        # 从 0.50m 起跳以稳定三角定位，上限 0.6m
        self.baseline_distances = [0.5, 0.6]

        # =====================
        # 双视点视觉定位 - Observation B 真实记录
        # 机器人到达 B goal 后，通过 TF 获取真实位姿 + 新 Camera bearing
        # =====================
        self.observation_b = None
        self.observation_b_nav_started = False
        self.observation_b_nav_active = False
        self.observation_b_pending = False
        # temporal bearing filtering：B 停车后连续收集多帧取 median
        self.observation_b_angle_samples = []

        # =====================
        # 三角定位结果：Ray A + Ray B 求交得到 bottle 在 map 的估计位置
        # 只在两组 Observation 都记录完成后计算一次
        # =====================
        # triangulation 在 odom 中完成（不受 AMCL map->odom 动态修正影响）
        self.target_odom_position = None
        # 最终结果一次性通过最新 map<-odom TF 转换到 map
        self.target_map_position = None
        # 定位来源：'camera_lidar_direct'（可靠 range 直接定位）
        # 或 'active_triangulation'（range_uncertain 双视点 fallback）
        self.target_localization_source = None

        # =====================
        # 阶段五：Approach pose（目标估计位置前的安全接近点）
        # Stage 5.1：只生成 + costmap safety + ComputePathToPose 验证，不导航
        # =====================
        self.approach_pose = None
        self.approach_generation_done = False
        # Progress-based safe approach planner：
        # approach_candidates 为按优先级排序的 LOCAL SAFE 候选列表，
        # approach_candidate_index 为当前正在做 ComputePathToPose 验证的下标
        self.approach_candidates = []
        self.approach_candidate_index = 0
        # Approach 路径验证 guard：None=仍在尝试候选, True=找到有效路径, False=全部失败
        self.approach_path_validation_started = False
        self.approach_path_valid = None

        # Stage 5.2：Approach 导航 guard（一个 approach pose 只发一次 goal）
        self.approach_nav_started = False
        self.approach_nav_active = False
        # Stage 5.3：到达 approach pose 后的最终视觉确认
        self.approach_arrival_time_ns = None
        self.final_confirmation_done = False
        self.mission_success = False
        self.final_confirmation_timeout_sec = 5.0
        # Stage 5 iterative approach：迭代逼近直到最终观察距离
        self.final_standoff_distance = 1.2   # 最终成功距离阈值 (m)
        self.approach_iteration = 0          # 已完成/进行中的 approach 轮次
        self.max_approach_iterations = 3     # 最多 approach 导航次数

        # =====================
        # SEARCHING 主动旋转搜索（原地低速旋转，不调用 Nav2）
        # =====================
        self.search_angular_speed = 0.30     # 搜索角速度 rad/s
        self.search_timeout_sec = 30.0       # 最大搜索时长 (s)
        self.search_start_time_ns = None     # 本轮搜索开始时刻
        self.search_failed = False           # 搜索超时失败后停止旋转
        self.search_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # =====================
        # Nav2 NavigateToPose Action Client
        # =====================
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        # 防止重复发送 goal
        self.nav_goal_active = False
        # 保存选定的观察点（由 sector search 选出）
        self.selected_observation = None
        # 保存当前 goal_handle（用于结果等待）
        self.current_goal_handle = None

        # =====================
        # Nav2 ComputePathToPose Action Client
        # 用于 Observation B Goal 路径验证（dry-run，不移动机器人）
        # =====================
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )
        # 路径验证 guard：None=未验证, True=有效, False=无效
        self.observation_b_path_validation_started = False
        self.observation_b_path_valid = None

        # =====================
        # 周期定时器：0.2 秒检查一次超时
        # =====================
        self.timer = self.create_timer(0.2, self.timer_callback)

        # 初始化后发布一次 SEARCHING
        self._publish_state()

        self.get_logger().info(
            f'[SemanticSearch] Started (detection_timeout={self.detection_timeout}s)'
        )

    # -----------------------------
    # 订阅回调
    # -----------------------------
    def angle_callback(self, msg):
        """收到 angle：bottle 被视觉检测到。"""

        self.latest_angle = float(msg.data)
        self.last_detection_time_ns = self.get_clock().now().nanoseconds

        # SEARCHING -> TARGET_FOUND：视觉首次发现 bottle，立即停止旋转搜索
        if self.state == SearchState.SEARCHING:
            self.stop_search_motion()
            self.get_logger().info(
                '[SEARCH]\n    bottle detected, stopping rotation'
            )
            self.transition_to(
                SearchState.TARGET_FOUND,
                'bottle detected'
            )

        # WAITING_FOR_OBSERVATION_B：每收到一帧新 Camera angle 采一次样
        # 只有停车后（进入 WAITING_FOR_OBSERVATION_B）的新帧才进入 B samples，
        # 避免移动过程中图像污染 B 观测
        # 收满 bearing_sample_count 帧后置 pending，让 timer 重试 TF 记录
        if (
            self.state == SearchState.WAITING_FOR_OBSERVATION_B
            and self.observation_b is None
            and len(self.observation_b_angle_samples) < self.bearing_sample_count
        ):
            self.observation_b_angle_samples.append(self.latest_angle)
            self.get_logger().info(
                f'[OBS B SAMPLE] '
                f'{len(self.observation_b_angle_samples)}/{self.bearing_sample_count} '
                f'angle={self.latest_angle:.2f} deg'
            )
            if (
                len(self.observation_b_angle_samples) >= self.bearing_sample_count
                and not self.observation_b_pending
            ):
                self.observation_b_pending = True
                self.get_logger().info(
                    '[OBSERVATION B] samples collected, pending TF lookup'
                )

        # 收到新数据后，根据 occluded 继续推进状态
        # MOVING_TO_OBSERVATION_B / WAITING_FOR_OBSERVATION_B 时 _update_target_state
        # 内部会因 state 不在允许集合中而直接 return，不触发任何状态转移
        self._update_target_state()

        # Observation A angle 采样：maybe_start_observation_a 在 _update_target_state
        # 内部置 observation_a_pending=True 后，每帧新 angle 追加一次（仅 TARGET_FOUND）
        if (
            self.observation_a_pending
            and self.observation_a is None
            and self.state == SearchState.TARGET_FOUND
            and len(self.observation_a_angle_samples) < self.bearing_sample_count
        ):
            self.observation_a_angle_samples.append(self.latest_angle)
            self.get_logger().info(
                f'[OBS A SAMPLE] '
                f'{len(self.observation_a_angle_samples)}/{self.bearing_sample_count} '
                f'angle={self.latest_angle:.2f} deg'
            )

    def distance_callback(self, msg):
        """收到 distance：更新最新距离值。"""

        self.latest_distance = float(msg.data)

    def occluded_callback(self, msg):
        """收到 occluded：更新遮挡标志并重判目标状态。"""

        self.latest_occluded = bool(msg.data)

        # 只有在已经发现目标的情况下，occluded 才有意义
        self._update_target_state()

    def costmap_callback(self, msg):
        """
        收到 global costmap：保存最新地图。
        若已有候选点且尚未打印 cost/region，补打一次。
        """

        self.latest_global_costmap = msg

        # 候选点已计算但单 cell cost 未打印（之前 costmap 还没到）
        if (
            self.left_candidate is not None
            and self.right_candidate is not None
            and not self.candidate_costs_printed
        ):
            self._print_candidate_costs()

        # 候选点已计算但区域统计未打印
        if (
            self.left_candidate is not None
            and self.right_candidate is not None
            and not self.candidate_region_printed
        ):
            self._print_candidate_region()

    def range_uncertain_callback(self, msg):
        """
        收到 range_uncertain 消息。
        语义修正：range_uncertain=True 只表示 LiDAR bearing 上有近障碍物，
        不等于 Camera 视线被遮挡。
        当前阶段不由此触发 REPOSITION_REQUIRED。
        """

        self.latest_range_uncertain = bool(msg.data)

        # 只有在已经发现目标的情况下，range_uncertain 才有意义
        self._update_target_state()

    # -----------------------------
    # 状态转移核心逻辑
    # -----------------------------
    def maybe_start_observation_a(self):
        """
        检查是否满足 Observation A 触发条件，满足则启动 pending。
        条件：TARGET_FOUND + angle 有效 + range_uncertain=True + 尚未记录 + 未 pending
        """
        if self.state != SearchState.TARGET_FOUND:
            return
        if self.latest_angle is None:
            return
        if self.latest_range_uncertain is not True:
            return
        if self.observation_a is not None:
            return
        if self.observation_a_pending:
            return

        # 一次性 debug 日志：帮助定位触发链路
        self.get_logger().info(
            f'[OBS A DEBUG] state={self.state.name}, '
            f'angle={self.latest_angle:.2f}, '
            f'range_uncertain={self.latest_range_uncertain}, '
            f'observation_a={self.observation_a}, '
            f'pending={self.observation_a_pending}'
        )

        self.observation_a_pending = True
        self.get_logger().info('[OBSERVATION A] pending TF lookup')

    def try_record_observation_a(self):
        """
        尝试通过 TF 获取机器人位姿并记录 Observation A。
        使用 5 帧 camera_angle 的 median 抑制单帧抖动。
        样本不足或 TF 失败时保持 pending=True，下次 timer 重试。
        """
        if self.observation_a is not None:
            return  # 已记录
        if not self.observation_a_pending:
            return
        # 样本未收满，等待更多新帧（不在 timer 中重复采样）
        if len(self.observation_a_angle_samples) < self.bearing_sample_count:
            return

        pose = self.get_robot_pose()
        if pose is None:
            # TF 不可用，保持 pending，下次 timer 重试
            return

        # Camera 光心 TF（map）：视觉射线必须从 camera_link 出发，不是 base_footprint
        cam_pose = self.get_camera_pose()
        if cam_pose is None:
            # Camera TF 不可用，保持 pending，下次 timer 重试
            return

        # odom 位姿：短基线三角定位统一在 odom 中完成
        pose_odom = self.get_robot_pose_odom()
        if pose_odom is None:
            # odom TF 不可用，保持 pending，下次 timer 重试
            return
        cam_pose_odom = self.get_camera_pose_odom()
        if cam_pose_odom is None:
            return

        # 5 帧 median 抑制单帧抖动
        camera_angle_filtered = statistics.median(
            self.observation_a_angle_samples
        )
        self.get_logger().info(
            f'[OBS A FILTER]\n'
            f'    samples={[round(a, 2) for a in self.observation_a_angle_samples]}\n'
            f'    median_angle={camera_angle_filtered:.2f} deg'
        )

        # TF 成功，计算 Observation A（map 字段保留给 planner/Nav2）
        robot_x, robot_y, robot_yaw = pose
        camera_x, camera_y, _ = cam_pose
        target_map_bearing = self.compute_target_map_bearing(
            robot_yaw,
            camera_angle_filtered
        )

        # odom 字段：Ray A origin/bearing 全部在 odom 中表达
        robot_odom_x, robot_odom_y, robot_odom_yaw = pose_odom
        camera_odom_x, camera_odom_y, camera_odom_yaw = cam_pose_odom
        # 图像约定：camera_angle + = right；ROS yaw + = left
        # => 目标相对 Camera 方向 = -camera_angle
        target_odom_bearing = math.atan2(
            math.sin(camera_odom_yaw - math.radians(camera_angle_filtered)),
            math.cos(camera_odom_yaw - math.radians(camera_angle_filtered))
        )

        self.observation_a = {
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_yaw': robot_yaw,
            'camera_x': camera_x,
            'camera_y': camera_y,
            'camera_angle_deg': camera_angle_filtered,
            'target_map_bearing': target_map_bearing,
            # odom 几何（triangulation 实际使用）
            'robot_odom_x': robot_odom_x,
            'robot_odom_y': robot_odom_y,
            'robot_odom_yaw': robot_odom_yaw,
            'camera_odom_x': camera_odom_x,
            'camera_odom_y': camera_odom_y,
            'camera_odom_yaw': camera_odom_yaw,
            'target_odom_bearing': target_odom_bearing
        }
        self.observation_a_pending = False

        # 打印一次 Observation A + Ray A ODOM（Ray 在 odom 坐标系中）
        bearing_deg = math.degrees(target_map_bearing)
        odom_bearing_deg = math.degrees(target_odom_bearing)
        dir_x = math.cos(target_odom_bearing)
        dir_y = math.sin(target_odom_bearing)
        self.get_logger().info(
            f'[OBSERVATION A]\n'
            f'    robot_x={robot_x:.2f} m\n'
            f'    robot_y={robot_y:.2f} m\n'
            f'    robot_yaw={math.degrees(robot_yaw):.2f} deg\n'
            f'    camera_x={camera_x:.2f} m\n'
            f'    camera_y={camera_y:.2f} m\n'
            f'    camera_angle={camera_angle_filtered:.2f} deg\n'
            f'    target_map_bearing={bearing_deg:.2f} deg\n'
            f'    camera_odom_x={camera_odom_x:.2f} m\n'
            f'    camera_odom_y={camera_odom_y:.2f} m\n'
            f'    camera_odom_yaw={math.degrees(camera_odom_yaw):.2f} deg\n'
            f'    target_odom_bearing={odom_bearing_deg:.2f} deg\n'
            f'[RAY A ODOM]\n'
            f'    origin=({camera_odom_x:.2f}, {camera_odom_y:.2f})\n'
            f'    bearing={odom_bearing_deg:.2f} deg\n'
            f'    direction_x={dir_x:.3f}\n'
            f'    direction_y={dir_y:.3f}'
        )

    # -----------------------------
    # Observation B 候选选择（基于 Ray A 垂直方向）
    # -----------------------------
    def maybe_select_observation_b(self):
        """
        当 Observation A 已记录且 costmap 已就绪时，
        以 A 为中心、沿 target Ray A 的 6 个角度偏移 × 3 个半径
        生成候选观察点，通过严格 costmap 安全过滤后选择 B goal。
        本阶段为 dry-run：不发送 NavigateToPose，不移动机器人。
        """

        # 去重保护：已选择或 A 未就绪时不执行
        if self.observation_b_selection_done:
            return
        if self.observation_a is None:
            return
        if self.latest_global_costmap is None:
            return  # costmap 未到，等待下次

        self.observation_b_selection_done = True

        # Observation A 基础几何
        theta = self.observation_a['target_map_bearing']
        a_x = self.observation_a['robot_x']
        a_y = self.observation_a['robot_y']

        # =====================
        # 新 Observation B viewpoint planner 配置
        # =====================
        # 候选半径
        radii = [0.5, 0.6, 0.7]
        # 相对 target Ray A 的角度偏移（不含 0°：正前方不产生侧向视差）
        offsets_deg = [-90, -60, -30, 30, 60, 90]
        # 侧向 baseline 下限：保证三角定位视差
        min_lateral_baseline = 0.30
        # 候选区域 clearance 检查半径（比旧 0.25m 更保守）
        candidate_clearance_radius = 0.35
        # 严格安全阈值
        center_cost_threshold = 50
        region_max_cost_threshold = 80

        self.get_logger().info('[VIEWPOINT PLANNER] starting candidate search')

        candidates = []

        for radius in radii:
            for offset_deg in offsets_deg:
                offset_rad = math.radians(offset_deg)
                candidate_angle = theta + offset_rad

                cx = a_x + radius * math.cos(candidate_angle)
                cy = a_y + radius * math.sin(candidate_angle)

                # 相对 Ray A 的侧向位移（决定三角定位视差）
                lateral_baseline = radius * abs(math.sin(offset_rad))

                center_cost = self.get_costmap_cost(cx, cy)
                region = self.evaluate_candidate_region(
                    cx, cy, radius=candidate_clearance_radius
                )

                # =====================
                # 安全性检查（严格标准）
                # =====================
                safe = False
                reason = ''

                # epsilon 容差：避免 0.6*sin(30°)≈0.2999999 浮点边界误拒
                _eps = 1e-6
                if lateral_baseline + _eps < min_lateral_baseline:
                    reason = 'insufficient lateral baseline'
                elif center_cost is None:
                    reason = 'candidate outside map'
                elif region is None:
                    reason = 'region evaluation None'
                elif region.get('outside'):
                    reason = 'region outside costmap'
                elif region.get('unknown_count', 0) > 0:
                    reason = 'unknown cells in candidate region'
                elif region.get('out_of_bounds_count', 0) > 0:
                    reason = 'out_of_bounds cells in candidate region'
                elif center_cost >= center_cost_threshold:
                    reason = 'high center inflation cost'
                elif (
                    region.get('max_cost') is not None
                    and region['max_cost'] >= region_max_cost_threshold
                ):
                    reason = 'obstacle too close to candidate footprint'
                else:
                    safe = True

                # 格式化日志
                cc_str = str(center_cost) if center_cost is not None else 'N/A'
                mc_str = (
                    str(region.get('max_cost'))
                    if region and region.get('max_cost') is not None
                    else 'N/A'
                )
                mean_str = (
                    f"{region['mean_cost']:.1f}"
                    if region and region.get('mean_cost') is not None
                    else 'N/A'
                )
                unk_str = (
                    str(region.get('unknown_count', 0)) if region else 'N/A'
                )
                result_str = 'SAFE' if safe else 'REJECTED'

                self.get_logger().info(
                    f'[VIEWPOINT CANDIDATE]\n'
                    f'    offset={offset_deg:+d} deg\n'
                    f'    radius={radius:.2f} m\n'
                    f'    x={cx:.2f}\n'
                    f'    y={cy:.2f}\n'
                    f'    yaw={math.degrees(theta):.2f} deg\n'
                    f'    lateral_baseline={lateral_baseline:.2f}\n'
                    f'    center_cost={cc_str}\n'
                    f'    max_cost={mc_str}\n'
                    f'    mean_cost={mean_str}\n'
                    f'    unknown={unk_str}\n'
                    f'    result={result_str}\n'
                    f'    reason={reason}'
                )

                candidates.append({
                    'offset_deg': offset_deg,
                    'radius': radius,
                    'x': cx,
                    'y': cy,
                    'yaw': theta,
                    'lateral_baseline': lateral_baseline,
                    'center_cost': center_cost,
                    'max_cost': region.get('max_cost') if region else None,
                    'mean_cost': region.get('mean_cost') if region else None,
                    'unknown_count': region.get('unknown_count', 0) if region else 0,
                    'out_of_bounds_count': region.get('out_of_bounds_count', 0) if region else 0,
                    'safe': safe,
                })

        # =====================
        # 选择最优候选
        # 排序优先级：
        #   1) max_cost 更低（更空旷、更安全）
        #   2) mean_cost 更低
        #   3) lateral_baseline 更大（视差更好）
        #   4) radius 更小（移动距离更短）
        # =====================
        safe_candidates = [c for c in candidates if c['safe']]

        if safe_candidates:
            best = min(
                safe_candidates,
                key=lambda c: (
                    c['max_cost'] if c['max_cost'] is not None else 9999,
                    c['mean_cost'] if c['mean_cost'] is not None else 9999.0,
                    -(c['lateral_baseline']),  # 取负：lateral 更大排更前
                    c['radius'],
                )
            )
            self.observation_b_goal = {
                'x': best['x'],
                'y': best['y'],
                'yaw': theta,
                'radius': best['radius'],
                'offset_deg': best['offset_deg'],
                'lateral_baseline': best['lateral_baseline'],
            }
            self.get_logger().info(
                f'[OBSERVATION B GOAL]\n'
                f'    offset={best["offset_deg"]:+d} deg\n'
                f'    radius={best["radius"]:.2f} m\n'
                f'    lateral_baseline={best["lateral_baseline"]:.2f} m\n'
                f'    x={best["x"]:.2f}\n'
                f'    y={best["y"]:.2f}\n'
                f'    yaw={math.degrees(theta):.2f} deg\n'
                f'    center_cost={best["center_cost"]}\n'
                f'    max_cost={best["max_cost"]}'
            )
        else:
            self.get_logger().info(
                '[OBSERVATION B GOAL]\n'
                '    NONE - no safe viewpoint candidate passed strict filter'
            )

        # dry-run：验证阶段不发送 NavigateToPose，不移动机器人
        self.get_logger().info(
            '[VIEWPOINT PLANNER] dry-run complete, navigation disabled for validation'
        )

    def _update_target_state(self):
        """
        语义修正后：Camera 能看到 bottle 即认为目标可见。
        range_uncertain=True 只打印提示，不触发 REPOSITION_REQUIRED。
        当前阶段：仅当 Camera 完全丢失目标（detection timeout）才回 SEARCHING。
        """

        # 没有 range_uncertain 数据，无法判定（保留兼容）
        if self.latest_range_uncertain is None:
            return

        # 只在"目标仍然可见"的状态下处理
        if self.state not in (
            SearchState.TARGET_FOUND,
            SearchState.REPOSITION_REQUIRED,
            SearchState.TARGET_CLEAR,
        ):
            return

        # 语义修正：Camera 仍能看到目标 -> 保持 TARGET_FOUND / TARGET_CLEAR
        # range_uncertain=True 不再触发 REPOSITION_REQUIRED
        if self.latest_range_uncertain:
            # 目标可见但 LiDAR range 不确定
            if self.state == SearchState.TARGET_FOUND:
                self.get_logger().info(
                    f'[TARGET VISIBLE] angle={self._fmt(self.latest_angle)} deg, '
                    f'range_uncertain=True, '
                    f'reason=foreground LiDAR return does not prove visual occlusion',
                    throttle_duration_sec=2.0
                )
                # 尝试触发 Observation A（仅一次，内部有去重保护）
                self.maybe_start_observation_a()
            # 不主动转移
        else:
            # range 确定可靠 -> 目标方向清晰
            if self.state != SearchState.TARGET_CLEAR:
                self.transition_to(
                    SearchState.TARGET_CLEAR,
                    'target direction is clear'
                )
            # Stage 4 PRIMARY：可靠 range 直接用 Camera bearing + LiDAR range 定位，
            # 不进入 Observation A/B 三角定位（内部幂等，TF 失败由 timer 重试）
            self.direct_target_localization()

    # -----------------------------
    # 超时检查定时器
    # -----------------------------
    def timer_callback(self):
        """
        每 0.2 秒执行：
            0) SEARCHING 下主动原地旋转搜索（发现 bottle 立即停车）
            1) 若在 REPOSITION_REQUIRED 且 pose_query_pending，重试 TF 查询
            2) 检查 detection_timeout，超时则回到 SEARCHING
        """

        # -----------------------------
        # SEARCHING 主动旋转搜索（仅 SEARCHING + 无 Nav2 goal 时发布 cmd_vel）
        # -----------------------------
        self.update_active_search()

        # -----------------------------
        # TF 重试逻辑（独立于 detection_timeout，不依赖 last_detection_time_ns）
        # -----------------------------
        if (
            self.state == SearchState.REPOSITION_REQUIRED
            and self.pose_query_pending
        ):
            pose = self.get_robot_pose()

            if pose is not None:
                self.pose_query_pending = False
                # TF 成功且有 camera angle，计算目标方向 + 候选观察点
                if self.latest_angle is not None:
                    target_map_yaw = self.compute_target_map_bearing(
                        pose[2],          # robot_yaw (rad)
                        self.latest_angle  # camera_angle (deg)
                    )
                    candidates = self.compute_observation_candidates(
                        pose[0],           # robot_x
                        pose[1],           # robot_y
                        target_map_yaw
                    )
                    # 保存候选点，供 costmap_callback 或此处立即查询
                    self.left_candidate = candidates[0]
                    self.right_candidate = candidates[1]
                    # 若 costmap 已就绪，立即打印单 cell cost + 区域统计
                    if self.latest_global_costmap is not None:
                        self._print_candidate_costs()
                        self._print_candidate_region()
                    # 多偏移候选搜索 + 安全过滤 + 选择
                    if self.latest_global_costmap is not None:
                        self.search_observation_candidate(
                            pose[0], pose[1], target_map_yaw
                        )
                    # 前向扇区候选搜索
                    if (
                        self.latest_global_costmap is not None
                        and not self.sector_search_printed
                    ):
                        self.search_sector_observation_candidate(
                            pose[0], pose[1], target_map_yaw
                        )
                        self.sector_search_printed = True

        # -----------------------------
        # Observation A TF 重试（独立于 detection_timeout）
        # 在 TARGET_FOUND + range_uncertain 时记录第一条 bearing ray
        # -----------------------------
        if self.observation_a_pending and self.observation_a is None:
            self.try_record_observation_a()

        # -----------------------------
        # Observation B 候选选择（A 记录完成后，costmap 就绪时执行一次）
        # -----------------------------
        if (
            self.observation_a is not None
            and not self.observation_b_selection_done
        ):
            self.maybe_select_observation_b()

        # -----------------------------
        # Observation B 路径验证（ComputePathToPose）
        # B goal 选择完成后，验证路径是否有效且不经过高 cost 区域
        # -----------------------------
        if (
            self.observation_b_goal is not None
            and not self.observation_b_path_validation_started
            and self.observation_b_path_valid is None
        ):
            self.validate_observation_b_path()

        # -----------------------------
        # Observation B Nav2 goal 发送
        # 必须同时满足：candidate 已选 + 路径验证通过(PATH VALID) + 未开始导航
        # PATH REJECTED 时 observation_b_path_valid=False，绝对不会发送 NavigateToPose
        # -----------------------------
        if (
            self.observation_a is not None
            and self.observation_b_goal is not None
            and self.observation_b_path_valid is True
            and self.observation_b is None
            and not self.observation_b_nav_started
            and not self.observation_b_nav_active
        ):
            self.get_logger().info(
                '[VIEWPOINT PLANNER] candidate and path validated, navigation enabled'
            )
            self.send_observation_b_goal()

        # -----------------------------
        # Observation B TF 重试（到达 B 视点后，等待新 Camera detection）
        # -----------------------------
        if self.observation_b_pending and self.observation_b is None:
            self.try_record_observation_b()

        # -----------------------------
        # odom triangulation 成功后，重试 odom -> map 最终转换
        # （map<-odom TF 暂不可用时不重新 triangulate，只重试转换）
        # -----------------------------
        if (
            self.target_odom_position is not None
            and self.target_map_position is None
        ):
            self.try_transform_target_to_map()

        # -----------------------------
        # Stage 4 PRIMARY：可靠 range 直接定位重试
        # 仅 TARGET_FOUND / TARGET_CLEAR 且三角 fallback 未开始时尝试；
        # range_uncertain=True 时 direct_target_localization 内部直接返回，
        # 由既有 Obs A/B 三角定位路径处理。
        # -----------------------------
        if (
            self.target_map_position is None
            and self.observation_a is None
            and not self.observation_a_pending
            and self.observation_b is None
            and self.state in (
                SearchState.TARGET_FOUND,
                SearchState.TARGET_CLEAR,
            )
        ):
            self.direct_target_localization()

        # -----------------------------
        # 阶段五 Stage 5.1：Approach pose 生成（target_map_position 就绪后）
        # 直接定位或三角定位成功后都会进入这里
        # -----------------------------
        if (
            self.target_map_position is not None
            and not self.approach_generation_done
        ):
            self.generate_approach_pose()

        # -----------------------------
        # Approach 路径验证（ComputePathToPose，复用阶段四 client）
        # 按排序后的 local-safe 候选逐个尝试，失败自动换下一个
        # Stage 5.1 dry-run：不发送 NavigateToPose
        # -----------------------------
        if (
            self.approach_candidates
            and self.approach_path_valid is None
            and not self.approach_path_validation_started
        ):
            self.validate_approach_path()

        # -----------------------------
        # Stage 5.2：Approach path 验证通过后发送 NavigateToPose
        # （result 回调已直接调用过一次，这里作为 Nav2 server 未 ready 时的兜底重试；
        #   approach_nav_started guard 保证一个 pose 只发一次 goal）
        # -----------------------------
        if (
            self.approach_path_valid is True
            and self.approach_pose is not None
            and not self.approach_nav_started
            and not self.approach_nav_active
        ):
            self.send_approach_goal()

        # -----------------------------
        # Stage 5.3：到达 approach pose 后等待新鲜 bottle detection（有限超时）
        # -----------------------------
        if (
            self.state == SearchState.FINAL_CONFIRMATION
            and not self.final_confirmation_done
        ):
            self.check_final_confirmation()

        # -----------------------------
        # detection_timeout 检查
        # 导航 / 等待观测 / Stage5 导航 期间跳过：不因视觉暂时丢失而回 SEARCHING
        # -----------------------------
        if self.state in (
            SearchState.MOVING_TO_OBSERVATION,
            SearchState.MOVING_TO_OBSERVATION_B,
            SearchState.NAVIGATING_TO_APPROACH,
            SearchState.APPROACH_REACHED,
            SearchState.FINAL_CONFIRMATION,
        ):
            return

        # WAITING_FOR_OBSERVATION_B 时也跳过 detection_timeout
        # （机器人刚到达，Camera 可能还没出新一帧）
        if self.state == SearchState.WAITING_FOR_OBSERVATION_B:
            return

        # Stage 4 已完成定位（直接定位或三角定位）并进入 Stage 5：
        # 此时 Camera 更新率可能较低（VMware 约 2~3Hz），
        # detection timeout 不允许把任务重置回 SEARCHING / 清空 target_map_position。
        if self.target_map_position is not None:
            return

        if self.last_detection_time_ns is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.last_detection_time_ns) / 1e9

        if elapsed > self.detection_timeout:

            # 任何目标相关状态 -> SEARCHING
            if self.state != SearchState.SEARCHING:
                self.transition_to(
                    SearchState.SEARCHING,
                    'target detection timeout'
                )

            # 清空缓存，防止残留数据影响下次判定
            self.latest_angle = None
            self.latest_distance = None
            self.latest_occluded = None
            self.latest_range_uncertain = None
            self.last_detection_time_ns = None
            # 目标丢失，重置 Observation A + B
            self.observation_a = None
            self.observation_a_pending = False
            self.observation_a_warning_shown = False
            self.observation_a_angle_samples = []
            self.observation_b_goal = None
            self.observation_b_selection_done = False
            self.observation_b = None
            self.observation_b_nav_started = False
            self.observation_b_nav_active = False
            self.observation_b_pending = False
            self.observation_b_angle_samples = []
            # 路径验证状态也一并重置，保证下次重新验证
            self.observation_b_path_validation_started = False
            self.observation_b_path_valid = None
            # 定位结果也一并重置，保证下次三角定位能重新计算
            self.target_odom_position = None
            self.target_map_position = None
            self.target_localization_source = None
            # 阶段五 approach 状态也一并重置
            self.approach_pose = None
            self.approach_generation_done = False
            self.approach_candidates = []
            self.approach_candidate_index = 0
            self.approach_path_validation_started = False
            self.approach_path_valid = None
            # Stage 5.2/5.3 approach 导航与最终确认状态也一并重置
            self.approach_nav_started = False
            self.approach_nav_active = False
            self.approach_arrival_time_ns = None
            self.final_confirmation_done = False
            self.mission_success = False
            self.approach_iteration = 0

    # -----------------------------
    # TF2 机器人位姿查询
    # -----------------------------
    def get_robot_pose(self):
        """
        查询 base_footprint 在 map 中的位姿。
        返回 (x, y, yaw_rad) 或 None（TF 不可用时）。
        """

        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                Time()
            )
        except TransformException as e:
            # 仅第一次失败时打印 warning，避免 0.2s 重复刷屏
            if not self.pose_query_warning_shown:
                self.get_logger().warning(
                    f'Failed to get robot pose: {e}'
                )
                self.pose_query_warning_shown = True
            return None

        # 成功获取 TF，重置 warning flag
        self.pose_query_warning_shown = False

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y

        # 四元数 -> yaw（不依赖 tf_transformations）
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw)

        self.get_logger().info(
            f'[ROBOT POSE] x={robot_x:.2f} m, '
            f'y={robot_y:.2f} m, yaw={yaw_deg:.2f} deg'
        )

        return (robot_x, robot_y, yaw)

    # -----------------------------
    # TF2 相机位姿查询
    # 视觉射线必须从 Camera 光心出发，而不是 base_footprint
    # -----------------------------
    def get_camera_pose(self):
        """
        查询 camera_link 在 map 中的位姿。
        返回 (x, y, yaw_rad) 或 None（TF 不可用时）。
        使用 Time() 获取 latest transform（startup 更稳定，不依赖 now()）。
        """

        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                Time()
            )
        except TransformException as e:
            if not self.pose_query_warning_shown:
                self.get_logger().warning(
                    f'Failed to get camera pose: {e}'
                )
                self.pose_query_warning_shown = True
            return None

        # 成功获取 TF，重置 warning flag
        self.pose_query_warning_shown = False

        camera_x = tf.transform.translation.x
        camera_y = tf.transform.translation.y

        # 四元数 -> yaw（不依赖 tf_transformations）
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw)

        self.get_logger().info(
            f'[CAMERA POSE] x={camera_x:.2f} m, '
            f'y={camera_y:.2f} m, yaw={yaw_deg:.2f} deg'
        )

        return (camera_x, camera_y, yaw)

    # -----------------------------
    # TF2 odom 坐标系位姿查询
    # 短基线三角定位在 odom 中完成，避免 AMCL map->odom 动态修正
    # 在两次观测之间改变两条 Ray 的相对几何
    # -----------------------------
    def get_robot_pose_odom(self):
        """
        查询 base_footprint 在 odom 中的位姿。
        返回 (x, y, yaw_rad) 或 None（TF 不可用时，由 pending/timer 重试）。
        使用 Time() 获取 latest transform（不使用 now()）。
        """

        try:
            tf = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                Time()
            )
        except TransformException:
            return None

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y

        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (robot_x, robot_y, yaw)

    def get_camera_pose_odom(self):
        """
        查询 camera_link 在 odom 中的位姿。
        返回 (x, y, yaw_rad) 或 None（TF 不可用时，由 pending/timer 重试）。
        视觉 Ray 的 origin 与 bearing 均在 odom 中表达。
        """

        try:
            tf = self.tf_buffer.lookup_transform(
                'odom',
                'camera_link',
                Time()
            )
        except TransformException:
            return None

        camera_x = tf.transform.translation.x
        camera_y = tf.transform.translation.y

        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (camera_x, camera_y, yaw)

    # -----------------------------
    # 目标在 map 坐标系中的方向计算
    # -----------------------------
    def compute_target_map_bearing(self, robot_yaw, camera_angle_deg):
        """
        根据 机器人 map 朝向(robot_yaw, rad) + Camera 水平偏角(deg)
        计算目标在 map 坐标系中的绝对方向。

        符号关系（已实测验证）：
            Camera: angle<0 = 目标在左, angle>0 = 目标在右
            ROS  : +yaw = 左转
            =>   目标相对机器人方向 = -camera_angle_rad
        所以：
            target_map_yaw = robot_yaw - camera_angle_rad
        """

        # Camera 角度 deg -> rad
        camera_angle_rad = math.radians(camera_angle_deg)

        # 目标相对机器人方向（ROS 约定：+ = 左）
        target_relative_yaw = -camera_angle_rad

        # 目标在 map 中的绝对方向
        target_map_yaw = robot_yaw + target_relative_yaw

        # 归一化到 [-pi, pi]
        target_map_yaw = math.atan2(
            math.sin(target_map_yaw),
            math.cos(target_map_yaw)
        )

        target_map_yaw_deg = math.degrees(target_map_yaw)

        self.get_logger().info(
            f'[TARGET BEARING] '
            f'camera_angle={camera_angle_deg:.2f} deg, '
            f'robot_yaw={math.degrees(robot_yaw):.2f} deg, '
            f'target_map_yaw={target_map_yaw_deg:.2f} deg'
        )

        return target_map_yaw

    # -----------------------------
    # 换视角候选点计算
    # -----------------------------
    def compute_observation_candidates(
        self,
        robot_x,
        robot_y,
        target_map_yaw
    ):
        """
        根据机器人当前位置 + 目标方向，计算左右两个换视角候选点。

        forward_offset: 沿目标方向前移距离
        lateral_offset : 垂直目标方向的左右偏移距离
        """

        # 目标方向单位向量
        forward_x = math.cos(target_map_yaw)
        forward_y = math.sin(target_map_yaw)

        # 目标方向左侧垂直单位向量
        left_x = -math.sin(target_map_yaw)
        left_y =  math.cos(target_map_yaw)

        # 目标方向右侧垂直单位向量
        right_x =  math.sin(target_map_yaw)
        right_y = -math.cos(target_map_yaw)

        # 左候选观察点
        left_candidate_x = (
            robot_x
            + self.observation_forward_offset * forward_x
            + self.observation_lateral_offset * left_x
        )
        left_candidate_y = (
            robot_y
            + self.observation_forward_offset * forward_y
            + self.observation_lateral_offset * left_y
        )

        # 右候选观察点
        right_candidate_x = (
            robot_x
            + self.observation_forward_offset * forward_x
            + self.observation_lateral_offset * right_x
        )
        right_candidate_y = (
            robot_y
            + self.observation_forward_offset * forward_y
            + self.observation_lateral_offset * right_y
        )

        self.get_logger().info(
            f'[OBSERVATION CANDIDATES] '
            f'forward_offset={self.observation_forward_offset:.2f} m, '
            f'lateral_offset={self.observation_lateral_offset:.2f} m'
        )
        self.get_logger().info(
            f'    LEFT : x={left_candidate_x:.2f}, '
            f'y={left_candidate_y:.2f}'
        )
        self.get_logger().info(
            f'    RIGHT: x={right_candidate_x:.2f}, '
            f'y={right_candidate_y:.2f}'
        )

        return (
            (left_candidate_x, left_candidate_y),
            (right_candidate_x, right_candidate_y)
        )

    # -----------------------------
    # Global costmap cell cost 查询
    # -----------------------------
    def get_costmap_cost(self, x, y):
        """
        查询 (x, y) 在 global_costmap 中对应栅格的 cost。
        返回 int cost 或 None（costmap 未到/越界）。

        注意：使用 math.floor 而不是 int()，
        因为 int() 对负数向 0 截断（int(-0.8)=0），
        会把位于 origin 外侧的点错误映射到 grid 0。
        """

        if self.latest_global_costmap is None:
            return None

        info = self.latest_global_costmap.info
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        # map 坐标 -> 栅格索引（floor 保证负数正确向 -∞ 截断）
        grid_x = math.floor((x - origin_x) / resolution)
        grid_y = math.floor((y - origin_y) / resolution)

        # 严格越界检查
        if grid_x < 0 or grid_y < 0 or grid_x >= width or grid_y >= height:
            self.get_logger().warning(
                f'candidate outside global costmap: '
                f'grid=({grid_x}, {grid_y}), '
                f'size=({width} x {height})'
            )
            return None

        index = grid_y * width + grid_x
        cost = self.latest_global_costmap.data[index]

        return cost

    def _print_candidate_costs(self):
        """查询并打印左右候选点的 costmap cost。"""

        if (
            self.left_candidate is None
            or self.right_candidate is None
            or self.latest_global_costmap is None
        ):
            return

        left_x, left_y = self.left_candidate
        right_x, right_y = self.right_candidate

        left_cost = self.get_costmap_cost(left_x, left_y)
        right_cost = self.get_costmap_cost(right_x, right_y)

        self.get_logger().info(
            f'[CANDIDATE COSTS] '
            f'LEFT : x={left_x:.2f}, y={left_y:.2f}, cost={left_cost} | '
            f'RIGHT: x={right_x:.2f}, y={right_y:.2f}, cost={right_cost}'
        )

        self.candidate_costs_printed = True

    # -----------------------------
    # 候选点周围区域 cost 统计
    # -----------------------------
    def evaluate_candidate_region(self, x, y, radius=None):
        """
        统计 (x, y) 周围圆形区域（半径 candidate_check_radius）内的 costmap cost。

        参数:
            radius: 可选，覆盖默认 candidate_check_radius；
                    None 时使用 self.candidate_check_radius（保持向后兼容）

        返回 dict:
            若中心在 costmap 外: {outside=True, max_cost=None, ...,
                                   out_of_bounds_count}
            若有 valid_cost:      {outside=False, max_cost, mean_cost, ...}
            若无 valid_cost:      {outside=False, max_cost=None, valid_count=0, ...}
            若 costmap 未到:       None
        """

        if self.latest_global_costmap is None:
            return None

        info = self.latest_global_costmap.info
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        # 实际使用半径：显式参数优先，否则用默认 candidate_check_radius
        r_m = radius if radius is not None else self.candidate_check_radius

        # 中心栅格（floor 保证负数正确向 -∞ 截断）
        center_gx = math.floor((x - origin_x) / resolution)
        center_gy = math.floor((y - origin_y) / resolution)

        # 中心本身在 costmap 外 -> 直接返回 outside 结果
        if not (0 <= center_gx < width and 0 <= center_gy < height):
            return {
                'outside': True,
                'max_cost': None,
                'mean_cost': None,
                'valid_count': 0,
                'unknown_count': 0,
                'out_of_bounds_count': 1,  # 中心点越界
            }

        # 半径（cell 数）
        radius_cells = int(math.ceil(
            r_m / resolution
        ))

        # 圆形半径平方（m^2），用于精确圆形判断
        r_sq = r_m * r_m

        valid_costs = []
        unknown_count = 0
        out_of_bounds_count = 0

        # 遍历外接正方形
        for dgx in range(-radius_cells, radius_cells + 1):
            for dgy in range(-radius_cells, radius_cells + 1):

                gx = center_gx + dgx
                gy = center_gy + dgy

                # 越界 cell 计数后跳过
                if not (0 <= gx < width and 0 <= gy < height):
                    out_of_bounds_count += 1
                    continue

                # 精确圆形判断（用 m 为单位的距离）
                dx = dgx * resolution
                dy = dgy * resolution
                if dx * dx + dy * dy > r_sq:
                    continue

                index = gy * width + gx
                cost = self.latest_global_costmap.data[index]

                if cost == -1:
                    unknown_count += 1
                else:
                    valid_costs.append(cost)

        if not valid_costs:
            return {
                'outside': False,
                'max_cost': None,
                'mean_cost': None,
                'valid_count': 0,
                'unknown_count': unknown_count,
                'out_of_bounds_count': out_of_bounds_count,
            }

        return {
            'outside': False,
            'max_cost': max(valid_costs),
            'mean_cost': float(sum(valid_costs)) / len(valid_costs),
            'valid_count': len(valid_costs),
            'unknown_count': unknown_count,
            'out_of_bounds_count': out_of_bounds_count,
        }

    def _print_candidate_region(self):
        """查询并打印左右候选点周围区域的 cost 统计。"""

        if (
            self.left_candidate is None
            or self.right_candidate is None
            or self.latest_global_costmap is None
        ):
            return

        left_x, left_y = self.left_candidate
        right_x, right_y = self.right_candidate

        left_region = self.evaluate_candidate_region(left_x, left_y)
        right_region = self.evaluate_candidate_region(right_x, right_y)

        self.get_logger().info(
            f'[CANDIDATE REGION] '
            f'radius={self.candidate_check_radius:.2f} m'
        )

        for name, region in [('LEFT', left_region), ('RIGHT', right_region)]:
            if region is None:
                self.get_logger().info(f'    {name:5s}: costmap not received')
                continue

            if region.get('outside'):
                self.get_logger().info(
                    f'    {name:5s}: outside=True, '
                    f'max_cost=N/A, mean_cost=N/A, '
                    f'valid=0, unknown=0, '
                    f'out_of_bounds={region["out_of_bounds_count"]}'
                )
            elif region['max_cost'] is None:
                self.get_logger().info(
                    f'    {name:5s}: outside=False, '
                    f'max_cost=N/A, mean_cost=N/A, '
                    f'valid=0, unknown={region["unknown_count"]}, '
                    f'out_of_bounds={region["out_of_bounds_count"]}'
                )
            else:
                self.get_logger().info(
                    f'    {name:5s}: outside=False, '
                    f'max_cost={region["max_cost"]}, '
                    f'mean_cost={region["mean_cost"]:.1f}, '
                    f'valid={region["valid_count"]}, '
                    f'unknown={region["unknown_count"]}, '
                    f'out_of_bounds={region["out_of_bounds_count"]}'
                )

        self.candidate_region_printed = True

    # -----------------------------
    # 多偏移候选观察点搜索 + 安全过滤 + 选择
    # -----------------------------
    def search_observation_candidate(self, robot_x, robot_y, target_map_yaw):
        """
        沿左右侧向方向，使用多个 lateral_offset 生成候选观察点，
        利用 global costmap 检查安全性，筛选并选择最优候选。

        返回 dict 或 None：
            {side, offset, x, y, center_cost, max_cost, mean_cost, ...}
        """

        # 目标方向单位向量
        forward_x = math.cos(target_map_yaw)
        forward_y = math.sin(target_map_yaw)

        # 左/右侧垂直单位向量
        left_x = -math.sin(target_map_yaw)
        left_y =  math.cos(target_map_yaw)
        right_x =  math.sin(target_map_yaw)
        right_y = -math.cos(target_map_yaw)

        forward_offset = self.observation_forward_offset
        base_x = robot_x + forward_offset * forward_x
        base_y = robot_y + forward_offset * forward_y

        safe_candidates = []

        self.get_logger().info('[OBSERVATION SEARCH]')

        for side_name, side_x, side_y in [
            ('LEFT',  left_x,  left_y),
            ('RIGHT', right_x, right_y),
        ]:
            for offset in self.observation_lateral_offsets:

                cand_x = base_x + offset * side_x
                cand_y = base_y + offset * side_y

                center_cost = self.get_costmap_cost(cand_x, cand_y)
                region = self.evaluate_candidate_region(cand_x, cand_y)

                # 安全过滤
                rejected = False
                reason = ''

                if center_cost is None:
                    # 可能是 costmap 未到，也可能是 outside
                    if self.latest_global_costmap is not None:
                        rejected = True
                        reason = 'outside costmap'
                    else:
                        rejected = True
                        reason = 'costmap not received'

                elif center_cost >= 100:
                    rejected = True
                    reason = 'lethal center'

                elif region is None:
                    rejected = True
                    reason = 'region None'

                elif region.get('outside'):
                    rejected = True
                    reason = 'region outside costmap'

                elif region['max_cost'] is not None and region['max_cost'] >= 100:
                    rejected = True
                    reason = 'lethal cost inside candidate region'

                elif region['unknown_count'] > 0:
                    rejected = True
                    reason = 'unknown cells in candidate region'

                # 打印每个候选的评估结果
                if region is not None and not region.get('outside'):
                    self.get_logger().info(
                        f'{side_name:5s} offset={offset:.2f}: '
                        f'x={cand_x:.2f}, y={cand_y:.2f}, '
                        f'center_cost={center_cost}, '
                        f'max_cost={region["max_cost"]}, '
                        f'mean_cost={region["mean_cost"]:.1f}, '
                        f'valid={region["valid_count"]}, '
                        f'unknown={region["unknown_count"]}, '
                        f'out_of_bounds={region["out_of_bounds_count"]}, '
                        f'result={"REJECTED" if rejected else "SAFE"}, '
                        f'reason={reason}'
                    )
                else:
                    self.get_logger().info(
                        f'{side_name:5s} offset={offset:.2f}: '
                        f'x={cand_x:.2f}, y={cand_y:.2f}, '
                        f'center_cost={center_cost}, '
                        f'result={"REJECTED" if rejected else "SAFE"}, '
                        f'reason={reason}'
                    )

                if not rejected:
                    safe_candidates.append({
                        'side': side_name,
                        'offset': offset,
                        'x': cand_x,
                        'y': cand_y,
                        'center_cost': center_cost,
                        'max_cost': region['max_cost'],
                        'mean_cost': region['mean_cost'],
                        'valid_count': region['valid_count'],
                        'unknown_count': region['unknown_count'],
                        'out_of_bounds_count': region['out_of_bounds_count'],
                    })

        # 选择最优安全候选
        if not safe_candidates:
            self.get_logger().info(
                '[SELECTED OBSERVATION] NONE - no safe observation candidate found'
            )
            return None

        # 排序优先级：
        # 1) lateral_offset 最小
        # 2) mean_cost 最低
        # 3) center_cost 最低
        safe_candidates.sort(key=lambda c: (
            c['offset'],
            c['mean_cost'],
            c['center_cost'],
        ))

        best = safe_candidates[0]

        self.get_logger().info(
            f'[SELECTED OBSERVATION] '
            f'side={best["side"]}, offset={best["offset"]:.2f}, '
            f'x={best["x"]:.2f}, y={best["y"]:.2f}, '
            f'center_cost={best["center_cost"]}, '
            f'mean_cost={best["mean_cost"]:.1f}'
        )

        return best

    # -----------------------------
    # 前向扇区候选观察点搜索
    # -----------------------------
    def search_sector_observation_candidate(
        self,
        robot_x,
        robot_y,
        target_map_yaw
    ):
        """
        围绕目标视线方向，在机器人附近的前向扇区生成二维候选观察点。
        避开正前方（0°，已知有近障碍物），在 ±30°~±90° 范围内搜索。
        """

        safe_candidates = []

        self.get_logger().info('[SECTOR OBSERVATION SEARCH]')

        for radius in self.search_radii:
            for angle_offset_deg in self.search_angle_offsets_deg:

                # 相对目标视线的角度 -> map 绝对角度
                candidate_angle = target_map_yaw + math.radians(
                    angle_offset_deg
                )

                # 归一化到 [-pi, pi]
                candidate_angle = math.atan2(
                    math.sin(candidate_angle),
                    math.cos(candidate_angle)
                )

                cand_x = robot_x + radius * math.cos(candidate_angle)
                cand_y = robot_y + radius * math.sin(candidate_angle)

                center_cost = self.get_costmap_cost(cand_x, cand_y)
                region = self.evaluate_candidate_region(cand_x, cand_y)

                # 安全过滤
                safe = False
                reject_reason = ''

                if center_cost is None:
                    if self.latest_global_costmap is not None:
                        reject_reason = 'outside costmap'
                    else:
                        reject_reason = 'costmap not received'

                elif center_cost >= 100:
                    reject_reason = 'lethal center'

                elif region is None:
                    reject_reason = 'region None'

                elif region.get('outside'):
                    reject_reason = 'region outside costmap'

                elif (
                    region['max_cost'] is not None
                    and region['max_cost'] >= 100
                ):
                    reject_reason = 'lethal cost inside candidate region'

                elif region['unknown_count'] > 0:
                    reject_reason = 'unknown cells in candidate region'

                else:
                    safe = True

                # 打印每个候选评估结果
                if region is not None and not region.get('outside'):
                    self.get_logger().info(
                        f'r={radius:.2f}, offset={angle_offset_deg:+d} deg: '
                        f'x={cand_x:.2f}, y={cand_y:.2f}, '
                        f'center_cost={center_cost}, '
                        f'max_cost={region["max_cost"]}, '
                        f'mean_cost={region["mean_cost"]:.1f}, '
                        f'result={"SAFE" if safe else "REJECTED"}, '
                        f'reason={reject_reason}'
                    )
                else:
                    self.get_logger().info(
                        f'r={radius:.2f}, offset={angle_offset_deg:+d} deg: '
                        f'x={cand_x:.2f}, y={cand_y:.2f}, '
                        f'center_cost={center_cost}, '
                        f'result={"SAFE" if safe else "REJECTED"}, '
                        f'reason={reject_reason}'
                    )

                if safe:
                    safe_candidates.append({
                        'radius': radius,
                        'angle_offset_deg': angle_offset_deg,
                        'x': cand_x,
                        'y': cand_y,
                        'center_cost': center_cost,
                        'max_cost': region['max_cost'],
                        'mean_cost': region['mean_cost'],
                        'unknown_count': region['unknown_count'],
                        'out_of_bounds_count': region['out_of_bounds_count'],
                    })

        # 选择最优安全候选
        if not safe_candidates:
            self.get_logger().info(
                '[SELECTED SECTOR OBSERVATION] '
                'NONE - no safe sector candidate found'
            )
            return None

        # 优先级：
        # 1) radius 最小
        # 2) mean_cost 最低
        # 3) center_cost 最低
        safe_candidates.sort(key=lambda c: (
            c['radius'],
            c['mean_cost'],
            c['center_cost'],
        ))

        best = safe_candidates[0]

        self.get_logger().info(
            f'[SELECTED SECTOR OBSERVATION] '
            f'radius={best["radius"]:.2f}, '
            f'angle_offset={best["angle_offset_deg"]:+d} deg, '
            f'x={best["x"]:.2f}, y={best["y"]:.2f}, '
            f'center_cost={best["center_cost"]}, '
            f'mean_cost={best["mean_cost"]:.1f}'
        )

        # 保存选定的观察点（yaw 使用 target_map_yaw，到达后 Camera 会重新检测修正）
        self.selected_observation = {
            'x': best['x'],
            'y': best['y'],
            'yaw': target_map_yaw
        }

        # 尝试发送 Nav2 observation goal
        self.try_send_observation_goal()

        return best

    # -----------------------------
    # Nav2 NavigateToPose observation movement
    # -----------------------------
    def try_send_observation_goal(self):
        """
        在 REPOSITION_REQUIRED 状态下，若有 selected_observation 且未在导航中，
        发送 Nav2 NavigateToPose goal 到观察点。
        """

        if self.state != SearchState.REPOSITION_REQUIRED:
            return

        if self.selected_observation is None:
            return

        if self.nav_goal_active:
            return

        obs = self.selected_observation
        self.send_observation_goal(obs['x'], obs['y'], obs['yaw'])

    def send_observation_goal(self, x, y, yaw):
        """
        构造 PoseStamped + NavigateToPose.Goal 并异步发送。
        """

        if not self.nav_client.server_is_ready():
            self.get_logger().warning(
                '[OBSERVATION NAV] Nav2 action server not ready, skip sending'
            )
            return

        # 构造 PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # yaw -> quaternion (仅绕 Z 轴)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        yaw_deg = math.degrees(yaw)
        self.get_logger().info(
            f'[OBSERVATION NAV] sending goal: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.2f} deg'
        )

        # 构造 goal
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.nav_goal_active = True
        self.selected_observation = None  # 消费掉，防止重发

        # 进入 MOVING_TO_OBSERVATION 状态
        self.transition_to(
            SearchState.MOVING_TO_OBSERVATION,
            'navigating to observation point'
        )

        # 异步发送
        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def _nav_feedback_callback(self, feedback_msg):
        """Nav2 反馈回调（当前不处理 feedback）。"""
        pass

    def goal_response_callback(self, future):
        """Nav2 goal 响应回调：accepted / rejected。"""

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(
                '[OBSERVATION NAV] goal rejected by Nav2'
            )
            self.nav_goal_active = False
            # rejected 也回到 SEARCHING 重新感知，不立即重发
            self.latest_angle = None
            self.latest_distance = None
            self.latest_occluded = None
            self.latest_range_uncertain = None
            self.last_detection_time_ns = None
            self.transition_to(
                SearchState.SEARCHING,
                'observation goal rejected, re-observing from current pose'
            )
            return

        self.get_logger().info('[OBSERVATION NAV] goal accepted')

        self.current_goal_handle = goal_handle

        # 等待导航结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Nav2 导航结果回调。"""

        self.nav_goal_active = False
        self.current_goal_handle = None

        status = future.result().status if future.result() else None

        # 无论成功还是失败，都清空移动过程中缓存的旧感知数据，
        # 然后回到 SEARCHING 等待新的 Camera/LiDAR 消息。
        # 这样防止 Nav2 abort 后立即用旧 perception 触发新一轮 reposition，
        # 也避免基于已变化的实际位置但未刷新的 occluded 做错误判定。
        self.latest_angle = None
        self.latest_distance = None
        self.latest_occluded = None
        self.latest_range_uncertain = None
        self.last_detection_time_ns = None

        # status 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info(
                '[OBSERVATION NAV] goal succeeded'
            )
            self.transition_to(
                SearchState.SEARCHING,
                'observation point reached, waiting for fresh perception'
            )
        else:
            self.get_logger().warning(
                f'[OBSERVATION NAV] goal aborted, status={status}'
            )
            self.transition_to(
                SearchState.SEARCHING,
                'observation navigation ended, re-observing from current pose'
            )

    # =====================================================
    # Observation B：Nav2 移动 + 到达后真实位姿记录
    # 与旧 active-perception observation movement 完全分离，
    # 使用独立的状态变量与回调，不复用 REPOSITION_REQUIRED 逻辑。
    # =====================================================

    # =====================================================
    # Observation B Goal 路径验证（ComputePathToPose, dry-run）
    # 只验证路径是否存在 + 是否经过高 cost 区域，不移动机器人
    # =====================================================
    def validate_observation_b_path(self):
        """
        对当前选定的 Observation B Goal 发送 ComputePathToPose，
        验证从当前位置到 B 的路径是否有效且不经过高 cost 区域。
        本阶段不发送 NavigateToPose，机器人保持静止。
        """

        # 去重 guard：已开始验证则等待结果
        if self.observation_b_path_validation_started:
            return
        if self.observation_b_path_valid is not None:
            return  # 已有结论，不重复验证
        if self.observation_b_goal is None:
            return

        # Action server 未 ready 时等待，不标记失败
        if not self.compute_path_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(
                '[VIEWPOINT PATH] ComputePathToPose server not ready, waiting'
            )
            return

        # 构造 goal（map frame，使用当前机器人位置作为起点）
        x = self.observation_b_goal['x']
        y = self.observation_b_goal['y']
        yaw = self.observation_b_goal['yaw']

        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        # yaw -> quaternion
        goal.goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(yaw / 2.0)
        goal.use_start = False  # 使用当前机器人 pose
        goal.planner_id = ''

        self.observation_b_path_validation_started = True
        self.get_logger().info(
            f'[VIEWPOINT PATH] sending ComputePathToPose:\n'
            f'    goal_x={x:.2f}\n'
            f'    goal_y={y:.2f}\n'
            f'    goal_yaw={math.degrees(yaw):.2f} deg'
        )

        send_future = self.compute_path_client.send_goal_async(
            goal
        )
        send_future.add_done_callback(
            self._observation_b_path_goal_response_callback
        )

    def _observation_b_path_goal_response_callback(self, future):
        """ComputePathToPose goal 响应回调：accepted / rejected。"""

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info(
                '[VIEWPOINT PATH REJECTED]\n'
                '    reason=planner goal rejected'
            )
            self.observation_b_path_valid = False
            return

        self.get_logger().info('[VIEWPOINT PATH] goal accepted, waiting for result')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self._observation_b_path_result_callback
        )

    def _observation_b_path_result_callback(self, future):
        """
        ComputePathToPose 结果回调。
        检查：path 存在、poses >= 2、所有 pose cost < 80、无 None。
        """

        result = future.result().result

        # 无有效 path
        if result is None or result.path is None or len(result.path.poses) < 2:
            self.get_logger().info(
                '[VIEWPOINT PATH REJECTED]\n'
                '    reason=no valid path returned'
            )
            self.observation_b_path_valid = False
            return

        path_poses = result.path.poses
        path_pose_count = len(path_poses)

        # 计算 path_length（相邻 pose 距离累加）
        path_length = 0.0
        for i in range(1, path_pose_count):
            dx = path_poses[i].pose.position.x - path_poses[i - 1].pose.position.x
            dy = path_poses[i].pose.position.y - path_poses[i - 1].pose.position.y
            path_length += math.hypot(dx, dy)

        # 检查每个 path pose 在 global costmap 中的 cost
        path_costs = []
        path_max_cost = 0
        cost_sum = 0
        valid_cost_count = 0
        path_cost_threshold = 80  # 与 Observation B 严格安全一致

        for pose in path_poses:
            cost = self.get_costmap_cost(
                pose.pose.position.x,
                pose.pose.position.y
            )
            if cost is None:
                self.get_logger().info(
                    f'[VIEWPOINT PATH REJECTED]\n'
                    f'    goal_x={self.observation_b_goal["x"]:.2f}\n'
                    f'    goal_y={self.observation_b_goal["y"]:.2f}\n'
                    f'    reason=path pose outside costmap\n'
                    f'    path_max_cost={path_max_cost}'
                )
                self.observation_b_path_valid = False
                return

            path_costs.append(cost)
            if cost > path_max_cost:
                path_max_cost = cost
            cost_sum += cost
            valid_cost_count += 1

            if cost >= path_cost_threshold:
                self.get_logger().info(
                    f'[VIEWPOINT PATH REJECTED]\n'
                    f'    goal_x={self.observation_b_goal["x"]:.2f}\n'
                    f'    goal_y={self.observation_b_goal["y"]:.2f}\n'
                    f'    reason=high-cost cell along planned path\n'
                    f'    path_max_cost={path_max_cost}'
                )
                self.observation_b_path_valid = False
                return

        path_mean_cost = cost_sum / valid_cost_count if valid_cost_count > 0 else 0.0

        # 全部通过：路径有效
        self.observation_b_path_valid = True
        self.get_logger().info(
            f'[VIEWPOINT PATH VALID]\n'
            f'    goal_x={self.observation_b_goal["x"]:.2f}\n'
            f'    goal_y={self.observation_b_goal["y"]:.2f}\n'
            f'    path_length={path_length:.2f} m\n'
            f'    path_pose_count={path_pose_count}\n'
            f'    path_max_cost={path_max_cost}\n'
            f'    path_mean_cost={path_mean_cost:.1f}'
        )
        # 路径验证通过，timer 将自动触发 send_observation_b_goal()

    def send_observation_b_goal(self):
        """
        observation_b_goal 选择完成后，发送 Nav2 NavigateToPose goal。
        使用独立回调，避免与旧 observation movement 混在一起。
        """

        # 去重保护
        if self.observation_b_nav_started:
            return
        if self.observation_b_nav_active:
            return
        # 避免与其他 Nav2 goal 冲突
        if self.nav_goal_active:
            return

        if self.observation_b_goal is None:
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warning(
                '[OBSERVATION B NAV] Nav2 action server not ready, skip sending'
            )
            return

        goal_data = self.observation_b_goal
        x = goal_data['x']
        y = goal_data['y']
        yaw = goal_data['yaw']

        # 构造 PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # yaw -> quaternion (仅绕 Z 轴)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        yaw_deg = math.degrees(yaw)
        self.get_logger().info(
            f'[OBSERVATION B NAV] sending goal:\n'
            f'    x={x:.2f}\n'
            f'    y={y:.2f}\n'
            f'    yaw={yaw_deg:.2f} deg'
        )

        # 构造 goal
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # 发送前置位：防止重复发送
        self.observation_b_nav_started = True
        self.observation_b_nav_active = True

        # 进入 MOVING_TO_OBSERVATION_B：冻结普通语义状态机
        self.transition_to(
            SearchState.MOVING_TO_OBSERVATION_B,
            'moving to second triangulation viewpoint'
        )

        # 异步发送（使用独立回调）
        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._observation_b_nav_feedback_callback
        )
        send_future.add_done_callback(
            self.observation_b_goal_response_callback
        )

    def _observation_b_nav_feedback_callback(self, feedback_msg):
        """Observation B Nav2 反馈回调（当前不处理 feedback）。"""
        pass

    def observation_b_goal_response_callback(self, future):
        """Observation B Nav2 goal 响应回调：accepted / rejected。"""

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning('[OBSERVATION B NAV] goal rejected')
            # 不自动无限重发，保持 observation_b=None
            self.observation_b_nav_active = False
            return

        self.get_logger().info('[OBSERVATION B NAV] goal accepted')

        # 等待导航结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.observation_b_navigation_result_callback
        )

    def observation_b_navigation_result_callback(self, future):
        """Observation B Nav2 导航结果回调。"""

        self.observation_b_nav_active = False

        status = future.result().status if future.result() else None

        # status 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info(
                '[OBSERVATION B NAV] goal succeeded'
            )
            # 清空移动过程中的旧感知缓存，必须等待新的 Camera angle
            self.latest_angle = None
            self.latest_distance = None
            self.latest_range_uncertain = None
            self.last_detection_time_ns = None
            # 清空移动过程中可能残留的 B samples，
            # 只收集停车后 WAITING_FOR_OBSERVATION_B 期间的新帧
            self.observation_b_angle_samples = []
            # 进入 WAITING_FOR_OBSERVATION_B：等待新 Camera detection
            self.transition_to(
                SearchState.WAITING_FOR_OBSERVATION_B,
                'reached second viewpoint, waiting for fresh camera detection'
            )
        else:
            self.get_logger().warning(
                f'[OBSERVATION B NAV] goal aborted, status={status}'
            )
            # 第一版：不把失败位置当 Observation B，保持 None，停止自动流程
            # 不重新发 goal，不进入旧 REPOSITION_REQUIRED，不 sector search

    def try_record_observation_b(self):
        """
        在 WAITING_FOR_OBSERVATION_B 状态下，收满 5 帧 camera_angle 后，
        通过 TF 获取机器人真实位姿并记录 Observation B + Ray B。
        使用 5 帧 median 抑制单帧抖动。
        样本不足或 TF 失败时保持 pending=True，下次 timer 重试。
        """

        if self.observation_b is not None:
            return  # 已记录
        if not self.observation_b_pending:
            return
        # 样本未收满，等待更多新帧（不在 timer 中重复采样）
        if len(self.observation_b_angle_samples) < self.bearing_sample_count:
            return

        pose = self.get_robot_pose()
        if pose is None:
            # TF 不可用，保持 pending，下次 timer 重试（不清空已收好的样本）
            return

        # Camera 光心 TF（map）：视觉射线必须从 camera_link 出发，不是 base_footprint
        cam_pose = self.get_camera_pose()
        if cam_pose is None:
            # Camera TF 不可用，保持 pending，下次 timer 重试（不清空已收好的样本）
            return

        # odom 位姿：短基线三角定位统一在 odom 中完成
        pose_odom = self.get_robot_pose_odom()
        if pose_odom is None:
            # odom TF 不可用，保持 pending，下次 timer 重试（不清空已收好的样本）
            return
        cam_pose_odom = self.get_camera_pose_odom()
        if cam_pose_odom is None:
            return

        # 5 帧 median 抑制单帧抖动
        camera_angle_filtered = statistics.median(
            self.observation_b_angle_samples
        )
        self.get_logger().info(
            f'[OBS B FILTER]\n'
            f'    samples={[round(a, 2) for a in self.observation_b_angle_samples]}\n'
            f'    median_angle={camera_angle_filtered:.2f} deg'
        )

        # TF 成功，计算 Observation B（使用真实位姿，不是 b_goal 计划值）
        robot_x, robot_y, robot_yaw = pose
        camera_x, camera_y, _ = cam_pose

        target_map_bearing = self.compute_target_map_bearing(
            robot_yaw,
            camera_angle_filtered
        )

        # odom 字段：Ray B origin/bearing 全部在 odom 中表达
        robot_odom_x, robot_odom_y, robot_odom_yaw = pose_odom
        camera_odom_x, camera_odom_y, camera_odom_yaw = cam_pose_odom
        # 图像约定：camera_angle + = right；ROS yaw + = left
        # => 目标相对 Camera 方向 = -camera_angle
        target_odom_bearing = math.atan2(
            math.sin(camera_odom_yaw - math.radians(camera_angle_filtered)),
            math.cos(camera_odom_yaw - math.radians(camera_angle_filtered))
        )

        # actual_baseline：机器人 base_footprint A->B 在 map 中的实际移动距离
        actual_baseline = math.hypot(
            robot_x - self.observation_a['robot_x'],
            robot_y - self.observation_a['robot_y']
        )
        # camera_baseline：Camera 光心 A->B 在 odom 中的实际移动距离
        # （triangulation 在 odom 中进行，视觉 baseline 也用 odom 表达）
        camera_baseline = math.hypot(
            camera_odom_x - self.observation_a['camera_odom_x'],
            camera_odom_y - self.observation_a['camera_odom_y']
        )

        self.observation_b = {
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_yaw': robot_yaw,
            'camera_x': camera_x,
            'camera_y': camera_y,
            'camera_angle_deg': camera_angle_filtered,
            'target_map_bearing': target_map_bearing,
            'actual_baseline': actual_baseline,
            'camera_baseline': camera_baseline,
            # odom 几何（triangulation 实际使用）
            'robot_odom_x': robot_odom_x,
            'robot_odom_y': robot_odom_y,
            'robot_odom_yaw': robot_odom_yaw,
            'camera_odom_x': camera_odom_x,
            'camera_odom_y': camera_odom_y,
            'camera_odom_yaw': camera_odom_yaw,
            'target_odom_bearing': target_odom_bearing
        }
        self.observation_b_pending = False

        # 打印 Observation B + Ray B ODOM（只打一次，Ray 在 odom 坐标系中）
        bearing_deg = math.degrees(target_map_bearing)
        odom_bearing_deg = math.degrees(target_odom_bearing)
        dir_x = math.cos(target_odom_bearing)
        dir_y = math.sin(target_odom_bearing)
        self.get_logger().info(
            f'[OBSERVATION B]\n'
            f'    robot_x={robot_x:.2f} m\n'
            f'    robot_y={robot_y:.2f} m\n'
            f'    robot_yaw={math.degrees(robot_yaw):.2f} deg\n'
            f'    camera_x={camera_x:.2f} m\n'
            f'    camera_y={camera_y:.2f} m\n'
            f'    camera_angle={camera_angle_filtered:.2f} deg\n'
            f'    target_map_bearing={bearing_deg:.2f} deg\n'
            f'    actual_baseline={actual_baseline:.2f} m\n'
            f'    camera_baseline={camera_baseline:.2f} m\n'
            f'    camera_odom_x={camera_odom_x:.2f} m\n'
            f'    camera_odom_y={camera_odom_y:.2f} m\n'
            f'    camera_odom_yaw={math.degrees(camera_odom_yaw):.2f} deg\n'
            f'    target_odom_bearing={odom_bearing_deg:.2f} deg\n'
            f'[RAY B ODOM]\n'
            f'    origin=({camera_odom_x:.2f}, {camera_odom_y:.2f})\n'
            f'    bearing={odom_bearing_deg:.2f} deg\n'
            f'    direction_x={dir_x:.3f}\n'
            f'    direction_y={dir_y:.3f}'
        )

        # Observation A + B 均已就绪，执行 Ray A / Ray B odom 2D 求交
        self.get_logger().info(
            '[TRIANGULATION] Observation A and B ready'
        )
        self.triangulate_target()

    # =====================================================
    # Ray A + Ray B 2D 求交：在 odom 坐标系中计算 bottle 位置
    # 射线模型 A + t*da / B + u*db，要求交点在两条射线前方
    # 只计算一次（guard: target_odom_position is None）
    # odom 结果得到后，只做一次最新 map<-odom TF 转换得到 target_map_position
    # =====================================================
    @staticmethod
    def cross_2d(a, b):
        """二维叉积：a × b = a.x * b.y - a.y * b.x"""
        return a[0] * b[1] - a[1] * b[0]

    def triangulate_target(self):
        """
        Ray A + Ray B 在 odom 中 2D 求交，得到 target_odom_position。
        有效性检查：近似平行 / 视差不足 / 交点位于射线后方 均判定失败。
        数学公式与 map 版本完全一致，仅输入改为 odom 几何。
        """

        # 去重 guard：odom 求交只做一次，避免 timer 周期重复刷日志
        if self.target_odom_position is not None:
            return

        # A、B 任一未记录，无法求交
        if self.observation_a is None or self.observation_b is None:
            return

        # Observation A（theta 已是 rad，不再次转换）
        # 射线起点使用 Camera 光心 odom 坐标，bearing 使用 target_odom_bearing
        ax = self.observation_a['camera_odom_x']
        ay = self.observation_a['camera_odom_y']
        theta_a = self.observation_a['target_odom_bearing']

        # Observation B（theta 已是 rad，不再次转换）
        bx = self.observation_b['camera_odom_x']
        by = self.observation_b['camera_odom_y']
        theta_b = self.observation_b['target_odom_bearing']

        # 单位方向向量
        da = (math.cos(theta_a), math.sin(theta_a))
        db = (math.cos(theta_b), math.sin(theta_b))

        # B - A 向量
        ba = (bx - ax, by - ay)

        # 射线模型：A + t*da, B + u*db
        denominator = self.cross_2d(da, db)

        # -----------------------------
        # 有效性检查 1：近似平行（denominator 过小无法稳定求交）
        # -----------------------------
        if abs(denominator) < 1e-3:
            self.get_logger().info(
                '[TRIANGULATION FAILED] rays are nearly parallel'
            )
            return

        # 求交参数 t / u
        t = self.cross_2d(ba, db) / denominator
        u = self.cross_2d(ba, da) / denominator

        # -----------------------------
        # 射线夹角（两 odom bearing 最小角差）
        # -----------------------------
        ray_angle = abs(
            math.atan2(
                math.sin(theta_a - theta_b),
                math.cos(theta_a - theta_b)
            )
        )

        # -----------------------------
        # 有效性检查 2：视差太小
        # -----------------------------
        min_ray_angle = math.radians(3.0)
        if ray_angle < min_ray_angle:
            self.get_logger().info(
                f'[TRIANGULATION FAILED] insufficient parallax: '
                f'{math.degrees(ray_angle):.2f} deg'
            )
            return

        # -----------------------------
        # 有效性检查 3：交点必须位于两条射线前方
        # -----------------------------
        if t <= 0.0 or u <= 0.0:
            self.get_logger().info(
                f'[TRIANGULATION FAILED] intersection behind camera: '
                f't={t:.2f}, u={u:.2f}'
            )
            return

        # 交点（odom 坐标）
        target_odom_x = ax + t * da[0]
        target_odom_y = ay + t * da[1]

        # 保存 odom 定位结果
        self.target_odom_position = {
            'x': target_odom_x,
            'y': target_odom_y
        }

        # 只打印一次 odom 结果
        self.get_logger().info(
            f'[TRIANGULATION RESULT ODOM]\n'
            f'    target_odom_x={target_odom_x:.2f} m\n'
            f'    target_odom_y={target_odom_y:.2f} m\n'
            f'    distance_from_A={t:.2f} m\n'
            f'    distance_from_B={u:.2f} m\n'
            f'    ray_angle={math.degrees(ray_angle):.2f} deg'
        )

        # odom triangulation 成功后，立即尝试一次性转换到 map；
        # map<-odom TF 暂时不可用时不丢弃 odom 结果，由 timer retry
        self.try_transform_target_to_map()

    def try_transform_target_to_map(self):
        """
        将 target_odom_position 通过最新 map<-odom TF 一次性转换到 map。
        TF 暂时不可用时保持 target_map_position=None，由 timer 后续重试，
        不重新 triangulate。
        """

        if self.target_map_position is not None:
            return  # 已转换完成
        if self.target_odom_position is None:
            return  # odom triangulation 尚未成功

        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                Time()
            )
        except TransformException:
            # map<-odom 暂不可用（AMCL 尚未发布），保持 pending，timer retry
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_map_odom = math.atan2(siny_cosp, cosy_cosp)

        ox = self.target_odom_position['x']
        oy = self.target_odom_position['y']

        # odom -> map 刚体变换（旋转 + 平移）
        target_map_x = (
            tx
            + math.cos(yaw_map_odom) * ox
            - math.sin(yaw_map_odom) * oy
        )
        target_map_y = (
            ty
            + math.sin(yaw_map_odom) * ox
            + math.cos(yaw_map_odom) * oy
        )

        self.target_map_position = {
            'x': target_map_x,
            'y': target_map_y
        }

        self.get_logger().info(
            f'[TARGET MAP POSITION]\n'
            f'    target_map_x={target_map_x:.2f} m\n'
            f'    target_map_y={target_map_y:.2f} m\n'
            f'    map_odom_yaw={math.degrees(yaw_map_odom):.2f} deg\n'
            f'    source=active_triangulation'
        )
        self.target_localization_source = 'active_triangulation'

    # =====================================================
    # Stage 4 PRIMARY：Camera bearing + reliable LiDAR range 直接定位
    # 仅当 range_uncertain==False 且 range 合法时，从 map->camera_link 光心
    # 沿目标方向投射 range 得到 target_map_position。
    # range_uncertain==True / range 非法时不执行，由既有 Obs A/B 三角定位 fallback。
    # =====================================================
    def direct_target_localization(self):
        """
        直接定位：camera_link 光心 + target bearing + reliable LiDAR range。
        - TF 暂不可用：返回，由 timer 重试（不切 fallback）
        - range 不可靠/非法：打印 SKIPPED 并返回（range_uncertain=True 时
          由现有 maybe_start_observation_a 走三角定位 fallback）
        幂等：target_map_position 已存在或三角 fallback 已开始时直接返回。
        """

        # 已完成定位（直接或三角）-> 不重复
        if self.target_map_position is not None:
            return
        # 三角定位 fallback 已在进行 -> 不切换到 direct
        if self.observation_a is not None or self.observation_a_pending:
            return
        if self.observation_b is not None:
            return
        # 需要有效 camera bearing + 可靠 range
        if self.latest_angle is None:
            return
        if self.latest_range_uncertain is not False:
            return

        r = self.latest_distance
        # range 合法性：有限值，0.10m < r < 10.0m（LiDAR max range）
        if (
            r is None
            or not math.isfinite(r)
            or r <= 0.10
            or r >= 10.0
        ):
            self.get_logger().info(
                f'[DIRECT TARGET LOCALIZATION SKIPPED] '
                f'reason=invalid range r={r}',
                throttle_duration_sec=2.0
            )
            return

        # map -> camera_link 光心（latest transform，不使用 now()）
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                Time()
            )
        except TransformException:
            # TF 暂时不可用，保持重试，不切 fallback
            return

        camera_x = tf.transform.translation.x
        camera_y = tf.transform.translation.y

        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        camera_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 图像约定：camera_angle + = right；ROS yaw + = left
        # => 目标相对 Camera 方向 = -camera_angle
        angle_rad = math.radians(self.latest_angle)
        bearing = math.atan2(
            math.sin(camera_yaw - angle_rad),
            math.cos(camera_yaw - angle_rad)
        )

        # 从 Camera 光心沿 bearing 投射 range
        target_map_x = camera_x + r * math.cos(bearing)
        target_map_y = camera_y + r * math.sin(bearing)

        self.target_localization_source = 'camera_lidar_direct'
        self.target_map_position = {
            'x': target_map_x,
            'y': target_map_y,
            'source': 'camera_lidar_direct'
        }

        self.get_logger().info(
            f'[DIRECT TARGET LOCALIZATION]\n'
            f'    source=camera_lidar\n'
            f'    camera_x={camera_x:.2f}\n'
            f'    camera_y={camera_y:.2f}\n'
            f'    camera_yaw={math.degrees(camera_yaw):.2f} deg\n'
            f'    camera_angle={self.latest_angle:.2f} deg\n'
            f'    target_bearing={math.degrees(bearing):.2f} deg\n'
            f'    range={r:.2f} m\n'
            f'    target_map_x={target_map_x:.2f}\n'
            f'    target_map_y={target_map_y:.2f}'
        )
        self.get_logger().info(
            f'[TARGET MAP POSITION]\n'
            f'    target_map_x={target_map_x:.2f} m\n'
            f'    target_map_y={target_map_y:.2f} m\n'
            f'    source=camera_lidar_direct'
        )

    # =====================================================
    # 阶段五 Stage 5.1：Progress-based safe approach planner
    # 以当前机器人位置为中心、向粗目标区域推进生成候选，
    # 候选必须离粗目标 >=1.0m（remaining_to_target），
    # 通过保守 costmap safety 后按优先级排序，
    # 再由 validate_approach_path() 逐个 ComputePathToPose 验证。
    # 本阶段 dry-run：不发送 NavigateToPose，机器人不移动。
    # =====================================================
    def generate_approach_pose(self):
        """
        PROGRESS-BASED SAFE APPROACH PLANNER。
        以机器人当前位置为中心，沿 target 方向用多个 forward_distance ×
        angular offset 生成候选，候选朝向粗目标且 remaining_to_target>=1.0m，
        本地 costmap 安全检查通过后排序存入 approach_candidates。
        TF / costmap 不可用时 retry，不直接失败。
        """

        if self.approach_generation_done:
            return
        if self.target_map_position is None:
            return

        # 当前机器人 map pose（latest transform，不使用 now()）
        pose = self.get_robot_pose()
        if pose is None:
            # TF 不可用，保持未完成，下次 timer retry
            return

        if self.latest_global_costmap is None:
            # costmap 未到，等待下次
            return

        self.approach_generation_done = True

        rx, ry, _ = pose
        tx = self.target_map_position['x']
        ty = self.target_map_position['y']

        dx = tx - rx
        dy = ty - ry
        distance = math.hypot(dx, dy)

        # 已经在目标估计区域附近：不生成前进 approach pose，停止自动流程
        if distance < 1.2:
            self.get_logger().info(
                '[APPROACH PLANNER] robot already near estimated target region'
            )
            return

        # -----------------------------
        # PROGRESS-BASED SAFE APPROACH PLANNER（Stage 5.1 ACTIVE）
        # 候选以“当前机器人位置”为中心，向粗目标区域推进，
        # 不是围绕 target_map_position 画圆。
        # -----------------------------
        forward_distances = [0.8, 1.2, 1.6, 2.0]
        angular_offsets_deg = [-45, -30, -15, 0, 15, 30, 45]

        # 机器人 -> 粗目标方向
        target_direction = math.atan2(ty - ry, tx - rx)

        # 保守 costmap 安全标准（不降低阈值）
        approach_clearance_radius = 0.35
        center_cost_threshold = 50
        region_max_cost_threshold = 80
        # 粗定位不可信，候选必须离粗目标估计点至少 1.0m
        min_remaining_to_target = 1.0

        safe_candidates = []

        for forward_distance in forward_distances:
            for offset_deg in angular_offsets_deg:
                # 以 ROBOT 当前位置为圆心向前推进
                candidate_direction = target_direction + math.radians(offset_deg)
                cx = rx + forward_distance * math.cos(candidate_direction)
                cy = ry + forward_distance * math.sin(candidate_direction)
                # 到达后朝向粗目标区域
                cyaw = math.atan2(ty - cy, tx - cx)

                # 候选距粗目标估计点的剩余距离（必须保留 safety margin）
                remaining_to_target = math.hypot(tx - cx, ty - cy)

                center_cost = self.get_costmap_cost(cx, cy)
                region = self.evaluate_candidate_region(
                    cx, cy, radius=approach_clearance_radius
                )

                max_cost = region.get('max_cost') if region else None
                mean_cost = region.get('mean_cost') if region else None

                result = 'REJECTED'
                reason = ''

                if remaining_to_target < min_remaining_to_target:
                    reason = 'too close to coarse target estimate'
                elif center_cost is None:
                    reason = 'approach pose outside map'
                elif region is None:
                    reason = 'region evaluation None'
                elif region.get('outside'):
                    reason = 'region outside costmap'
                elif region.get('unknown_count', 0) > 0:
                    reason = 'unknown cells in approach region'
                elif region.get('out_of_bounds_count', 0) > 0:
                    reason = 'out_of_bounds cells in approach region'
                elif center_cost >= center_cost_threshold:
                    reason = 'high center inflation cost'
                elif (
                    max_cost is not None
                    and max_cost >= region_max_cost_threshold
                ):
                    reason = 'obstacle too close to approach footprint'
                else:
                    result = 'SAFE'

                self.get_logger().info(
                    f'[APPROACH CANDIDATE]\n'
                    f'    forward={forward_distance:.2f} m\n'
                    f'    offset={offset_deg:+d} deg\n'
                    f'    x={cx:.2f}\n'
                    f'    y={cy:.2f}\n'
                    f'    yaw={math.degrees(cyaw):.2f} deg\n'
                    f'    remaining_to_target={remaining_to_target:.2f} m\n'
                    f'    center_cost={center_cost}\n'
                    f'    max_cost={max_cost}\n'
                    f'    mean_cost={mean_cost if mean_cost is not None else "N/A"}\n'
                    f'    result={result}\n'
                    f'    reason={reason}'
                )

                if result == 'SAFE':
                    safe_candidates.append({
                        'x': cx,
                        'y': cy,
                        'yaw': cyaw,
                        'forward_distance': forward_distance,
                        'offset_deg': offset_deg,
                        'remaining_to_target': remaining_to_target,
                        'center_cost': center_cost,
                        'max_cost': max_cost if max_cost is not None else 9999,
                        'mean_cost': mean_cost if mean_cost is not None else 9999.0,
                    })

        # 没有任何 local safe 候选：标记全部失败（后续不再 ring planner）
        if not safe_candidates:
            self.get_logger().info(
                '[APPROACH PLANNER FAILED]\n'
                '    reason=no reachable safe progress candidate'
            )
            self.approach_path_valid = False
            return

        # -----------------------------
        # 候选排序（只在 LOCAL SAFE candidates 中）：
        # 1. remaining_to_target 越小越优先（尽可能向粗目标推进）
        # 2. max_cost 越低越优先
        # 3. mean_cost 越低越优先
        # 4. |offset_deg| 越小越优先
        # 路径验证阶段会按此顺序逐个尝试 ComputePathToPose。
        # -----------------------------
        self.approach_candidates = sorted(
            safe_candidates,
            key=lambda c: (
                c['remaining_to_target'],
                c['max_cost'],
                c['mean_cost'],
                abs(c['offset_deg'])
            )
        )
        self.approach_candidate_index = 0
        self.get_logger().info(
            f'[APPROACH PLANNER] {len(self.approach_candidates)} local-safe '
            f'progress candidates, starting path validation'
        )

    def validate_approach_path(self):
        """
        按排序后的 LOCAL SAFE 候选逐个发送 ComputePathToPose：
        某个候选 path 验证失败则自动尝试下一个，直到找到 LOCAL SAFE + PATH VALID。
        复用阶段四 compute_path_client。Stage 5.1 dry-run：不发送 NavigateToPose。
        """

        if self.approach_path_validation_started:
            return  # 有 goal 在途
        if self.approach_path_valid is not None:
            return  # 已成功或已全部失败

        # 取出下一个待验证候选（上一个失败后 approach_pose 被清空）
        if self.approach_pose is None:
            if self.approach_candidate_index >= len(self.approach_candidates):
                self.approach_path_valid = False
                self.get_logger().info(
                    '[APPROACH PLANNER FAILED]\n'
                    '    reason=no reachable safe progress candidate'
                )
                return
            cand = self.approach_candidates[self.approach_candidate_index]
            self.approach_pose = dict(cand)

        # Action server 未 ready 时等待（保留 approach_pose，下次重试同一候选）
        if not self.compute_path_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(
                '[APPROACH PATH] ComputePathToPose server not ready, waiting'
            )
            return

        x = self.approach_pose['x']
        y = self.approach_pose['y']
        yaw = self.approach_pose['yaw']

        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(yaw / 2.0)
        goal.use_start = False
        goal.planner_id = ''

        self.approach_path_validation_started = True
        self.get_logger().info(
            f'[APPROACH PATH] sending ComputePathToPose '
            f'(candidate {self.approach_candidate_index + 1}/'
            f'{len(self.approach_candidates)}):\n'
            f'    forward={self.approach_pose["forward_distance"]:.2f} m\n'
            f'    offset={self.approach_pose["offset_deg"]:+d} deg\n'
            f'    goal_x={x:.2f}\n'
            f'    goal_y={y:.2f}\n'
            f'    goal_yaw={math.degrees(yaw):.2f} deg'
        )

        send_future = self.compute_path_client.send_goal_async(goal)
        send_future.add_done_callback(
            self._approach_path_goal_response_callback
        )

    def _approach_path_candidate_failed(self, reason, path_max_cost=None):
        """当前候选 path 验证失败：记录日志并切换到下一个候选（不终止整体流程）。"""

        self.get_logger().info(
            f'[APPROACH PATH REJECTED]\n'
            f'    reason={reason}\n'
            f'    forward={self.approach_pose["forward_distance"]:.2f} m\n'
            f'    offset={self.approach_pose["offset_deg"]:+d} deg'
            + (f'\n    path_max_cost={path_max_cost}' if path_max_cost is not None else '')
        )
        # 丢弃当前候选，推进下标，解除 in-flight guard，让 timer 尝试下一个
        self.approach_pose = None
        self.approach_path_validation_started = False
        self.approach_candidate_index += 1

    def _approach_path_goal_response_callback(self, future):
        """Approach ComputePathToPose goal 响应回调。"""

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self._approach_path_candidate_failed('planner goal rejected')
            return

        self.get_logger().info(
            '[APPROACH PATH] goal accepted, waiting for result'
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self._approach_path_result_callback
        )

    def _approach_path_result_callback(self, future):
        """Approach ComputePathToPose 结果回调：检查 path + 路径 cost。"""

        result = future.result().result

        if result is None or result.path is None or len(result.path.poses) < 2:
            self._approach_path_candidate_failed('no valid path returned')
            return

        path_poses = result.path.poses
        path_pose_count = len(path_poses)

        path_length = 0.0
        for i in range(1, path_pose_count):
            dx = path_poses[i].pose.position.x - path_poses[i - 1].pose.position.x
            dy = path_poses[i].pose.position.y - path_poses[i - 1].pose.position.y
            path_length += math.hypot(dx, dy)

        path_max_cost = 0
        cost_sum = 0
        valid_cost_count = 0
        path_cost_threshold = 80

        for pose in path_poses:
            cost = self.get_costmap_cost(
                pose.pose.position.x,
                pose.pose.position.y
            )
            if cost is None:
                self._approach_path_candidate_failed(
                    'path pose outside costmap', path_max_cost=path_max_cost
                )
                return

            if cost > path_max_cost:
                path_max_cost = cost
            cost_sum += cost
            valid_cost_count += 1

            if cost >= path_cost_threshold:
                self._approach_path_candidate_failed(
                    'high-cost cell along planned path',
                    path_max_cost=path_max_cost
                )
                return

        path_mean_cost = cost_sum / valid_cost_count if valid_cost_count > 0 else 0.0

        # 当前候选 LOCAL SAFE + PATH VALID：定稿
        self.approach_path_valid = True

        self.get_logger().info(
            f'[APPROACH POSE]\n'
            f'    x={self.approach_pose["x"]:.2f}\n'
            f'    y={self.approach_pose["y"]:.2f}\n'
            f'    yaw={math.degrees(self.approach_pose["yaw"]):.2f} deg\n'
            f'    forward_distance={self.approach_pose["forward_distance"]:.2f} m\n'
            f'    offset={self.approach_pose["offset_deg"]:+d} deg\n'
            f'    remaining_to_target={self.approach_pose["remaining_to_target"]:.2f} m\n'
            f'    center_cost={self.approach_pose["center_cost"]}\n'
            f'    max_cost={self.approach_pose["max_cost"]}'
        )
        self.get_logger().info(
            f'[APPROACH PATH VALID]\n'
            f'    goal_x={self.approach_pose["x"]:.2f}\n'
            f'    goal_y={self.approach_pose["y"]:.2f}\n'
            f'    path_length={path_length:.2f} m\n'
            f'    path_pose_count={path_pose_count}\n'
            f'    path_max_cost={path_max_cost}\n'
            f'    path_mean_cost={path_mean_cost:.1f}'
        )
        # Stage 5.2：candidate local-safe + path valid，立即发送 NavigateToPose。
        # send_approach_goal 内部有 approach_nav_started guard 防重复；
        # 若此刻 Nav2 server 未 ready 会返回，由 timer 兜底重试。
        self.send_approach_goal()

    # =====================================================
    # Stage 5.2：Approach NavigateToPose（复用 nav_client）
    # 与 Observation B 导航使用独立回调，避免结果串到 Obs B 后处理。
    # =====================================================
    def send_approach_goal(self):
        """对已通过 local-safe + path 验证的 approach_pose 发送 NavigateToPose。"""

        # 去重保护：一个 approach pose 只发一次 goal
        if self.approach_nav_started or self.approach_nav_active:
            return
        # 避免与其他 Nav2 goal 冲突
        if self.nav_goal_active:
            return
        if self.approach_pose is None:
            return
        if self.approach_path_valid is not True:
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warning(
                '[APPROACH NAV] Nav2 action server not ready, waiting (retry)'
            )
            return

        x = self.approach_pose['x']
        y = self.approach_pose['y']
        yaw = self.approach_pose['yaw']

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f'[APPROACH NAV]\n'
            f'    sending NavigateToPose\n'
            f'    goal_x={x:.2f}\n'
            f'    goal_y={y:.2f}\n'
            f'    goal_yaw={math.degrees(yaw):.2f} deg'
        )

        # 发送前置位：防止 timer(5Hz) 重复发送同一个 goal
        self.approach_nav_started = True
        self.approach_nav_active = True
        self.nav_goal_active = True
        self.approach_iteration += 1
        self.get_logger().info(
            f'[APPROACH ITERATION]\n'
            f'    iteration={self.approach_iteration}/{self.max_approach_iterations}'
        )

        # 进入 NAVIGATING_TO_APPROACH：冻结普通语义状态机
        self.transition_to(
            SearchState.NAVIGATING_TO_APPROACH,
            'navigating to approach pose'
        )

        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._approach_nav_feedback_callback
        )
        send_future.add_done_callback(
            self._approach_nav_goal_response_callback
        )

    def _approach_nav_feedback_callback(self, feedback_msg):
        """Approach Nav2 feedback（当前不处理）。"""
        pass

    def _approach_nav_goal_response_callback(self, future):
        """Approach NavigateToPose goal 响应：accepted / rejected。"""

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(
                '[APPROACH NAV FAILED]\n    reason=goal rejected'
            )
            self._handle_approach_nav_failure()
            return

        self.get_logger().info(
            '[APPROACH NAV]\n    goal accepted, waiting for result'
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self._approach_nav_result_callback
        )

    def _approach_nav_result_callback(self, future):
        """Approach NavigateToPose 结果：SUCCEEDED -> 最终确认；否则尝试下一候选。"""

        self.approach_nav_active = False
        self.nav_goal_active = False

        status = future.result().status if future.result() else None

        if status == 4:  # SUCCEEDED
            x = self.approach_pose['x'] if self.approach_pose else float('nan')
            y = self.approach_pose['y'] if self.approach_pose else float('nan')
            self.get_logger().info(
                f'[APPROACH NAV SUCCESS]\n'
                f'    goal_x={x:.2f}\n'
                f'    goal_y={y:.2f}'
            )
            # 清空导航期间旧感知缓存：最终确认必须使用到达后的新 Camera 帧
            self.latest_angle = None
            self.latest_distance = None
            self.latest_range_uncertain = None
            self.last_detection_time_ns = None
            # 到达时刻：fresh frame 判定 + 5s 超时基准（在清缓存之后设置）
            self.approach_arrival_time_ns = self.get_clock().now().nanoseconds
            self.final_confirmation_done = False

            self.transition_to(
                SearchState.APPROACH_REACHED,
                'reached approach pose'
            )
            self.transition_to(
                SearchState.FINAL_CONFIRMATION,
                'waiting for fresh bottle detection'
            )
            self.get_logger().info(
                '[FINAL CONFIRMATION]\n'
                '    waiting for fresh bottle detection'
            )
        else:
            # ABORTED / CANCELED / FAILED
            self.get_logger().warning(
                f'[APPROACH NAV FAILED]\n    status={status}'
            )
            self._handle_approach_nav_failure()

    def _handle_approach_nav_failure(self):
        """
        Approach 导航 rejected/aborted/failed 的处理：
        - 不重跑 Stage 4，不回到 Observation A/B，不重新生成 target_map_position；
        - 若还有其他 local-safe candidate，推进下标并退回 ComputePathToPose 验证
          尝试下一个（由 timer 重新触发 validate_approach_path）；
        - 候选耗尽则 validate_approach_path 输出 [APPROACH PLANNER FAILED] 停止。
        """
        self.approach_nav_active = False
        self.nav_goal_active = False
        self.approach_nav_started = False
        self.approach_pose = None
        self.approach_path_validation_started = False
        self.approach_path_valid = None
        self.approach_candidate_index += 1

        if self.approach_candidate_index < len(self.approach_candidates):
            self.get_logger().info(
                f'[APPROACH NAV] trying next candidate '
                f'(index={self.approach_candidate_index}/'
                f'{len(self.approach_candidates)})'
            )
        else:
            self.get_logger().warning(
                '[APPROACH NAV] no more safe candidates to try'
            )

    # =====================================================
    # Stage 5.3：Final bottle confirmation
    # 到达 approach pose 后等待停车后的新鲜 bottle detection，
    # 5.0s 内无新鲜检测则失败并停止（不重跑任务，不无限等待）。
    # =====================================================
    def check_final_confirmation(self):
        """FINAL_CONFIRMATION 状态下由 timer 周期调用。"""

        if self.final_confirmation_done:
            return
        if self.approach_arrival_time_ns is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = (now_ns - self.approach_arrival_time_ns) / 1e9

        # 新鲜检测：到达时已清空感知缓存，
        # 故 last_detection_time_ns 晚于到达时刻的必为停车后新帧
        fresh = (
            self.latest_angle is not None
            and self.last_detection_time_ns is not None
            and self.last_detection_time_ns > self.approach_arrival_time_ns
        )

        if fresh:
            # 立即标记该 confirmation 已消费，避免 5Hz timer 用同一帧重复触发
            self.final_confirmation_done = True

            r = self.latest_distance
            angle = self.latest_angle

            # distance 基本合法性（有限且在 LiDAR 有效范围内）
            dist_valid = (
                r is not None
                and math.isfinite(r)
                and 0.10 < r < 10.0
            )

            angle_str = f'{angle:.2f}' if angle is not None else 'N/A'
            dist_str = f'{r:.2f}' if (r is not None and math.isfinite(r)) else 'N/A'
            self.get_logger().info(
                f'[FINAL CONFIRMATION]\n'
                f'    bottle detected\n'
                f'    angle={angle_str} deg\n'
                f'    distance={dist_str} m\n'
                f'    range_uncertain={self.latest_range_uncertain}'
            )

            # distance 非法：无法判定距离，停止，不自动切回 Stage 4
            if not dist_valid:
                self.get_logger().warning(
                    '[APPROACH ITERATION FAILED]\n'
                    '    reason=invalid distance after approach'
                )
                return

            # 最终成功条件：进入最终观察距离 且 bottle 基本正对相机
            # （distance<=standoff 时不再要求 range_uncertain=False，
            #   因为近场 LiDAR 可能被 bottle/前景干扰，但视觉已足够确认到达）
            close = r <= self.final_standoff_distance
            centered = angle is not None and abs(angle) <= 15.0

            if close and centered:
                self.mission_success = True
                self.get_logger().info(
                    '[MISSION SUCCESS]\n'
                    '    target reached and visually confirmed\n'
                    f'    final_distance={r:.2f} m\n'
                    f'    final_angle={angle:.2f} deg\n'
                    f'    range_uncertain={self.latest_range_uncertain}\n'
                    f'    approach_iterations={self.approach_iteration}'
                )
                return

            # 未满足成功条件（距离仍 > standoff）：
            # 下一轮 direct_target_localization 需要 reliable range。
            if self.latest_range_uncertain is not False:
                self.get_logger().warning(
                    '[APPROACH ITERATION FAILED]\n'
                    '    reason=range unreliable after approach'
                )
                return

            # 仍然太远：达到最大轮数则失败停止
            if self.approach_iteration >= self.max_approach_iterations:
                self.get_logger().warning(
                    '[MISSION FAILED]\n'
                    '    reason=max approach iterations reached\n'
                    f'    final_distance={r:.2f} m\n'
                    f'    desired_distance<={self.final_standoff_distance:.2f} m'
                )
                return

            self.get_logger().info(
                '[APPROACH ITERATION]\n'
                '    target still too far\n'
                f'    current_distance={r:.2f} m\n'
                f'    desired_distance<={self.final_standoff_distance:.2f} m\n'
                '    starting next approach iteration'
            )

            # 用到达后的 fresh detection 重新直接定位（更新 target_map_position）。
            # 先清旧定位结果，direct_target_localization 内部幂等 + TF 失败可由 timer 重试。
            self.target_map_position = None
            self.target_localization_source = None
            self.direct_target_localization()

            # 只重置 Stage 5 本轮 flag，准备下一轮 Stage 5.1
            self._reset_approach_iteration_flags()

            # 回到 planning 状态，由 timer 重新触发 Stage 5.1（target_map_position 就绪后）
            self.transition_to(
                SearchState.TARGET_CLEAR,
                're-planning next approach iteration'
            )
            return

        # 超时仍无新鲜检测：失败并停止
        if elapsed_sec >= self.final_confirmation_timeout_sec:
            self.final_confirmation_done = True
            self.get_logger().warning(
                '[FINAL CONFIRMATION FAILED]\n'
                '    reason=bottle not detected after approach navigation'
            )

    def _reset_approach_iteration_flags(self):
        """
        开始下一轮 approach 前，只重置 Stage 5 本轮相关 flag。
        不动 detection 订阅 / Nav2 clients / TF buffer / costmap / Stage 4 架构。
        """
        self.approach_pose = None
        self.approach_generation_done = False
        self.approach_candidates = []
        self.approach_candidate_index = 0
        self.approach_path_validation_started = False
        self.approach_path_valid = None
        self.approach_nav_started = False
        self.approach_nav_active = False
        self.final_confirmation_done = False
        self.approach_arrival_time_ns = None

    # =====================================================
    # SEARCHING 主动原地旋转搜索
    # 只在 SEARCHING 且无 Nav2 goal 时持续发布 /cmd_vel；
    # 发现 bottle / 离开 SEARCHING / 超时时立即停车。
    # =====================================================
    def publish_search_rotation(self):
        """持续发布原地低速旋转命令（cmd_vel 需周期刷新）。"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.search_angular_speed
        self.search_cmd_pub.publish(msg)

    def stop_search_motion(self):
        """立即停车：发布零 Twist，避免上一帧旋转速度残留。"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.search_cmd_pub.publish(msg)

    def update_active_search(self):
        """
        timer 周期调用：SEARCHING 下驱动原地旋转 + 30s 超时。
        非 SEARCHING / 有 Nav2 goal / 已失败 / 已完成定位 时不发布。
        """

        if self.state != SearchState.SEARCHING:
            return
        # 任何 Nav2 导航在途时不发 search cmd_vel（避免与 Nav2 冲突）
        if self.nav_goal_active or self.observation_b_nav_active:
            return
        # Stage 4 已定位（进入 Stage 5）后不再搜索
        if self.target_map_position is not None:
            return
        # 搜索已超时失败：停止，不再旋转
        if self.search_failed:
            return

        now_ns = self.get_clock().now().nanoseconds

        # 首次进入本轮搜索：记录起始时刻并打印一次开始日志
        if self.search_start_time_ns is None:
            self.search_start_time_ns = now_ns
            self.get_logger().info(
                f'[SEARCH]\n'
                f'    active rotation started\n'
                f'    angular_speed={self.search_angular_speed:.2f} rad/s\n'
                f'    timeout={self.search_timeout_sec:.1f} s'
            )

        elapsed_sec = (now_ns - self.search_start_time_ns) / 1e9

        # 超过最大搜索时长：停车并失败，不自动无限重启
        if elapsed_sec >= self.search_timeout_sec:
            self.stop_search_motion()
            self.search_failed = True
            self.get_logger().warning(
                f'[SEARCH FAILED]\n'
                f'    reason=target not found within {self.search_timeout_sec:.1f} s'
            )
            return

        # 正常搜索：持续刷新旋转 cmd_vel
        self.publish_search_rotation()

    # -----------------------------
    # 统一状态转移函数
    # -----------------------------
    def transition_to(self, new_state, reason):
        """
        仅在状态真正变化时打印日志 + 发布状态。
        避免每帧重复打印同一状态。
        """

        if new_state == self.state:
            return

        old_state = self.state
        old_name = old_state.name
        self.state = new_state

        # 离开 SEARCHING（发现 bottle / 进入任何非搜索状态）：
        # 立即停车，避免上一帧旋转 cmd_vel 残留，并清搜索计时以便将来新 mission 重计
        if old_state == SearchState.SEARCHING and new_state != SearchState.SEARCHING:
            self.stop_search_motion()
            self.search_start_time_ns = None
            self.search_failed = False

        self.get_logger().info(
            f'[STATE] {old_name} -> {new_state.name} | {reason}'
        )

        # 关键状态变化时附带当前观测值（方便调试）
        self.get_logger().info(
            f'    angle={self._fmt(self.latest_angle)} deg, '
            f'distance={self._fmt(self.latest_distance)} m, '
            f'range_uncertain={self._fmt(self.latest_range_uncertain)}'
        )

        # 进入 REPOSITION_REQUIRED 时：触发 TF 查询（首次 + 重试机制）
        if new_state == SearchState.REPOSITION_REQUIRED:
            self.pose_query_pending = True
            # 新一次换视角：重置候选点 cost / region 打印标志
            self.candidate_costs_printed = False
            self.candidate_region_printed = False
            self.sector_search_printed = False
            pose = self.get_robot_pose()
            if pose is not None:
                self.pose_query_pending = False
                # TF 成功且有 camera angle，计算目标方向 + 候选观察点
                if self.latest_angle is not None:
                    target_map_yaw = self.compute_target_map_bearing(
                        pose[2],          # robot_yaw (rad)
                        self.latest_angle  # camera_angle (deg)
                    )
                    candidates = self.compute_observation_candidates(
                        pose[0],           # robot_x
                        pose[1],           # robot_y
                        target_map_yaw
                    )
                    # 保存候选点，供 costmap_callback 或此处立即查询
                    self.left_candidate = candidates[0]
                    self.right_candidate = candidates[1]
                    # 若 costmap 已就绪，立即打印单 cell cost + 区域统计
                    if self.latest_global_costmap is not None:
                        self._print_candidate_costs()
                        self._print_candidate_region()
                    # 多偏移候选搜索 + 安全过滤 + 选择
                    if self.latest_global_costmap is not None:
                        self.search_observation_candidate(
                            pose[0], pose[1], target_map_yaw
                        )
                    # 前向扇区候选搜索
                    if (
                        self.latest_global_costmap is not None
                        and not self.sector_search_printed
                    ):
                        self.search_sector_observation_candidate(
                            pose[0], pose[1], target_map_yaw
                        )
                        self.sector_search_printed = True

        # 离开 REPOSITION_REQUIRED 时：取消 pending 并重置 warning flag
        elif (
            self.state == SearchState.REPOSITION_REQUIRED
            and new_state != SearchState.REPOSITION_REQUIRED
        ):
            self.pose_query_pending = False
            self.pose_query_warning_shown = False

        self._publish_state()

    def _publish_state(self):
        """发布当前状态字符串。"""

        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    @staticmethod
    def _fmt(value):
        """格式化辅助：None 显示 N/A，浮点保留 2 位。"""

        if value is None:
            return 'N/A'
        if isinstance(value, bool):
            return str(value)
        return f'{value:.2f}'


def main(args=None):

    rclpy.init(args=args)

    node = SemanticSearchController()

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
