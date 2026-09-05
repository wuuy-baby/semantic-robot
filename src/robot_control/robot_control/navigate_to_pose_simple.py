import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose, FollowPath
from geometry_msgs.msg import PoseStamped


class SimpleNavigator(Node):

    def __init__(self):
        super().__init__('simple_navigator')

        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        self.controller_client = ActionClient(
            self,
            FollowPath,
            '/follow_path'
        )

        self.target_x = 3.4
        self.target_y = 3.4

        self.send_plan_request()

    def send_plan_request(self):

        self.get_logger().info('Waiting for planner server...')

        self.planner_client.wait_for_server()

        goal_msg = ComputePathToPose.Goal()

        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()

        goal_msg.goal.pose.position.x = self.target_x
        goal_msg.goal.pose.position.y = self.target_y
        goal_msg.goal.pose.orientation.w = 1.0

        goal_msg.planner_id = 'GridBased'
        goal_msg.use_start = False

        self.get_logger().info(
            f'Sending goal to planner: ({self.target_x}, {self.target_y})'
        )

        future = self.planner_client.send_goal_async(goal_msg)
        future.add_done_callback(
            self.plan_goal_response_callback
        )

    def plan_goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Planner goal rejected')
            return

        self.get_logger().info('Planner goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.plan_result_callback
        )

    def plan_result_callback(self, future):

        result = future.result().result

        if len(result.path.poses) == 0:
            self.get_logger().error(
                'Planner returned empty path'
            )
            return

        self.get_logger().info(
            f'Path received: {len(result.path.poses)} poses'
        )

        self.send_follow_path(result.path)

    def send_follow_path(self, path):

        self.get_logger().info(
            'Waiting for controller server...'
        )

        self.controller_client.wait_for_server()

        goal_msg = FollowPath.Goal()

        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'goal_checker'

        self.get_logger().info(
            'Sending path to controller'
        )

        future = self.controller_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self.follow_goal_response_callback
        )

    def follow_goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Controller goal rejected'
            )
            return

        self.get_logger().info(
            'Controller goal accepted'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.follow_result_callback
        )

    def follow_result_callback(self, future):

        result = future.result()

        self.get_logger().info(
            f'Navigation finished with status: {result.status}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SimpleNavigator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
