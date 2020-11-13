import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

class PlannerActionClient(Node):

    def __init__(self):
        super().__init__('planner_action_client')
        self._action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def send_goal(self, pose, planner_id):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.pose = pose
        goal_msg.planner_id = planner_id

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = PlannerActionClient()
    pose = PoseStamped()
    pose.pose.position.x = 27.97
    pose.pose.position.y = 38.01
    pose.pose.position.z = 0.0
    pose.header.stamp = action_client.get_clock().now().to_msg()
    planner_id = "SMACPlanner"
    action_client.send_goal(pose, planner_id=planner_id)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
