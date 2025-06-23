# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped

# class GoalStatusListener(Node):
#     def __init__(self):
#         super().__init__('goal_status_listener')
#         self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

#         self.get_logger().info('Waiting for action server /navigate_to_pose...')
#         self._client.wait_for_server()

#         # Optional: send a goal here or just listen if goals come from elsewhere
#         # self.send_dummy_goal()

#     def send_dummy_goal(self):
#         """Optional: sends a goal to trigger the result flow"""
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.pose.position.x = 29.0
#         goal_msg.pose.pose.position.y = 18.0
#         goal_msg.pose.pose.orientation.w = 1.0  # Facing forward
#         self._send_goal_future = self._client.send_goal_async(goal_msg)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected.')
#             return
#         self.get_logger().info('Goal accepted. Waiting for result...')
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         result = future.result().result
#         status = future.result().status

#         if status == 4:
#             self.get_logger().info('Goal reached successfully.')
#         else:
#             self.get_logger().warn(f'Goal failed. Status: {status}')
#         rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = GoalStatusListener()
#     rclpy.spin(node)

# if __name__ == '__main__':
#     main()
