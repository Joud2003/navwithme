from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class Nav2GoalManager:
    def __init__(self, node, result_callback):
        self.node = node
        self.result_callback = result_callback
        self.action_client = ActionClient(
            node, NavigateToPose, "navigate_to_pose"
        )
        self.busy = False

    def send_goal(self, pose, goal_type):
        if self.busy:
            return False

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Action server not available")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.node.get_logger().info(f"Sending {goal_type} goal: {pose}")
        self.busy = True
        self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        ).add_done_callback(
            lambda future: self.goal_response_callback(future, goal_type)
        )
        return True

    def goal_response_callback(self, future, goal_type):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.node.get_logger().error(
                f"Failed to send {goal_type} goal: {error}"
            )
            self.busy = False
            self.result_callback(goal_type, False)
            return

        if not goal_handle.accepted:
            self.node.get_logger().warn(
                f"{goal_type.capitalize()} goal rejected"
            )
            self.busy = False
            self.result_callback(goal_type, False)
            return

        self.node.get_logger().info(f"{goal_type.capitalize()} goal accepted")
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self.process_result(result_future, goal_type)
        )

    def process_result(self, future, goal_type):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.node.get_logger().error(
                f"Failed to get {goal_type} result: {error}"
            )
            self.busy = False
            self.result_callback(goal_type, False)
            return

        succeeded = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        self.node.get_logger().info(
            f"{goal_type.capitalize()} goal finished with status "
            f"{wrapped_result.status}"
        )
        self.busy = False
        self.result_callback(goal_type, succeeded)

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().debug(
            f"Navigation feedback: {feedback_msg.feedback}"
        )
