import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from bumperbot_msgs.action import Fibonacci

class SimpleActionClient(Node):
    def __init__(self, order = 10):
        super().__init__("simple_action_client")

        self.action_client_ = ActionClient(self, Fibonacci, "fibonacci")
        self.action_client_.wait_for_server()

        self.goal_ = Fibonacci.Goal()
        self.goal_.order = order
        self.future_ = self.action_client_.send_goal_async(self.goal_, self.feedbackCallback)
        self.future_.add_done_callback(self.responseCallback)

    def feedbackCallback(self, feedback_msg):
        self.get_logger().info(f"Received feedback: {feedback_msg.partial_sequence}")

    def responseCallback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return
        
        self.get_logger().info("Goal accepted!")

        self.future_ :Future = goal_handle.get_result_async()
        self.future_.add_done_callback(self.resultCallback)

    def resultCallback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.sequence}")

        rclpy.shutdown()

def main():
    rclpy.init()

    node = SimpleActionClient()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()