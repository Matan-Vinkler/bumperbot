import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from bumperbot_msgs.action import Fibonacci

import time

class SimpleActionServer(Node):
    def __init__(self):
        super().__init__("simple_action_server")

        self.action_server_ = ActionServer(self, Fibonacci, "fibonacci", self.goalCallback)

        self.get_logger().info("Starting action server!!!")

    def goalCallback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received goal requesr with order {goal_handle.request.order}")

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info(f"Feedbac: {feedback_msg.partial_sequence}")
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.5)

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        goal_handle.succeed()
        return result
    
def main():
    rclpy.init()

    node = SimpleActionServer()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()