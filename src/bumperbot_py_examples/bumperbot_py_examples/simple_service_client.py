import rclpy
from rclpy.node import Node

from bumperbot_msgs.srv import AddTwoInts

import sys

class SimpleServiceClient(Node):
    def __init__(self, a: int, b: int):
        super().__init__("simple_service_client")

        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

        while not self.client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for service...")

        self.req_ = AddTwoInts.Request()
        self.req_.a = a
        self.req_.b = b

        self.future_ = self.client_.call_async(self.req_)
        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info(f"Service response: {future.result().sum}")

def main():
    rclpy.init()

    if len(sys.argv) != 3:
        print("Usage: simple_service_client {a} {b}")
        return -1
    
    a = int(sys.argv[1])
    b = int(sys.argv[2])

    simple_service_client = SimpleServiceClient(a, b)
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()