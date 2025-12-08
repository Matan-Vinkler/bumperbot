#!/usr/bin/env python3

import rclpy
import rclpy.time
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped

class PDMotionPlanner(Node):
    def __init__(self):
        super().__init__("pd_motion_planner")

        self.declare_parameter("kp", 2.0)
        self.declare_parameter("kd", 0.1)
        self.declare_parameter("step_size", 0.2)
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 1.0)

        self.kp_ = self.get_parameter("kp").get_parameter_value().double_value
        self.kd_ = self.get_parameter("kd").get_parameter_value().double_value
        self.step_size_ = self.get_parameter("step_size").get_parameter_value().double_value
        self.max_linear_velocity_ = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
        self.max_angular_velocity_ = self.get_parameter("max_angular_velocity").get_parameter_value().double_value

        self.path_sub_ = self.create_subscription(Path, "/a_star/path", self.pathCallback, 10)
        self.cmd_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.next_pose_pub_ = self.create_publisher(PoseStamped, "/pd/next_pose", 10)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.timer_ = self.create_timer(0.1, self.controlLoop)

        self.global_plan_ = None

    def pathCallback(self, path_msg: Path):
        self.global_plan_ = path_msg

    def controlLoop(self):
        if not (self.global_plan_) or (not self.global_plan_.poses):
            return
        
        try:
            robot_pose_transform = self.tf_buffer_.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")
            return
        
        self.get_logger().info(f"frame_id Robot Pose: {robot_pose_transform.header.frame_id}")
        self.get_logger().info(f"frame_id Global Plan: {self.global_plan_.header.frame_id}")

def main():
    rclpy.init()

    node = PDMotionPlanner()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
