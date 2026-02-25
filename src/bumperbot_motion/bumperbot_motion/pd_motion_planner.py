#!/usr/bin/env python3

import rclpy
import rclpy.time
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, inverse_matrix

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped

import math

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
        self.prev_angular_error_ = 0.0
        self.prev_linear_error_ = 0.0
        self.last_cycle_time_ = self.get_clock().now()

    def pathCallback(self, path_msg: Path):
        self.global_plan_ = path_msg

    def controlLoop(self):
        if (not self.global_plan_) or (not self.global_plan_.poses):
            return
        
        try:
            robot_pose_transform = self.tf_buffer_.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")
            return
        
        if not self.transform_plan(robot_pose_transform.header.frame_id):
            self.get_logger().error("Unable to transform Plan in robot's frame")
            return
        
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = robot_pose_transform.header.frame_id
        robot_pose.pose.position.x = robot_pose_transform.transform.translation.x
        robot_pose.pose.position.y = robot_pose_transform.transform.translation.y
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation

        next_pose = self.get_next_pose(robot_pose)

        distance = self.calc_poses_distance(next_pose, robot_pose)
        if distance <= 0.1:
            self.get_logger().info("Goal Reached!")
            self.global_plan_.poses.clear()
            return
        
        self.next_pose_pub_.publish(next_pose)

        robot_pose_tf = quaternion_matrix([
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w
        ])
        robot_pose_tf[0][3] = robot_pose.pose.position.x
        robot_pose_tf[1][3] = robot_pose.pose.position.y

        next_pose_tf = quaternion_matrix([
            next_pose.pose.orientation.x,
            next_pose.pose.orientation.y,
            next_pose.pose.orientation.z,
            next_pose.pose.orientation.w
        ])
        next_pose_tf[0][3] = next_pose.pose.position.x
        next_pose_tf[1][3] = next_pose.pose.position.y

        next_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_pose_tf), next_pose_tf)
        linear_error = next_pose_robot_tf[0][3]
        angular_error = next_pose_robot_tf[1][3]
        dt = (self.get_clock().now() - self.last_cycle_time_).nanoseconds * 1e-9

        linear_error_derivative = (linear_error - self.prev_linear_error_) / dt
        angular_error_derivative = (angular_error - self.prev_angular_error_) / dt

        cmd_vel = Twist()
        cmd_vel.linear.x = max(-self.max_linear_velocity_, min(self.kp_ * linear_error + self.kd_ * linear_error_derivative, self.max_linear_velocity_))
        cmd_vel.angular.z = max(-self.max_angular_velocity_, min(self.kp_ * angular_error + self.kd_ * angular_error_derivative, self.max_angular_velocity_))
        self.cmd_pub_.publish(cmd_vel)

        self.prev_linear_error_ = linear_error
        self.prev_angular_error_ = angular_error
        self.last_cycle_time_ = self.get_clock().now()


    def transform_plan(self, frame: str):
        if self.global_plan_.header.frame_id == frame:
            return True
        
        try:
            transform = self.tf_buffer_.lookup_transform(frame, self.global_plan_.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Could not transform plan from frame {self.global_plan_.header.frame_id} to {frame}: {e}")
            return False
        
        transform_matrix = quaternion_matrix([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        transform_matrix[0][3] = transform.transform.translation.x
        transform_matrix[1][3] = transform.transform.translation.y

        for pose in self.global_plan_.poses:
            pose: PoseStamped = pose

            pose_matrix = quaternion_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            pose_matrix[0][3] = pose.pose.position.x
            pose_matrix[1][3] = pose.pose.position.y

            transformed_pose = concatenate_matrices(pose_matrix, transform_matrix)
            [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_matrix(transformed_pose)
            [pose.pose.position.x, pose.pose.position.y] = translation_from_matrix(transformed_pose)

            pose.header.frame_id = frame

        self.global_plan_.header.frame_id = frame
        return True
    
    def get_next_pose(self, robot_pose: PoseStamped) -> PoseStamped:
        next_pose: PoseStamped = self.global_plan_.poses[-1]
        for pose in reversed(self.global_plan_.poses):
            pose: PoseStamped = pose
            distance = self.calc_poses_distance(pose, robot_pose)

            if distance > self.step_size_:
                next_pose = pose
            else:
                break

        return next_pose
    
    def calc_poses_distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)


def main():
    rclpy.init()

    node = PDMotionPlanner()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
