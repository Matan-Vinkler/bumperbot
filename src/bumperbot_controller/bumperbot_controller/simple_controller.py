#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

import numpy as np
import math

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)      # wr (the radius of the wheels)
        self.declare_parameter("wheel_separation", 0.17)  # ws (the distance between the two wheels)

        self.wr_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.ws_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using [w_r = {self.wr_}] and [w_s = {self.ws_}]")

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.wheel_command_pub_ = self.create_publisher(Float64MultiArray, "/simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "/bumperbot_controller/cmd_vel", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState, "/joint_states", self.jointStateCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "/bumperbot_controller/odom", 10)

        self.speed_conversion_ = np.array([[self.wr_ / 2, self.wr_ / 2], 
                                           [self.wr_ / self.ws_, -self.wr_ / self.ws_]])
        
        self.speed_conversion_inv_ = np.linalg.inv(self.speed_conversion_)

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.broadcaster_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"
        
        self.get_logger().info(f"Conversion matrix:\n{self.speed_conversion_}")

    def velCallback(self, msg: TwistStamped):
        robot_speed = np.array([[msg.twist.linear.x], 
                                [msg.twist.angular.z]])
        
        wheel_speed = np.matmul(self.speed_conversion_inv_, robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        self.wheel_command_pub_.publish(wheel_speed_msg)

    def jointStateCallback(self, msg: JointState):
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dp_right = msg.position[0] - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        linear = (self.wr_ * fi_right + self.wr_ * fi_left) / 2
        angular = (self.wr_ * fi_right - self.wr_ * fi_left) / self.ws_

        d_s = (self.wr_ * dp_right + self.wr_ * dp_left) / 2
        d_theta = (self.wr_ * dp_right - self.wr_ * dp_left) / self.ws_

        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = quaternion_from_euler(0, 0, self.theta_)

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        self.odom_pub_.publish(self.odom_msg_)
        self.broadcaster_.sendTransform(self.transform_stamped_)

def main():
    rclpy.init()

    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()