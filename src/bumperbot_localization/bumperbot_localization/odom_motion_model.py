#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose

from tf_transformations import euler_from_quaternion, quaternion_from_euler

from math import sin, cos, atan2, sqrt, fabs, pi
import random
import time

def angle_diff(a: float, b: float) -> float:
    a = atan2(sin(a), cos(a))
    b = atan2(sin(b), cos(b))

    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2

class OdomMotionModel(Node):
    def __init__(self):
        super().__init__("odom_motion_model")

        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom", self.odomCallback, 10)
        self.pose_array_pub_ = self.create_publisher(PoseArray, "odometry_motion_model/samples", 10)

        self.is_first_odom_ = True
        self.last_odom_x_ = 0.0
        self.last_odom_y_ = 0.0
        self.last_odom_theta_ = 0.0

        self.declare_parameter("alpha1", 0.1)
        self.declare_parameter("alpha2", 0.1)
        self.declare_parameter("alpha3", 0.1)
        self.declare_parameter("alpha4", 0.1)
        self.declare_parameter("n_samples", 300)

        self.alpha1_ = self.get_parameter("alpha1").get_parameter_value().double_value
        self.alpha2_ = self.get_parameter("alpha2").get_parameter_value().double_value
        self.alpha3_ = self.get_parameter("alpha3").get_parameter_value().double_value
        self.alpha4_ = self.get_parameter("alpha4").get_parameter_value().double_value
        self.n_samples_ = self.get_parameter("n_samples").get_parameter_value().integer_value

        if self.n_samples_ >= 0:
            self.samples_ = PoseArray()
            self.samples_.poses = [Pose() for _ in range(self.n_samples_)]
        else:
            self.get_logger().fatal(f"Invalid number of samples requested: {self.n_samples_}")
            return
            

    def odomCallback(self, msg: Odometry):
        q = [msg.pose.pose.orientation.x, 
             msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, 
             msg.pose.pose.orientation.w]
        
        _, _, yaw = euler_from_quaternion(q)

        if self.is_first_odom_:
            self.last_odom_x_ = msg.pose.pose.position.x
            self.last_odom_y_ = msg.pose.pose.position.y
            self.last_odom_theta_ = yaw

            self.samples_.header.frame_id = msg.header.frame_id

            self.is_first_odom_ = False
        else:
            odom_x_inc = msg.pose.pose.position.x - self.last_odom_x_
            odom_y_inc = msg.pose.pose.position.y - self.last_odom_y_
            odom_theta_inc = angle_diff(yaw, self.last_odom_theta_)

            if sqrt(odom_y_inc ** 2 + odom_x_inc ** 2) < 0.01:
                delta_rot1 = 0.0
            else:
                delta_rot1 = angle_diff(atan2(odom_y_inc, odom_x_inc), yaw)

            delta_trasl = odom_x_inc * cos(self.last_odom_theta_) + odom_y_inc * sin(self.last_odom_theta_)
            delta_rot2 = angle_diff(odom_theta_inc, delta_rot1)

            rot1_var = self.alpha1_ * delta_rot1 + self.alpha2_ * delta_trasl
            trasl_var = self.alpha3_ * delta_trasl + self.alpha4_ * (delta_rot1 + delta_rot2)
            rot2_var = self.alpha1_ * delta_rot2 + self.alpha2_ * delta_trasl

            random.seed(int(time.time()))

            for sample in self.samples_.poses:
                rot1_noise = random.gauss(0.0, rot1_var)
                trasl_noise = random.gauss(0.0, trasl_var)
                rot2_noise = random.gauss(0.0, rot2_var)

                delta_rot1_draw = angle_diff(delta_rot1, rot1_noise)
                delta_trasl_draw = delta_trasl - trasl_noise
                delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)

                sample: Pose = sample

                sample_q = [sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w]
                _, _, sample_yaw = euler_from_quaternion(sample_q)

                sample.position.x += delta_trasl_draw * cos(sample_yaw + delta_rot1_draw)
                sample.position.y += delta_trasl_draw * sin(sample_yaw + delta_rot1_draw)

                q1 = quaternion_from_euler(0, 0, sample_yaw + delta_rot1_draw + delta_rot2_draw)
                sample.orientation.x = q1[0]
                sample.orientation.y = q1[1]
                sample.orientation.z = q1[2]
                sample.orientation.w = q1[3]

            self.last_odom_x_ = msg.pose.pose.position.x
            self.last_odom_y_ = msg.pose.pose.position.y
            self.last_odom_theta_ = yaw

            self.pose_array_pub_.publish(self.samples_)

def main():
    rclpy.init()

    odom_motion_model = OdomMotionModel()
    rclpy.spin(odom_motion_model)
    odom_motion_model.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()