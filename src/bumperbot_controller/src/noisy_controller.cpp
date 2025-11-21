#include "bumperbot_controller/noisy_controller.hpp"

#include <tf2/LinearMath/Quaternion.hpp>

#include <cmath>
#include <random>

using std::placeholders::_1;

NoisyController::NoisyController() : Node("noisy_controller"), left_wheel_prev_pos_(0.0), right_wheel_prev_pos_(0.0), x_(0.0), y_(0.0), theta_(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    wr_ = get_parameter("wheel_radius").as_double();
    ws_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Using [w_r = " << wr_ << "] and [w_s = " << ws_ << "]");

    prev_time_ = get_clock()->now();

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&NoisyController::jointCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom_noisy", 10);

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_ekf";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_noisy";

    RCLCPP_INFO_STREAM(get_logger(), "[ADDING A LITTLE NOISE TO SPICE UP !!!!!]");
}

void NoisyController::jointCallback(const sensor_msgs::msg::JointState &msg)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_gen(seed);
    std::normal_distribution<double> left_encoder_noise(0.0, 0.00);
    std::normal_distribution<double> right_encoder_noise(0.0, 0.00);

    double wheel_encoder_left = msg.position.at(1) + left_encoder_noise(noise_gen);
    double wheel_encoder_right = msg.position.at(0) + right_encoder_noise(noise_gen);

    double dp_left = wheel_encoder_left - left_wheel_prev_pos_;
    double dp_right = wheel_encoder_right - right_wheel_prev_pos_;

    rclcpp::Time curr_time = msg.header.stamp;
    rclcpp::Duration dt = curr_time - prev_time_;

    left_wheel_prev_pos_ = msg.position.at(1);
    right_wheel_prev_pos_ = msg.position.at(0);
    prev_time_ = curr_time;

    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();

    double linear = (wr_ * fi_right + wr_ * fi_left) / 2;
    double angular = (wr_ * fi_right - wr_ * fi_left) / ws_;

    double d_s = (wr_ * dp_right + wr_ * dp_left) / 2;
    double d_theta = (wr_ * dp_right - wr_ * dp_left) / ws_;

    theta_ += d_theta;
    x_ += d_s * std::cos(theta_);
    y_ += d_s * std::sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;
    odom_msg_.header.stamp = get_clock()->now();

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();

    odom_pub_->publish(odom_msg_);
    broadcaster_->sendTransform(transform_stamped_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NoisyController>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}