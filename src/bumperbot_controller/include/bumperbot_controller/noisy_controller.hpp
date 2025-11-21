#ifndef NOISY_CONTROLLER_HPP_
#define NOISY_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

class NoisyController : public rclcpp::Node
{
public:
    NoisyController();

private:
    void jointCallback(const sensor_msgs::msg::JointState& msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double wr_;
    double ws_;

    double left_wheel_prev_pos_;
    double right_wheel_prev_pos_;
    rclcpp::Time prev_time_;

    double x_;
    double y_;
    double theta_;

    nav_msgs::msg::Odometry odom_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif