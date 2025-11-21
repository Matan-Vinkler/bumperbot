#ifndef ODOM_MOTION_MODEL_HPP_
#define ODOM_MOTION_MODEL_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

class OdomMotionModel : public rclcpp::Node
{
public:
    OdomMotionModel();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

    void odomCallback(const nav_msgs::msg::Odometry& msg);

    geometry_msgs::msg::PoseArray samples_;

    bool is_first_odom_;
    double last_odom_x_, last_odom_y_, last_odom_theta_;
    double alpha1_, alpha2_, alpha3_, alpha4_;
    int n_samples_;
};


#endif