#ifndef TRAJECTORY_DRAWER_HPP_
#define TRAJECTORY_DRAWER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class TrajectoryDrawer : public rclcpp::Node
{
public:
    TrajectoryDrawer();

private:
    void odomCallback(const nav_msgs::msg::Odometry& odom_msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;

    nav_msgs::msg::Path traj_msg_;
};

#endif