#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node
{
public:
    KalmanFilter();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double mean_;
    double variance_;

    double imu_angular_z_;
    bool is_first_odom_;
    double last_angular_z_;

    double motion_;
    nav_msgs::msg::Odometry kalman_odom_;

    double motion_variance_;
    double measurement_variance_;

    void odomCallback(const nav_msgs::msg::Odometry& msg);
    void imuCallback(const sensor_msgs::msg::Imu& msg);

    void statePredict();
    void measurementUpdate();
};

#endif