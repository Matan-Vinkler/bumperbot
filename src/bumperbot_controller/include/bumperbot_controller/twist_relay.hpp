#ifndef TWIST_RELAY_HPP_
#define TWIST_RELAY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistRelay : public rclcpp::Node
{
public:
    TwistRelay();

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_pub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;

    void controllerTwistCallback(const geometry_msgs::msg::Twist& msg);
    void joyTwistCallback(const geometry_msgs::msg::TwistStamped& msg);
};

#endif