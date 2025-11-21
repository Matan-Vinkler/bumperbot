#ifndef SIMPLE_QOS_SUB_HPP_
#define SIMPLE_QOS_SUB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

using namespace std::chrono_literals;

class SimpleQoSSub : public rclcpp::Node
{
public:
    SimpleQoSSub();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::QoS qos_profile_sub_;

    void msgCallback(const std_msgs::msg::String &message) const;
};

#endif