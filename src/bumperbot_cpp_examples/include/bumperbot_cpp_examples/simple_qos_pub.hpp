#ifndef SIMPLE_QOS_PUB_HPP_
#define SIMPLE_QOS_PUB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleQoSPub : public rclcpp::Node
{
public:
    SimpleQoSPub();

private:
    unsigned int counter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::QoS qos_profile_pub_;

    void timerCallback();
};


#endif