#ifndef JOY_WATCHDOG_HPP_
#define JOY_WATCHDOG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

class JoyWatchdog : public rclcpp::Node
{
public:
    JoyWatchdog();

private:
    void joyCallback(const sensor_msgs::msg::Joy& msg);
    void timerCallback();
    void publishLock(bool locked);

    double joy_timeout_;
    bool is_locked_;
    bool has_received_joy_;
    rclcpp::Time last_joy_time_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr watchdog_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
