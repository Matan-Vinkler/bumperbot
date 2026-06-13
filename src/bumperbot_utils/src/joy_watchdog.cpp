#include "bumperbot_utils/joy_watchdog.hpp"

using std::placeholders::_1;

JoyWatchdog::JoyWatchdog()
: Node("joy_watchdog"), is_locked_(true), has_received_joy_(false), last_joy_time_(0, 0, RCL_ROS_TIME)
{
    declare_parameter<double>("joy_timeout", 0.5);
    joy_timeout_ = get_parameter("joy_timeout").as_double();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyWatchdog::joyCallback, this, _1));

    watchdog_pub_ = create_publisher<std_msgs::msg::Bool>("joy_watchdog", 10);

    // Start locked — robot won't move until the controller sends its first message
    publishLock(true);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JoyWatchdog::timerCallback, this));
}

void JoyWatchdog::joyCallback(const sensor_msgs::msg::Joy&)
{
    last_joy_time_ = get_clock()->now();
    has_received_joy_ = true;

    if (is_locked_)
    {
        publishLock(false);
        is_locked_ = false;
        RCLCPP_INFO(get_logger(), "Controller connected — releasing lock");
    }
}

void JoyWatchdog::timerCallback()
{
    if (!has_received_joy_ || is_locked_)
        return;

    auto elapsed = (get_clock()->now() - last_joy_time_).seconds();
    if (elapsed > joy_timeout_)
    {
        publishLock(true);
        is_locked_ = true;
        RCLCPP_WARN(get_logger(), "Joy timeout (%.2fs) — locking robot", elapsed);
    }
}

void JoyWatchdog::publishLock(bool locked)
{
    std_msgs::msg::Bool msg;
    msg.data = locked;
    watchdog_pub_->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyWatchdog>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
