#include "bumperbot_controller/twist_relay.hpp"

using std::placeholders::_1;

TwistRelay::TwistRelay() : Node("twist_relay")
{
    controller_sub_ = create_subscription<geometry_msgs::msg::Twist>("/bumperbot_controller/cmd_vel_unstamped", 10, std::bind(&TwistRelay::controllerTwistCallback, this, _1));
    controller_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 10);

    joy_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/input_joy/cmd_vel_stamped", 10, std::bind(&TwistRelay::joyTwistCallback, this, _1));
    joy_pub_ = create_publisher<geometry_msgs::msg::Twist>("/input_joy/cmd_vel", 10);
}

void TwistRelay::controllerTwistCallback(const geometry_msgs::msg::Twist &msg)
{
    geometry_msgs::msg::TwistStamped twist_stamped_msg;
    twist_stamped_msg.header.stamp = get_clock()->now();
    twist_stamped_msg.twist = msg;

    controller_pub_->publish(twist_stamped_msg);
}

void TwistRelay::joyTwistCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    geometry_msgs::msg::Twist twist_msg;
    twist_msg = msg.twist;

    joy_pub_->publish(twist_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TwistRelay>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}