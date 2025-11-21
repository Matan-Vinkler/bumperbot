#include "bumperbot_utils/safety_stop.hpp"

#include <math.h>

using std::placeholders::_1;

SafetyStop::SafetyStop() : Node("safety_stop"), state_(State::FREE), prev_state_(State::FREE), is_first_msg_(true)
{
    declare_parameter<double>("danger_distance", 0.2);
    declare_parameter<double>("warning_distance", 0.6);
    declare_parameter<std::string>("scan_topic", "scan");
    declare_parameter<std::string>("safety_stop_topic", "safety_stop");

    danger_distance_ = get_parameter("danger_distance").as_double();
    warning_distance_ = get_parameter("warning_distance").as_double();
    scan_topic_ = get_parameter("scan_topic").as_string();
    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, 10, std::bind(&SafetyStop::laserCallback, this, _1));
    safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_stop_topic_, 10);
    zones_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("zones", 10);

    decrease_speed_client_ = rclcpp_action::create_client<JoyTurbo>(this, "joy_turbo_decrease");
    increase_speed_client_ = rclcpp_action::create_client<JoyTurbo>(this, "joy_turbo_increase");

    auto timeout = std::chrono::seconds(1);
    auto sleep_dur = std::chrono::seconds(2);

    while (!decrease_speed_client_->wait_for_action_server(timeout) && rclcpp::ok())
    {
        RCLCPP_WARN(get_logger(), "Action /joy_turbo_decrease not available! Waiting...");
        std::this_thread::sleep_for(sleep_dur);
    }
    while (!increase_speed_client_->wait_for_action_server(timeout) && rclcpp::ok())
    {
        RCLCPP_WARN(get_logger(), "Action /joy_turbo_increase not available! Waiting...");
        std::this_thread::sleep_for(sleep_dur);
    }

    visualization_msgs::msg::Marker warning_zone;
    warning_zone.id = 0;
    warning_zone.action = visualization_msgs::msg::Marker::ADD;
    warning_zone.type = visualization_msgs::msg::Marker::CYLINDER;
    warning_zone.scale.z = 0.001;
    warning_zone.scale.x = warning_distance_ * 2;
    warning_zone.scale.y = warning_distance_ * 2;
    warning_zone.color.r = 1.0;
    warning_zone.color.g = 0.984;
    warning_zone.color.b = 0.0;
    warning_zone.color.a = 0.5;

    visualization_msgs::msg::Marker danger_zone;
    danger_zone.id = 1;
    danger_zone.action = visualization_msgs::msg::Marker::ADD;
    danger_zone.type = visualization_msgs::msg::Marker::CYLINDER;
    danger_zone.scale.z = 0.001;
    danger_zone.scale.x = danger_distance_ * 2;
    danger_zone.scale.y = danger_distance_ * 2;
    danger_zone.color.r = 1.0;
    danger_zone.color.g = 0.0;
    danger_zone.color.b = 0.0;
    danger_zone.color.a = 0.5;
    danger_zone.pose.position.z = 0.01;

    zones_.markers.push_back(warning_zone);
    zones_.markers.push_back(danger_zone);
}

void SafetyStop::laserCallback(const sensor_msgs::msg::LaserScan &msg)
{
    state_ = State::FREE;
    for (const float &range_value : msg.ranges)
    {
        if (!std::isinf(range_value) && range_value <= warning_distance_)
        {
            state_ = State::WARNING;
            if (range_value <= danger_distance_)
            {
                state_ = State::DANGER;
                break;
            }
        }
    }

    if (state_ != prev_state_)
    {

        std_msgs::msg::Bool is_safety_stop;
        if (state_ == State::DANGER)
        {
            is_safety_stop.data = true;
            zones_.markers.at(0).color.a = 1.0;
            zones_.markers.at(1).color.a = 1.0;
        }
        else if (state_ == State::FREE)
        {
            is_safety_stop.data = false;
            zones_.markers.at(0).color.a = 0.5;
            zones_.markers.at(1).color.a = 0.5;
            increase_speed_client_->async_send_goal(JoyTurbo::Goal());
        }
        else if (state_ == State::WARNING)
        {
            is_safety_stop.data = false;
            zones_.markers.at(0).color.a = 1.0;
            zones_.markers.at(1).color.a = 0.5;
            decrease_speed_client_->async_send_goal(JoyTurbo::Goal());
        }

        safety_stop_pub_->publish(is_safety_stop);
        prev_state_ = state_;
    }

    if(is_first_msg_)
    {
        for(auto& zone: zones_.markers)
        {
            zone.header.frame_id = msg.header.frame_id;
        }
        is_first_msg_ = false;
    }
    zones_pub_->publish(zones_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SafetyStop>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}