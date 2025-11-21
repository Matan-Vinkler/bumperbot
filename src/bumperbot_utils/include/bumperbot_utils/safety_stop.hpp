#ifndef SAFETY_STOP_HPP_
#define SAFETY_STOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <twist_mux_msgs/action/joy_turbo.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

enum State { FREE, WARNING, DANGER };

using twist_mux_msgs::action::JoyTurbo;

class SafetyStop : public rclcpp::Node
{
public:
    SafetyStop();

private:
    double danger_distance_;
    double warning_distance_;
    std::string scan_topic_;
    std::string safety_stop_topic_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_;

    void laserCallback(const sensor_msgs::msg::LaserScan& msg);

    rclcpp_action::Client<JoyTurbo>::SharedPtr decrease_speed_client_;
    rclcpp_action::Client<JoyTurbo>::SharedPtr increase_speed_client_;

    State state_;
    State prev_state_;
    bool is_first_msg_;

    visualization_msgs::msg::MarkerArray zones_;
};

#endif