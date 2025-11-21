#ifndef SIMPLE_LIFECYCLE_HPP_
#define SIMPLE_LIFECYCLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleLifeCycle : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit SimpleLifeCycle(bool intra_process_comms);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void msgCallback(const std_msgs::msg::String& msg);
};

#endif