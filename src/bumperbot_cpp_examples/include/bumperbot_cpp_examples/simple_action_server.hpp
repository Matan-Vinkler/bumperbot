#ifndef SIMPLE_ACTION_SERVER_HPP_
#define SIMPLE_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <bumperbot_msgs/action/fibonacci.hpp>

namespace bumperbot_cpp_examples
{
    class SimpleActionServer : public rclcpp::Node
    {
    public:
        explicit SimpleActionServer(const rclcpp::NodeOptions& options);

    private:
        rclcpp_action::Server<bumperbot_msgs::action::Fibonacci>::SharedPtr action_server_;

        rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID&, std::shared_ptr<const bumperbot_msgs::action::Fibonacci::Goal> goal);
        void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle);
        rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>>);

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle);
    };
}

#endif