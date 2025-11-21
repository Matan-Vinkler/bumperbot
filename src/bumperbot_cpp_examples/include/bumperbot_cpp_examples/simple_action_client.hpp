#ifndef SIMPLE_ACTION_CLIENT_HPP_
#define SIMPLE_ACTION_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <bumperbot_msgs/action/fibonacci.hpp>

using bumperbot_msgs::action::Fibonacci;

namespace bumperbot_cpp_examples
{
    class SimpleActionClient : public rclcpp::Node
    {
    public:
        explicit SimpleActionClient(const rclcpp::NodeOptions& options, int order);

    private:
        int order_;

        rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
        rclcpp::TimerBase::SharedPtr timer_;

        void timerCallback();
        void goalCallback(const rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr& goal_handle);
        void feedbackCallback(rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback>& feedback);
        void resultCallback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult& result);
    };
}

#endif