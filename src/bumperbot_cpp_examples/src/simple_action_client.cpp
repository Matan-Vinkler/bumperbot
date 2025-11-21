#include "bumperbot_cpp_examples/simple_action_client.hpp"

#include <rclcpp_components/register_node_macro.hpp>

using namespace std::placeholders;

bumperbot_cpp_examples::SimpleActionClient::SimpleActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), int order = 10) : Node("simple_action_client", options), order_(order)
{
    action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&SimpleActionClient::timerCallback, this));
}

void bumperbot_cpp_examples::SimpleActionClient::timerCallback()
{
    timer_->cancel();

    if(!action_client_->wait_for_action_server())
    {
        RCLCPP_ERROR(get_logger(), "Action Server not available!");
        rclcpp::shutdown();
        return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = order_;

    RCLCPP_INFO(get_logger(), "Sending goal...");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&SimpleActionClient::goalCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&SimpleActionClient::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&SimpleActionClient::resultCallback, this, _1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void bumperbot_cpp_examples::SimpleActionClient::goalCallback(const rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr &goal_handle)
{
    if(!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server!");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Goal was accpeted by the server!");
    }
}

void bumperbot_cpp_examples::SimpleActionClient::feedbackCallback(rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> &feedback)
{
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for(auto num: feedback->partial_sequence)
    {
        ss << num << " ";
    }

    RCLCPP_INFO(get_logger(), ss.str().c_str());
}

void bumperbot_cpp_examples::SimpleActionClient::resultCallback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;

    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted!");
        return;

    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was cancelled!");
        return;
    
    default:
        RCLCPP_ERROR_STREAM(get_logger(), "Unkown result code: " << (int)result.code);
        return;
    }

    std::stringstream ss;
    ss << "Result received: ";
    for(auto num: result.result->sequence)
    {
        ss << num << " ";
    }

    RCLCPP_INFO(get_logger(), ss.str().c_str());

    rclcpp::shutdown();
}

RCLCPP_COMPONENTS_REGISTER_NODE(bumperbot_cpp_examples::SimpleActionClient);