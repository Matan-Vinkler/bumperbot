#include "bumperbot_cpp_examples/simple_action_server.hpp"

#include <thread>

using namespace std::placeholders;

bumperbot_cpp_examples::SimpleActionServer::SimpleActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("simple_action_server", options)
{
    action_server_ = rclcpp_action::create_server<bumperbot_msgs::action::Fibonacci>(
        this, 
        "fibonacci",
        std::bind(&SimpleActionServer::goalCallback, this, _1, _2),
        std::bind(&SimpleActionServer::cancelCallback, this, _1),
        std::bind(&SimpleActionServer::acceptedCallback, this, _1)
    );

    RCLCPP_INFO_STREAM(get_logger(), "Starting the Action server!!!!!!!!");
}

rclcpp_action::GoalResponse bumperbot_cpp_examples::SimpleActionServer::goalCallback(const rclcpp_action::GoalUUID&, std::shared_ptr<const bumperbot_msgs::action::Fibonacci::Goal> goal)
{
    RCLCPP_INFO_STREAM(get_logger(), "Received goal request with order: " << goal->order);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void bumperbot_cpp_examples::SimpleActionServer::acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
{
    std::thread{std::bind(&SimpleActionServer::execute, this, _1), goal_handle}.detach();
}

rclcpp_action::CancelResponse bumperbot_cpp_examples::SimpleActionServer::cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>>)
{
    RCLCPP_INFO_STREAM(get_logger(), "Received request to cancel goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void bumperbot_cpp_examples::SimpleActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
{
    RCLCPP_INFO_STREAM(get_logger(), "Executing goal!");

    rclcpp::Rate loop_rate(1);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<bumperbot_msgs::action::Fibonacci::Feedback>();
    auto& sequence = feedback->partial_sequence;

    sequence.push_back(0);
    sequence.push_back(1);

    auto result = std::make_shared<bumperbot_msgs::action::Fibonacci::Result>();

    for(int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
        if(goal_handle->is_canceling())
        {
            result->sequence = sequence;
            goal_handle->canceled(result);

            RCLCPP_INFO_STREAM(get_logger(), "Goal cancelled!");

            return;
        }

        sequence.push_back(sequence[i] + sequence[i - 1]);
        goal_handle->publish_feedback(feedback);
        
        RCLCPP_INFO_STREAM(get_logger(), "Publishing feedback...");

        loop_rate.sleep();
    }

    if(rclcpp::ok())
    {
        result->sequence = sequence;
        goal_handle->succeed(result);

        RCLCPP_INFO_STREAM(get_logger(), "Goal succeeded!");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(bumperbot_cpp_examples::SimpleActionServer);