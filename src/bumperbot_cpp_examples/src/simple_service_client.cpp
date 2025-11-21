#include "bumperbot_cpp_examples/simple_service_client.hpp"

#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

SimpleServiceClient::SimpleServiceClient(int a, int b) : Node("simple_service_client")
{
    client_ = create_client<bumperbot_msgs::srv::AddTwoInts>("add_two_ints");

    while (!client_->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service...");
            return;
        }

        RCLCPP_INFO_STREAM(get_logger(), "Waiting for service...");
    }

    auto request = std::make_shared<bumperbot_msgs::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, this, _1));
}

void SimpleServiceClient::responseCallback(rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedFuture future)
{
    if(future.valid())
    {
        RCLCPP_INFO_STREAM(get_logger(), "Service response: " << future.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Service failure!");
    }
}

int main(int argc, char* argv[])
{
    if(argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: simple_service_client [a] [b]");
        return -1;
    }

    int a = atoi(argv[1]);
    int b = atoi(argv[2]);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<SimpleServiceClient>(a, b);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}