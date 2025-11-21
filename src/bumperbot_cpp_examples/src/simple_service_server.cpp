#include "bumperbot_cpp_examples/simple_service_server.hpp"

using namespace std::placeholders;

SimpleServiceServer::SimpleServiceServer() : Node("simple_service_server")
{
    service_ = create_service<bumperbot_msgs::srv::AddTwoInts>("add_two_ints", std::bind(&SimpleServiceServer::serviceCallback, this, _1, _2));

    RCLCPP_INFO_STREAM(get_logger(), "Service 'add_two_ints' ready!");
}

void SimpleServiceServer::serviceCallback(bumperbot_msgs::srv::AddTwoInts::Request::SharedPtr req, bumperbot_msgs::srv::AddTwoInts::Response::SharedPtr res)
{
    RCLCPP_INFO_STREAM(get_logger(), "Receiving: a=" << req->a << ", b=" << req->b);
    res->sum = req->a + req->b;
    RCLCPP_INFO_STREAM(get_logger(), "Response: sum=" << res->sum);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}