#ifndef SIMPLE_SERVICE_CLIENT_HPP_
#define SIMPLE_SERVICE_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>

class SimpleServiceClient : public rclcpp::Node
{
public:
    SimpleServiceClient(int a, int b);

private:
    rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedPtr client_;

    void responseCallback(rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedFuture future);
};

#endif