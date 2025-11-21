#ifndef SIMPLE_SERVICE_SERVER_HPP_
#define SIMPLE_SERVICE_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>

class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer();

private:
    rclcpp::Service<bumperbot_msgs::srv::AddTwoInts>::SharedPtr service_;

    void serviceCallback(bumperbot_msgs::srv::AddTwoInts::Request::SharedPtr req, bumperbot_msgs::srv::AddTwoInts::Response::SharedPtr res);
};

#endif