#ifndef SIMPLE_TF_KINEMATICS_HPP_
#define SIMPLE_TF_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <bumperbot_msgs/srv/get_transform.hpp>

#include <memory>

class SimpleTfKinematics : public rclcpp::Node
{
public:
    SimpleTfKinematics();

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;

    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<bumperbot_msgs::srv::GetTransform>::SharedPtr get_transform_srv_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;

    double x_increment_;
    double last_x_;
    double max_x_;
    int rotation_counter_;
    tf2::Quaternion last_orientation_;
    tf2::Quaternion orientation_increment_;

    void timerCallback();
    void getTransformCallback(const bumperbot_msgs::srv::GetTransform::Request::SharedPtr req, const bumperbot_msgs::srv::GetTransform::Response::SharedPtr res);
};

#endif