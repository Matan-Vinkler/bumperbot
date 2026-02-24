#ifndef PD_MOTION_PLANNER_HPP_
#define PD_MOTION_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <nav_msgs/msg/path.hpp>

namespace bumperbot_motion
{
    class PDMotionPlanner : public rclcpp::Node
    {
    public:
        PDMotionPlanner();

    private:
        double kp_;
        double kd_;
        double step_size_;
        double max_linear_velocity_;
        double max_angular_velocity_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Path::SharedPtr global_plan_;

        double prev_linear_error_;
        double prev_angular_error_;
        rclcpp::Time last_cycle_time_;

        void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg);
        void controlLoop();
        bool transformPlan(const std::string& frame);
        geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped& robot_pose);

        double get_poses_distance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2) const;
    };
}

#endif