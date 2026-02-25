#include "bumperbot_motion/pd_motion_planner.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>

namespace bumperbot_motion
{
    PDMotionPlanner::PDMotionPlanner() : Node("pd_motion_planner"), kp_(2.0), kd_(0.1), step_size_(0.2), 
    max_linear_velocity_(0.3), max_angular_velocity_(1.0),
    prev_linear_error_(0.0), prev_angular_error_(0.0)
    {
        declare_parameter<double>("kp", kp_);
        declare_parameter<double>("kd", kd_);
        declare_parameter<double>("step_size", step_size_);
        declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
        declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

        kp_ = get_parameter("kp").as_double();
        kd_ = get_parameter("kd").as_double();
        step_size_ = get_parameter("step_size").as_double();
        max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

        path_sub_ = create_subscription<nav_msgs::msg::Path>("/a_star/path", 10, std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PDMotionPlanner::controlLoop, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        last_cycle_time_ = get_clock()->now();
    }

    void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        global_plan_ = path_msg;
    }

    void PDMotionPlanner::controlLoop()
    {
        if((!global_plan_) || (global_plan_->poses.empty()))
        {
            return;
        }
        
        geometry_msgs::msg::TransformStamped robot_transform_pose;
        try
        {
            robot_transform_pose = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        }
        catch(tf2::LookupException& e)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Could not transform: " << e.what());
            return;
        }
        
        if(!transformPlan(robot_transform_pose.header.frame_id))
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Unable to transform plan in robot's frame");
            return;
        }

        geometry_msgs::msg::PoseStamped robot_pose_stamped;
        robot_pose_stamped.header.frame_id = robot_transform_pose.header.frame_id;
        robot_pose_stamped.pose.position.x = robot_transform_pose.transform.translation.x;
        robot_pose_stamped.pose.position.y = robot_transform_pose.transform.translation.y;
        robot_pose_stamped.pose.orientation = robot_transform_pose.transform.rotation;

        geometry_msgs::msg::PoseStamped next_pose_stamped = getNextPose(robot_pose_stamped);

        double distance = get_poses_distance(next_pose_stamped, robot_pose_stamped);

        if(distance <= 0.1)
        {
            RCLCPP_INFO(get_logger(), "Goal Reached!");
            global_plan_->poses.clear();
            return;
        }

        next_pose_pub_->publish(next_pose_stamped);

        tf2::Transform robot_pose_tf, next_pose_tf;
        tf2::fromMsg(robot_pose_stamped.pose, robot_pose_tf);
        tf2::fromMsg(next_pose_stamped.pose, next_pose_tf);
        tf2::Transform next_pose_robot_tf = robot_pose_tf.inverse() * next_pose_tf;

        double linear_error = next_pose_robot_tf.getOrigin().getX();
        double angular_error = next_pose_robot_tf.getOrigin().getY();

        double dt = (get_clock()->now() - last_cycle_time_).seconds();
        double linear_error_derivative = (linear_error - prev_linear_error_) / dt;
        double angular_error_derivative = (angular_error - prev_angular_error_) / dt;

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative, -max_linear_velocity_, max_linear_velocity_);
        cmd_vel.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative, -max_angular_velocity_, max_angular_velocity_);
        cmd_pub_->publish(cmd_vel);

        prev_linear_error_ = linear_error;
        prev_angular_error_ = angular_error;
        last_cycle_time_ = get_clock()->now();
    }

    bool PDMotionPlanner::transformPlan(const std::string& frame)
    {
        if(global_plan_->header.frame_id == frame)
        {
            return true;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(frame, global_plan_->header.frame_id, tf2::TimePointZero);
        }
        catch(tf2::LookupException& ex)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Couldn't transform plan from " << global_plan_->header.frame_id << " to " << frame << ": " << ex.what());
            return false;
        }

        for(auto& pose: global_plan_->poses)
        {
            tf2::doTransform(pose, pose, transform);
        }

        global_plan_->header.frame_id = frame;
        return true;
    }

    geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped& robot_pose)
    {
        auto next_pose = global_plan_->poses.back();
        for(auto pose_it = global_plan_->poses.rbegin(); pose_it != global_plan_->poses.rend(); ++pose_it)
        {
            double distance = get_poses_distance(*pose_it, robot_pose);
            if(distance > step_size_)
            {
                next_pose = *pose_it;
            }
            else
            {
                break;
            }
        }

        return next_pose;
    }

    double PDMotionPlanner::get_poses_distance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2) const
    {
        double dx = pose1.pose.position.x - pose2.pose.position.x;
        double dy = pose1.pose.position.y - pose2.pose.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<bumperbot_motion::PDMotionPlanner>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}