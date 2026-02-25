#include "bumperbot_motion/pure_pursuit.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>

namespace bumperbot_motion
{
    PurePursuit::PurePursuit() : Node("pure_pursuit"), look_ahead_distance_(0.5), max_linear_velocity_(0.3), max_angular_velocity_(1.0)
    {
        declare_parameter<double>("look_ahead_distance", look_ahead_distance_);
        declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
        declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

        look_ahead_distance_ = get_parameter("look_ahead_distance").as_double();
        max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

        path_sub_ = create_subscription<nav_msgs::msg::Path>("/a_star/path", 10, std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        carrot_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/carrot", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PurePursuit::controlLoop, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        global_plan_ = path_msg;
    }

    void PurePursuit::controlLoop()
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

        geometry_msgs::msg::PoseStamped carrot_pose_stamped = getCarrotPose(robot_pose_stamped);

        double distance = get_poses_distance(carrot_pose_stamped, robot_pose_stamped);

        if(distance <= 0.1)
        {
            RCLCPP_INFO(get_logger(), "Goal Reached!");
            global_plan_->poses.clear();
            return;
        }

        carrot_pose_pub_->publish(carrot_pose_stamped);

        tf2::Transform robot_pose_tf, carrot_pose_tf;
        tf2::fromMsg(robot_pose_stamped.pose, robot_pose_tf);
        tf2::fromMsg(carrot_pose_stamped.pose, carrot_pose_tf);

        tf2::Transform carrot_robot_pose_tf = robot_pose_tf.inverse() * carrot_pose_tf;
        tf2::toMsg(carrot_robot_pose_tf, carrot_pose_stamped.pose);

        double curvature = getCurvature(carrot_pose_stamped.pose);

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = max_linear_velocity_;
        cmd_vel.angular.z = curvature * max_angular_velocity_;
        cmd_pub_->publish(cmd_vel);
    }

    bool PurePursuit::transformPlan(const std::string& frame)
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

    geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(const geometry_msgs::msg::PoseStamped& robot_pose)
    {
        auto carrot_pose = global_plan_->poses.back();
        for(auto pose_it = global_plan_->poses.rbegin(); pose_it != global_plan_->poses.rend(); ++pose_it)
        {
            double distance = get_poses_distance(*pose_it, robot_pose);
            if(distance > look_ahead_distance_)
            {
                carrot_pose = *pose_it;
            }
            else
            {
                break;
            }
        }

        return carrot_pose;
    }

    double PurePursuit::getCurvature(const geometry_msgs::msg::Pose& carrot_pose)
    {
        double L_squared = carrot_pose.position.x * carrot_pose.position.x + carrot_pose.position.y * carrot_pose.position.y;
        if(L_squared > 0.001)
        {
            return 2.0 * carrot_pose.position.y / L_squared;
        }
        else
        {
            return 0.0;
        }
    }

    double PurePursuit::get_poses_distance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2) const
    {
        double dx = pose1.pose.position.x - pose2.pose.position.x;
        double dy = pose1.pose.position.y - pose2.pose.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<bumperbot_motion::PurePursuit>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}