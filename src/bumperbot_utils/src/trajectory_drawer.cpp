#include "bumperbot_utils/trajectory_drawer.hpp"

using std::placeholders::_1;

TrajectoryDrawer::TrajectoryDrawer() : Node("trajectory_drawer")
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom", 10, std::bind(&TrajectoryDrawer::odomCallback, this, _1));
    traj_pub_ = create_publisher<nav_msgs::msg::Path>("bumperbot_controller/trajectory", 10);

    traj_msg_.header.frame_id = "odom";
}

void TrajectoryDrawer::odomCallback(const nav_msgs::msg::Odometry &odom_msg)
{
    auto pose_stamped = geometry_msgs::msg::PoseStamped();
    pose_stamped.pose = odom_msg.pose.pose;
    pose_stamped.header.stamp = get_clock()->now();
    pose_stamped.header.frame_id = "odom";

    traj_msg_.header.stamp = get_clock()->now();
    traj_msg_.poses.push_back(pose_stamped);

    traj_pub_->publish(traj_msg_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrajectoryDrawer>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}