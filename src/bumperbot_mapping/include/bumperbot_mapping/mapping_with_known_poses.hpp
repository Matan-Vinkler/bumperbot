#ifndef MAPPING_WITH_KNOWN_POSES_HPP_
#define MAPPING_WITH_KNOWN_POSES_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>

namespace bumperbot_mapping
{
    inline const double PRIOR_PROB = 0.5;
    inline const double OCC_PROB = 0.9;
    inline const double FREE_PROB = 0.35;

    struct Pose
    {
        Pose() = default;
        Pose(const int px, const int py) : x_(px), y_(py) {}
        int x_;
        int y_;
    };

    unsigned int poseToCell(const Pose& pose, const nav_msgs::msg::MapMetaData& map_info);
    Pose coordinatesToPose(const double px, const double py, const nav_msgs::msg::MapMetaData& map_info);
    bool poseOnMap(const Pose& pose, const nav_msgs::msg::MapMetaData& map_info);

    std::vector<Pose> bresenham(const Pose & start, const Pose & end);
    std::vector<std::pair<Pose, double>> inverseSensorModel(const Pose& robot_p, const Pose& beam_p);

    double prob2logodds(double p);
    double logodds2prob(double l);

    class MappingWithKnownPoses : public rclcpp::Node
    {
    public:
        MappingWithKnownPoses();
    private:
        void scanCallback(const sensor_msgs::msg::LaserScan& scan_msg);
        void timerCallback();

        nav_msgs::msg::OccupancyGrid map_;
        std::vector<double> prob_map_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    };
};

#endif