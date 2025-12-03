#ifndef DIJKSTRA_PLANNER_HPP_
#define DIJKSTRA_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace bumperbot_planning
{
    struct GraphNode
    {
        int x;
        int y;
        int cost;
        std::shared_ptr<GraphNode> prev;

        GraphNode(int in_x = 0, int in_y = 0) : x(in_x), y(in_y), cost(0) {}

        bool operator>(const GraphNode& other) const
        {
            return cost > other.cost;
        }

        bool operator==(const GraphNode& other) const
        {
            return x == other.x && y == other.y;
        }

        GraphNode operator+(std::pair<int, int> const& other)
        {
            GraphNode res(x + other.first, y + other.second);
            return res;
        }
    };

    class DijkstraPlanner : public rclcpp::Node
    {
    public:
        DijkstraPlanner();

    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        nav_msgs::msg::OccupancyGrid visited_map_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
        void goalCallback(const geometry_msgs::msg::PoseStamped& pose_msg);

        nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& goal);

        GraphNode world2Grid(const geometry_msgs::msg::Pose& pose);
        geometry_msgs::msg::Pose grid2World(const GraphNode& grid);
        bool poseOnMap(const GraphNode& node);
        unsigned int pose2Cell(const GraphNode& node);
    };
}

#endif