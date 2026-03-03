#ifndef A_STAR_PLANNER_HPP_
#define A_STAR_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <nav2_core/global_planner.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

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
        int heruistic;
        std::shared_ptr<GraphNode> prev;

        GraphNode(int in_x = 0, int in_y = 0) : x(in_x), y(in_y), cost(0), heruistic(0) {}

        bool operator>(const GraphNode& other) const
        {
            return cost + heruistic > other.cost + other.heruistic;
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

    class AStarPlanner : public nav2_core::GlobalPlanner
    {
    public:
        AStarPlanner() = default;
        ~AStarPlanner() = default;

        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) override;

    private:
        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_util::LifecycleNode::SharedPtr node_;
        nav2_costmap_2d::Costmap2D* costmap_;
        std::string global_frame_, name_;

        GraphNode world2Grid(const geometry_msgs::msg::Pose& pose);
        geometry_msgs::msg::Pose grid2World(const GraphNode& grid);
        bool poseOnMap(const GraphNode& node);
        unsigned int pose2Cell(const GraphNode& node);

        int manhattanDistance(const GraphNode& node1, const GraphNode& node2);
    };
}

#endif