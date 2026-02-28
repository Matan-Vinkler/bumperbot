#include "bumperbot_planning/a_star_planner.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rmw/qos_profiles.h>

#include <queue>

using std::placeholders::_1;

namespace bumperbot_planning
{
    AStarPlanner::AStarPlanner() : Node("a_star_planner")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS map_qos(10);
        map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap/costmap", map_qos, std::bind(&AStarPlanner::mapCallback, this, _1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&AStarPlanner::goalCallback, this, _1));
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/a_star/path", 10);
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/a_star/visited_map", 10);
    }

    void AStarPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
    {
        map_ = map_msg;
        visited_map_.header.frame_id = map_msg->header.frame_id;
        visited_map_.info = map_msg->info;
        visited_map_.data = std::vector<int8_t>(visited_map_.info.width * visited_map_.info.height, -1);
    }

    void AStarPlanner::goalCallback(const geometry_msgs::msg::PoseStamped& pose_msg)
    {
        if(!map_)
        {
            RCLCPP_ERROR(get_logger(), "No map received!");
            return;
        }

        visited_map_.data = std::vector<int8_t>(visited_map_.info.width * visited_map_.info.height, -1);
        geometry_msgs::msg::TransformStamped map_to_base_tf;
        try
        {
            map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_footprint", tf2::TimePointZero);
        }
        catch(const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(get_logger(), "Could not transform 'map' to 'base_footprint'");
            return;
        }

        geometry_msgs::msg::Pose map_to_base_pose;
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

        auto path = plan(map_to_base_pose, pose_msg.pose);
        if(!path.poses.empty())
        {
            RCLCPP_INFO(get_logger(), "Shortest path found!");
            path_pub_->publish(path);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "No path found to the goal.");
        }
        
    }

    nav_msgs::msg::Path AStarPlanner::plan(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& goal)
    {
        std::vector<std::pair<int, int>> explore_directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };
        std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
        std::vector<GraphNode> visited_nodes;

        auto start_grid = world2Grid(start);
        auto goal_grid = world2Grid(goal);
        start_grid.heruistic = manhattanDistance(start_grid, goal_grid);
        pending_nodes.push(start_grid);

        GraphNode active_node;
        while (!pending_nodes.empty() && rclcpp::ok())
        {
            active_node = pending_nodes.top();
            pending_nodes.pop();

            if(active_node == goal_grid)
            {
                break;
            }

            for(const auto& dir: explore_directions)
            {
                GraphNode new_node = active_node + dir;
                int current_cost = map_->data.at(pose2Cell(new_node));
                if(std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() && poseOnMap(new_node) && current_cost >= 0 && current_cost < 99)
                {
                    new_node.cost = active_node.cost + current_cost + 1;
                    new_node.prev = std::make_shared<GraphNode>(active_node);
                    new_node.heruistic = manhattanDistance(new_node, goal_grid);

                    pending_nodes.push(new_node);
                    visited_nodes.push_back(new_node);
                }
            }

            visited_map_.data.at(pose2Cell(active_node)) = -106;
            map_pub_->publish(visited_map_);
        }

        nav_msgs::msg::Path path;
        path.header.frame_id = map_->header.frame_id;
        while(active_node.prev && rclcpp::ok())
        {
            geometry_msgs::msg::Pose last_pose = grid2World(active_node);
            geometry_msgs::msg::PoseStamped last_pose_stamped;

            last_pose_stamped.header.frame_id = map_->header.frame_id;
            last_pose_stamped.pose = last_pose;

            path.poses.push_back(last_pose_stamped);

            active_node = *active_node.prev;
        }

        std::reverse(path.poses.begin(), path.poses.end());
        return path;
    }

    GraphNode AStarPlanner::world2Grid(const geometry_msgs::msg::Pose& pose)
    {
        int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
        int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);

        return GraphNode(grid_x, grid_y);
    }

    geometry_msgs::msg::Pose AStarPlanner::grid2World(const GraphNode& grid)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = grid.x * map_->info.resolution + map_->info.origin.position.x;
        pose.position.y = grid.y * map_->info.resolution + map_->info.origin.position.y;

        return pose;
    }

    bool AStarPlanner::poseOnMap(const GraphNode& node)
    {
        return 0 <= node.x && node.x < static_cast<int>(map_->info.width) && 0 <= node.y && node.y < static_cast<int>(map_->info.height);
    }

    unsigned int AStarPlanner::pose2Cell(const GraphNode& node)
    {
        return node.y * map_->info.width + node.x;
    }

    int AStarPlanner::manhattanDistance(const GraphNode& node1, const GraphNode& node2)
    {
        return abs(node1.x - node2.x) + abs(node1.y - node2.y);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<bumperbot_planning::AStarPlanner>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}