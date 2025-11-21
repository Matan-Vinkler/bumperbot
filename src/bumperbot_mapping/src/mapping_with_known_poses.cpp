#include "bumperbot_mapping/mapping_with_known_poses.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.hpp>

using std::placeholders::_1;

using bumperbot_mapping::Pose;

std::vector<Pose> bumperbot_mapping::bresenham(const Pose & start, const Pose & end)
{
    // Implementation of Bresenham's line drawing algorithm
    // See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    std::vector<Pose> line;

    int dx = end.x_ - start.x_;
    int dy = end.y_ - start.y_;

    int xsign = dx > 0 ? 1 : -1;
    int ysign = dy > 0 ? 1 : -1;

    dx = std::abs(dx);
    dy = std::abs(dy);

    int xx, xy, yx, yy;
    if(dx > dy)
    {
        xx = xsign;
        xy = 0;
        yx = 0;
        yy = ysign;
    }
    else
    {
        int tmp = dx;
        dx = dy;
        dy = tmp;
        xx = 0;
        xy = ysign;
        yx = xsign;
        yy = 0;
    }

    int D = 2 * dy - dx;
    int y = 0;

    line.reserve(dx + 1);
    for (int i = 0; i < dx + 1; i++)
    {
        line.emplace_back(Pose(start.x_ + i * xx + y * yx, start.y_ + i * xy + y * yy));
        if(D >= 0)
        {
            y++;
            D -= 2 * dx;
        }
        D += 2 * dy;
    }

    return line;
}

std::vector<std::pair<Pose, double>> bumperbot_mapping::inverseSensorModel(const Pose& robot_p, const Pose& beam_p)
{
    std::vector<std::pair<Pose, double>> occ_values;
    auto line = bresenham(robot_p, beam_p);
    occ_values.reserve(line.size());

    for(size_t i = 0; i < line.size() - 1; i++)
    {
        occ_values.emplace_back(std::pair<Pose, double>(line.at(i), FREE_PROB));
    }

    occ_values.emplace_back(std::pair<Pose, double>(line.back(), OCC_PROB));

    return occ_values;
}

double bumperbot_mapping::prob2logodds(double p)
{
    return std::log(p / (1 - p));
}
double bumperbot_mapping::logodds2prob(double l)
{
    return 1 - (1 / (1 + std::exp(l)));
}
bumperbot_mapping::MappingWithKnownPoses::MappingWithKnownPoses() : Node("mapping_with_known_poses")
{
    declare_parameter<double>("width", 50.0);
    declare_parameter<double>("height", 50.0);
    declare_parameter<double>("resolution", 0.1);

    double width = get_parameter("width").as_double();
    double height = get_parameter("height").as_double();
    double resolution = get_parameter("resolution").as_double();

    map_.info.resolution = resolution;
    map_.info.width = std::round(width / resolution);
    map_.info.height = std::round(height / resolution);

    map_.info.origin.position.x = -std::round(width / 2.0);
    map_.info.origin.position.y = -std::round(height / 2.0);

    map_.header.frame_id = "odom";
    
    map_.data = std::vector<int8_t>(map_.info.width * map_.info.height, -1);
    prob_map_ = std::vector<double>(map_.info.width * map_.info.height, prob2logodds(PRIOR_PROB));

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&MappingWithKnownPoses::scanCallback, this, _1));
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MappingWithKnownPoses::timerCallback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void bumperbot_mapping::MappingWithKnownPoses::scanCallback(const sensor_msgs::msg::LaserScan &scan_msg)
{
    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(map_.header.frame_id, scan_msg.header.frame_id, tf2::TimePointZero);
    }
    catch(const tf2::TransformException& exc)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Unable to transform between " << map_.header.frame_id << " and " << scan_msg.header.frame_id);
        return;
    }

    Pose robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, map_.info);
    if(!poseOnMap(robot_p, map_.info))
    {
        RCLCPP_ERROR(get_logger(), "Robot is not on the map!");
        return;
    }

    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    for(size_t i = 0; i < scan_msg.ranges.size(); i++)
    {
        double angle = scan_msg.angle_min + (scan_msg.angle_increment * i) + yaw;

        double px = scan_msg.ranges.at(i) * std::cos(angle) + t.transform.translation.x;
        double py = scan_msg.ranges.at(i) * std::sin(angle) + t.transform.translation.y;

        Pose beam_p = coordinatesToPose(px, py, map_.info);
        if(!poseOnMap(beam_p, map_.info))
        {
            continue;
        }

        auto poses = inverseSensorModel(robot_p, beam_p);
        for(const auto& pose: poses)
        {
            if(poseOnMap(pose.first, map_.info))
            {
                unsigned int cell = poseToCell(pose.first, map_.info);
                prob_map_.at(cell) += prob2logodds(pose.second) - prob2logodds(PRIOR_PROB);
            }
        }
    }
}

void bumperbot_mapping::MappingWithKnownPoses::timerCallback()
{
    map_.header.stamp = get_clock()->now();
    std::transform(prob_map_.begin(), prob_map_.end(), map_.data.begin(), [](double value) {return logodds2prob(value) * 100;});
    map_pub_->publish(map_);
}

unsigned int bumperbot_mapping::poseToCell(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info)
{
    return map_info.width * pose.y_ + pose.x_;
}

Pose bumperbot_mapping::coordinatesToPose(const double px, const double py, const nav_msgs::msg::MapMetaData &map_info)
{
    Pose pose;

    pose.x_ = std::round((px - map_info.origin.position.x) / map_info.resolution);
    pose.y_ = std::round((py - map_info.origin.position.y) / map_info.resolution);

    return pose;
}

bool bumperbot_mapping::poseOnMap(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info)
{
    return pose.x_ < static_cast<int>(map_info.width) && pose.x_ >= 0 && pose.y_ < static_cast<int>(map_info.height) && pose.y_ >= 0;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<bumperbot_mapping::MappingWithKnownPoses>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}