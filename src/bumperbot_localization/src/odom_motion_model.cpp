#include "bumperbot_localization/odom_motion_model.hpp"

#include <tf2/utils.hpp>

#include <cmath>
#include <random>

using std::placeholders::_1;

double angle_diff(double a, double b)
{
    a = atan2(sin(a), cos(a));
    b = atan2(sin(b), cos(b));

    double d1 = a - b;
    double d2 = 2 * M_PI - fabs(d1);

    if(d1 > 0)
    {
        d2 *= -1.0;
    }

    if(fabs(d1) < fabs(d2))
    {
        return d1;
    }

    return d2;
}

OdomMotionModel::OdomMotionModel() : Node("odom_motion_model"), 
    is_first_odom_(true), last_odom_x_(0.0), last_odom_y_(0.0), last_odom_theta_(0.0),
    alpha1_(0.0), alpha2_(0.0), alpha3_(0.0), alpha4_(0.0),
    n_samples_(300)
{
    declare_parameter("alpha1", 0.1);
    declare_parameter("alpha2", 0.1);
    declare_parameter("alpha3", 0.1);
    declare_parameter("alpha4", 0.1);
    declare_parameter("n_samples", 300);

    alpha1_ = get_parameter("alpha1").as_double();
    alpha2_ = get_parameter("alpha2").as_double();
    alpha3_ = get_parameter("alpha3").as_double();
    alpha4_ = get_parameter("alpha4").as_double();
    n_samples_ = get_parameter("n_samples").as_int();

    if(n_samples_ <= 0)
    {
        RCLCPP_FATAL_STREAM(get_logger(), "Invalid number of samples: " << n_samples_);
        return;
    }

    samples_.poses = std::vector<geometry_msgs::msg::Pose>(n_samples_, geometry_msgs::msg::Pose());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom", 10, std::bind(&OdomMotionModel::odomCallback, this, _1));
    pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("odometry_motion_model/samples", 10);
}

void OdomMotionModel::odomCallback(const nav_msgs::msg::Odometry& msg)
{
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(is_first_odom_)
    {
        last_odom_x_ = msg.pose.pose.position.x;
        last_odom_y_ = msg.pose.pose.position.y;
        last_odom_theta_ = yaw;

        samples_.header.frame_id = msg.header.frame_id;

        is_first_odom_ = false;

        return;
    }

    double odom_x_inc = msg.pose.pose.position.x - last_odom_x_;
    double odom_y_inc = msg.pose.pose.position.y - last_odom_y_;
    double odom_theta_inc = angle_diff(yaw, last_odom_theta_);

    //double delta_trasl = sqrt(std::pow(odom_y_inc, 2) + std::pow(odom_x_inc, 2));
    double delta_trasl = odom_x_inc * cos(last_odom_theta_) + odom_y_inc * sin(last_odom_theta_);

    double delta_rot1 = 0.0;
    if(sqrt(std::pow(odom_y_inc, 2) + std::pow(odom_x_inc, 2)) >= 0.01)
    {
        delta_rot1 = angle_diff(atan2(odom_y_inc, odom_x_inc), yaw);
    }
    
    double delta_rot2 = angle_diff(odom_theta_inc, delta_rot1);

    double rot1_var = alpha1_ * delta_rot1 + alpha2_ * delta_trasl;
    double trasl_var = alpha3_ * delta_trasl + alpha4_ * (delta_rot1 + delta_rot2);
    double rot2_var = alpha1_ * delta_rot2 + alpha2_ * delta_trasl;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_gen(seed);

    std::normal_distribution<double> rot1_noise(0.0, rot1_var);
    std::normal_distribution<double> trasl_noise(0.0, trasl_var);
    std::normal_distribution<double> rot2_noise(0.0, rot2_var);

    for(auto& sample: samples_.poses)
    {
        double delta_rot1_draw = angle_diff(delta_rot1, rot1_noise(noise_gen));
        double delta_trasl_draw = delta_trasl - trasl_noise(noise_gen);
        double delta_rot2_draw = angle_diff(delta_rot2, rot2_noise(noise_gen));

        tf2::Quaternion sample_q(sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w);
        tf2::Matrix3x3 sample_m(sample_q);

        double sample_roll, sample_pitch, sample_yaw;
        sample_m.getRPY(sample_roll, sample_pitch, sample_yaw);

        sample.position.x += delta_trasl_draw * cos(sample_yaw + delta_rot1_draw);
        sample.position.y += delta_trasl_draw * sin(sample_yaw + delta_rot1_draw);

        tf2::Quaternion new_q;
        new_q.setRPY(0.0, 0.0, sample_yaw + delta_rot1_draw + delta_rot2_draw);
        sample.orientation.x = new_q.getX();
        sample.orientation.y = new_q.getY();
        sample.orientation.z = new_q.getZ();
        sample.orientation.w = new_q.getW();
    }

    last_odom_x_ = msg.pose.pose.position.x;
    last_odom_y_ = msg.pose.pose.position.y;
    last_odom_theta_ = yaw;

    pose_array_pub_->publish(samples_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OdomMotionModel>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
