#include "bumperbot_localization/kalman_filter.hpp"

using std::placeholders::_1;

KalmanFilter::KalmanFilter() : 
    Node("kalman_filter"), 
    mean_(0.0), 
    variance_(1000.0), 
    imu_angular_z_(0.0), 
    is_first_odom_(true), 
    last_angular_z_(0.0), 
    motion_(0.0),
    motion_variance_(4.0),
    measurement_variance_(0.5)
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom_noisy", 10, std::bind(&KalmanFilter::odomCallback, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/out", 10, std::bind(&KalmanFilter::imuCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("bumperbot_controller/odom_kalman", 10);
}

void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry& msg)
{
    kalman_odom_ = msg;

    if(is_first_odom_)
    {
        mean_ = msg.twist.twist.angular.z;
        last_angular_z_ = msg.twist.twist.angular.z;

        is_first_odom_ = false;
    }
    else
    {
        motion_ = msg.twist.twist.angular.z - last_angular_z_;

        statePredict();
        measurementUpdate();

        // Update for the next iteration
        last_angular_z_ = msg.twist.twist.angular.z;

        kalman_odom_.twist.twist.angular.z = mean_;
        odom_pub_->publish(kalman_odom_);
    }
}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu& msg)
{
    imu_angular_z_ = msg.angular_velocity.z;
}

void KalmanFilter::statePredict()
{
    mean_ += motion_;
    variance_ += motion_variance_;
}

void KalmanFilter::measurementUpdate()
{
    mean_ = (measurement_variance_ * mean_ + variance_ * imu_angular_z_) / (variance_ + measurement_variance_);
    variance_ = (variance_ * measurement_variance_) / (variance_ + measurement_variance_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KalmanFilter>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
