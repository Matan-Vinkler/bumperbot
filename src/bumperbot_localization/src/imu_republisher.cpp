#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

// Gyro z bias measured at rest (rad/s). Run calibration, then set this.
static constexpr double GYRO_Z_BIAS = 0.0;

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

void imuCallback(const sensor_msgs::msg::Imu& msg)
{
    sensor_msgs::msg::Imu new_msg;
    new_msg = msg;

    new_msg.header.frame_id = "imu_link_ekf";

    new_msg.angular_velocity.z -= GYRO_Z_BIAS;

    // Signal that orientation and linear acceleration are not available
    new_msg.orientation_covariance[0] = -1.0;
    new_msg.linear_acceleration_covariance[0] = -1.0;

    // Angular velocity z variance (rad/s)^2 — tuned for MPU6050 at ±250 dps
    new_msg.angular_velocity_covariance[8] = 0.01;

    imu_pub->publish(new_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("imu_republisher_node");

    rclcpp::sleep_for(1s);

    imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_ekf", 10);
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>("imu/out", 10, imuCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
