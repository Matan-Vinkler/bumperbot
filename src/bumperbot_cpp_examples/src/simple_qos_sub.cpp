#include "bumperbot_cpp_examples/simple_qos_sub.hpp"

using std::placeholders::_1;

SimpleQoSSub::SimpleQoSSub() : Node("simple_qos_sub"), qos_profile_sub_(10)
{
    declare_parameter<std::string>("reliability", "system_default");
    declare_parameter<std::string>("durability", "system_default");

    const auto reliability = get_parameter("reliability").as_string();
    const auto durability = get_parameter("durability").as_string();

    if(reliability == "best_effort")
    {
        qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        RCLCPP_INFO(get_logger(), "[Reliability] : Best Effort");
    }
    else if(reliability == "reliable")
    {
        qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        RCLCPP_INFO(get_logger(), "[Reliability] : Reliable");
    }
    else if(reliability == "system_default")
    {
        qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
        RCLCPP_INFO(get_logger(), "[Reliability] : System Default");
    }
    else
    {
        RCLCPP_ERROR_STREAM(get_logger(), "[Reliability]: Invalid (" << reliability << ")");
        return;
    }

    if(durability == "volatile")
    {
        qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        RCLCPP_INFO(get_logger(), "[Durability] : Best Effort");
    }
    else if(durability == "transient_local")
    {
        qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        RCLCPP_INFO(get_logger(), "[Durability] : Reliable");
    }
    else if(durability == "system_default")
    {
        qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
        RCLCPP_INFO(get_logger(), "[Durability] : System Default");
    }
    else
    {
        RCLCPP_ERROR_STREAM(get_logger(), "[Durability]: Invalid (" << durability << ")");
        return;
    }

    sub_= create_subscription<std_msgs::msg::String>("chatter", qos_profile_sub_, std::bind(&SimpleQoSSub::msgCallback, this, _1));

    RCLCPP_INFO(get_logger(), "[Initialized subscriber!]");
}

void SimpleQoSSub::msgCallback(const std_msgs::msg::String &message) const
{
    RCLCPP_INFO_STREAM(get_logger(), "Message received: " << message.data.c_str());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SimpleQoSSub>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();

    return 0;
}