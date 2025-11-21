#include "bumperbot_cpp_examples/simple_lifecycle.hpp"

#include <thread>

using std::placeholders::_1;
using namespace std::chrono_literals;

SimpleLifeCycle::SimpleLifeCycle(bool intra_process_comms = false) : rclcpp_lifecycle::LifecycleNode("simple_lifecycle", rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifeCycle::on_configure(const rclcpp_lifecycle::State &)
{
    sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleLifeCycle::msgCallback, this, _1));
    RCLCPP_INFO(get_logger(), "Lifecycle node on_configure() called (C++)");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifeCycle::on_shutdown(const rclcpp_lifecycle::State &)
{
    sub_.reset();
    RCLCPP_INFO(get_logger(), "Lifecycle node on_shutdown() called (C++)");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifeCycle::on_cleanup(const rclcpp_lifecycle::State &)
{
    sub_.reset();
    RCLCPP_INFO(get_logger(), "Lifecycle node on_cleanup() called (C++)");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifeCycle::on_activate(const rclcpp_lifecycle::State &state)
{
    LifecycleNode::on_activate(state);
    RCLCPP_INFO(get_logger(), "Lifecycle node on_activate() called (C++)");
    std::this_thread::sleep_for(2s);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifeCycle::on_deactivate(const rclcpp_lifecycle::State &state)
{
    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(get_logger(), "Lifecycle node on_deactivate() called (C++)");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
void SimpleLifeCycle::msgCallback(const std_msgs::msg::String &msg)
{
    auto current_state = get_current_state();
    if(current_state.label() == "active")
    {
        RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data.c_str());
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor ste;
    auto node = std::make_shared<SimpleLifeCycle>();

    ste.add_node(node->get_node_base_interface());
    ste.spin();

    rclcpp::shutdown();

    return 0;
}