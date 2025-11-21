#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <vector>
#include <string>

using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node
{
public:
    SimpleParameter() : Node("simple_parameter")
    {
        declare_parameter<int>("param1", 32);
        declare_parameter<std::string>("param2", "Hello!!");

        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
    }

private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for(const auto& param: params)
        {
            if(param.get_name() == "param1" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "'param1' changed to " << param.as_int());
                result.successful = true;
            }

            else if(param.get_name() == "param2" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "'param2' changed to " << param.as_string());
                result.successful = true;
            }
        }

        return result;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}