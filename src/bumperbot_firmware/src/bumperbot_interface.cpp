#include "bumperbot_firmware/bumperbot_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace bumperbot_firmware
{
    BumperbotInterface::BumperbotInterface() {}

    BumperbotInterface::~BumperbotInterface()
    {
        if(arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("bumperbot_interface"), "Something went wrong while closing connection with port " << port_);
            }
        }
    }

    CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(rclcpp::get_logger("bumperbot_interface"), "Starting robot hardware...");

        std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
        std::fill(position_states_.begin(), position_states_.end(), 0.0);
        std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("bumperbot_interface"), "Something went wrong while opening connection with port " << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("bumperbot_interface"), "Hardware started, ready to take commands!");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(rclcpp::get_logger("bumperbot_interface"), "Stopping robot hardware...");
        if(arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("bumperbot_interface"), "Something went wrong while closing port " << port_);
                return CallbackReturn::FAILURE;
            }
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range& e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("bumperbot_interface"), "No Serial Port provided! Aborting...");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.resize(info_.joints.size(), 0.0);
        position_states_.resize(info_.joints.size(), 0.0);
        velocity_states_.resize(info_.joints.size(), 0.0);

        last_run_ = rclcpp::Clock().now();
        
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type BumperbotInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
    {
        if(arduino_.IsDataAvailable())
        {
            auto dt = (rclcpp::Clock().now() - last_run_).seconds();

            std::string msg;
            try
            {
                arduino_.ReadLine(msg, '\n', 10);
            }
            catch(const LibSerial::ReadTimeout&)
            {
                return hardware_interface::return_type::OK;
            }

            std::stringstream ss(msg);

            std::string res;
            int multiplier = 1;

            while (std::getline(ss, res, ','))
            {
                if (res.size() < 2) continue;
                multiplier = res.at(1) == 'p' ? 1 : -1;
                if(res.at(0) == 'r')
                {
                    velocity_states_.at(1) = -std::stod(res.substr(2, res.size())) * multiplier;
                    position_states_.at(1) += velocity_states_.at(1) * dt;
                }
                if(res.at(0) == 'l')
                {
                    velocity_states_.at(0) = std::stod(res.substr(2, res.size())) * multiplier;
                    position_states_.at(0) += velocity_states_.at(0) * dt;
                }
            }

            last_run_ = rclcpp::Clock().now();
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BumperbotInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
    {
        std::stringstream msg_stream;

        char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
        char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';

        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if(std::abs(velocity_commands_.at(0)) < 10.0)
        {
            compensate_zeros_right = "0";
        }
        else
        {
            compensate_zeros_right = "";
        }

        if(std::abs(velocity_commands_.at(1)) < 10.0)
        {
            compensate_zeros_left = "0";
        }
        else
        {
            compensate_zeros_left = "";
        }

        // 'r' channel is physically the left motor, 'l' channel is physically the right motor
        msg_stream << std::fixed << std::setprecision(2) << "r" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) <<
            ",l" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) << ",";

        try
        {
            arduino_.Write(msg_stream.str());
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("bumperbot_interface"), "Something went wrong while sending message '" << msg_stream.str() << "' on port " << port_);
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::BumperbotInterface, hardware_interface::SystemInterface);