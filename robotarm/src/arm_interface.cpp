#include "robotarm/arm_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp> 
#include <pluginlib/class_list_macros.hpp>

namespace arm_controller
{
    ArmInterface::ArmInterface()
    {

    }

    ArmInterface::~ArmInterface()
    {
        if(arduino_.IsOpen()){
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArmInterface"),"Error while port "<<port_);
            }
            
        }
    }

    CallbackReturn ArmInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
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
            RCLCPP_FATAL(rclcpp::get_logger("ArmInterface"),"Port Not found. Exiting");
            return CallbackReturn::FAILURE;
        }

        position_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        prev_position_commands_.reserve(info_.joints.size());

        return CallbackReturn::SUCCESS;
        
    }

    std::vector<hardware_interface::StateInterface> ArmInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i=0;i<info_.joints.size();i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArmInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i=0;i<info_.joints.size();i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        }
        return command_interfaces;
    }

    CallbackReturn ArmInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArmInterface"),"Initializing the Robot Hardware.......");
        position_commands_ = {0.0,0.0,0.0,0.0,0.0,0.0};
        prev_position_commands_ = {0.0,0.0,0.0,0.0,0.0,0.0};
        position_states_ = {0.0,0.0,0.0,0.0,0.0,0.0};
        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArmInterface"),"Port Initialization failed while trying to open port" << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("ArmInterface"),"Robot Hardware Successfully Started");
        return CallbackReturn::SUCCESS;


    }
    CallbackReturn ArmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArmInterface"),"Stopping the robot hardware...");
        if(arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArmInterface"),"Error Occured while trying to close port" << port_);
                return CallbackReturn::FAILURE;
            }
            
        }
        RCLCPP_INFO(rclcpp::get_logger("ArmInterface"),"Robot Hardware Successfully Stopped");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ArmInterface::read(const rclcpp::Time &time, const rclcpp::Duration & period)
    {
        position_states_= position_commands_; // since my motors have no feedback
        return hardware_interface::return_type::OK;
    }



    hardware_interface::return_type ArmInterface::write(const rclcpp::Time &time, const rclcpp::Duration & period)
    {
        if(position_commands_ == prev_position_commands_)
        {
            return hardware_interface::return_type::OK;
        }

        std::string msg;
        int waist = static_cast<int>((position_commands_.at(0))* 180) / M_PI;
        msg.append("b");
        msg.append(std::to_string(waist));
        msg.append(",");

        int shoulder = static_cast<int>((position_commands_.at(1))* 180) / M_PI;
        msg.append("s");
        msg.append(std::to_string(shoulder));
        msg.append(",");

        int elbow = static_cast<int>((position_commands_.at(2))* 180) / M_PI;
        msg.append("e");
        msg.append(std::to_string(elbow));
        msg.append(",");

        int wrist = static_cast<int>((position_commands_.at(3))* 180) / M_PI;
        msg.append("w");
        msg.append(std::to_string(wrist));
        msg.append(",");

        int wrist_twist = static_cast<int>((position_commands_.at(4))* 180) / M_PI;
        msg.append("t");
        msg.append(std::to_string(wrist_twist));
        msg.append(",");

        int tool_tip = static_cast<int>((position_commands_.at(5))* 180) / M_PI;
        msg.append("g");
        msg.append(std::to_string(tool_tip));
        msg.append(",");

        try
        {
            arduino_.Write(msg);
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArmInterface"),"Could not send message to arduino" << msg);
            return hardware_interface::return_type::ERROR;
        }
        prev_position_commands_ = position_commands_;
        return hardware_interface::return_type::OK;
    }

}

PLUGINLIB_EXPORT_CLASS(arm_controller::ArmInterface, hardware_interface::SystemInterface);