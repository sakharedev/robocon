#include "hardware/include/robot_two/robot_two.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_two
{
    hardware_interface::CallBackReturn RobotTwoHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if(
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallBackReturn::SUCCES)
        {
            return hardware_interface::CallBackReturn::ERROR;
        }

        
        cfg_.wheel_1 = info_.hardware_parameters["wheel_1"];
        cfg_.wheel_2 = info_.hardware_parameters["wheel_2"];
        cfg_.wheel_3 = info_.hardware_parameters["wheel_3"];
        cfg_.wheel_4 = info_.hardware_parameters["wheel_4"];

        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["device"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        if (info_.hardware_parameters.count("pid_p") > 0)
        {
            cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
            cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
            cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
            cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("RobotTwoHardware"), "PID values not supplied, using defaults.");
        }

        wheel_1.setup(cfg_.wheel_1, cfg_.enc_counts_per_rev);
        wheel_2.setup(cfg_.wheel_2, cfg_.enc_counts_per_rev);
        wheel_3.setup(cfg_.wheel_3, cfg_.enc_counts_per_rev);
        wheel_4.setup(cfg_.wheel_4, cfg_.enc_counts_per_rev);

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RobotTwoHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RobotTwoHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RobotTwoHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RobotTwoHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RobotTwoHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;

    }

    std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_1.name, hardware_interface::HW_IF_POSITION, &wheel_1.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_1.name, hardware_interface::HW_IF_VELOCITY, &wheel_1.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_2.name, hardware_interface::HW_IF_POSITION, &wheel_2.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_2.name, hardware_interface::HW_IF_VELOCITY, &wheel_2.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_3.name, hardware_interface::HW_IF_POSITION, &wheel_3.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_3.name, hardware_interface::HW_IF_VELOCITY, &wheel_3.vel));
            
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_4.name, hardware_interface::HW_IF_POSITION, &wheel_4.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_4.name, hardware_interface::HW_IF_VELOCITY, &wheel_4.vel));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_1.name, hardware_interface::HW_IF_VELOCITY, &whee1.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_2.name, hardware_interface::HW_IF_VELOCITY, &wheel_2.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_3.name, hardware_interface::HW_IF_VELOCITY, &wheel_3.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_4.name, hardware_interface::HW_IF_VELOCITY, &wheel_4.cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotTwoHardware"), "Configuring ...please wait...");
        if (comms_.connected())
        {
            comms_.disconnect();
        }
        comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
        RCLCPP_INFO(rclcpp::get_logger("RobotTwoHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotTwoHardware"), "Cleaning up ...please wait...");
        if (comms_.connected())
        {
            comms_.disconnect();
        }
        RCLCPP_INFO(rclcpp::get_logger("RobotTwoHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotTwoHardware"), "Activating ...please wait...");
        if (!comms_.connected())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (cfg_.pid_p > 0)
        {
            comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
        }
        RCLCPP_INFO(rclcpp::get_logger("RobotTwoHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotTwoHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("RobotTwoHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    


}