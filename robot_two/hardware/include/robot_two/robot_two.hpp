#ifndef ROBOT_TWO_HPP
#define ROBOT_TWO_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robot_two/visibility_control.h"
#include "robot_two/robot_two_arduino_comms.hpp"

namespace robot_two
{
    class RobotTwoHardware : public hardware_interface::SystemInterface
    {

        class Config
        {
            std::string wheel_1 = "";
            std::string wheel_2 = "";
            std::string wheel_3 = "";
            std::string wheel_4 = "";
            float loop_rate = 0.0;
            std::string device = "";
            int baud_rate = 0;
            int timeout_ms = 0;
            int enc_counts_per_rev = 0;
        }
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(RobotTwoHardware);

            ROBOT_TWO_PUBLIC
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo & info) override;

            ROBOT_TWO_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            ROBOT_TWO_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            ROBOT_TWO_PUBLIC
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            ROBOT_TWO_PUBLIC
            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;

            ROBOT_TWO_PUBLIC
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

            ROBOT_TWO_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            ROBOT_TWO_PUBLIC
            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

            ROBOT_TWO_PUBLIC
            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            robot_two_ArduinoComms comms_;
            Config cfg_;

    }
}

#endif