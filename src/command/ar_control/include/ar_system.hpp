// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"
#include "ar_hardware_interface.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ar_control
{
    class ArSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ArSystemHardware);

        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
		hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

		ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
		hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
		~ArSystemHardware() override;

        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        void shutdown();

	private:
        // Parameters for the Ar robot
        std::string robot_desc; ///< Description of the robot
        bool is_ui;             ///< Flag to indicate if the UI is enabled
		bool is_simulation;     ///< Flag to indicate if the hardware is simulated
        bool b_quit;           ///< Flag to indicate if the system should quit
        std::vector<std::shared_ptr<boost::mutex>> comm_mutex; ///< Mutexes for communication
        boost::mutex control_mutex; ///< Mutex for control operations
        pthread_cond_t cond = PTHREAD_COND_INITIALIZER; ///< Condition variable for synchronization
        pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER; ///< Mutex for the condition variable

        std::shared_ptr<ArHardwareInterface> robot; ///< Shared pointer to the Ar hardware interface

        // Last timestamp for the read operation
        double last_timestamp; ///< Last timestamp for the read operation


    };

}  // namespace ar_control
