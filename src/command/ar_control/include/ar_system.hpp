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
    /**
     * @brief ROS2 Control hardware interface implementation
     * 
     * Implements the hardware_interface::SystemInterface for ROS2 Control.
     * Manages the lifecycle of the hardware interface and provides read/write
     * operations for joint state and command interfaces.
     */
    class ArSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ArSystemHardware);

        /**
         * @brief Initialize the hardware interface
         * @param info Hardware information from URDF
         * @return CallbackReturn indicating success or failure
         */
        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        /**
         * @brief Configure the hardware interface
         * @param previous_state Previous lifecycle state
         * @return CallbackReturn indicating success or failure
         */
        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Export state interfaces (joint positions, velocities)
         * @return Vector of state interfaces
         */
        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /**
         * @brief Export command interfaces (joint position/velocity commands)
         * @return Vector of command interfaces
         */
        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /**
         * @brief Activate the hardware interface
         * @param previous_state Previous lifecycle state
         * @return CallbackReturn indicating success or failure
         */
        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Deactivate the hardware interface
         * @param previous_state Previous lifecycle state
         * @return CallbackReturn indicating success or failure
         */
        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Read current joint states from hardware
         * @param time Current time
         * @param period Time period since last read
         * @return Return type indicating success or failure
         */
        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /**
         * @brief Write joint commands to hardware
         * @param time Current time
         * @param period Time period since last write
         * @return Return type indicating success or failure
         */
        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        
        /**
         * @brief Destructor
         */
        ~ArSystemHardware() override;

        /**
         * @brief Shutdown the hardware interface
         */
        ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
        void shutdown();

    private:
        // Parameters for the Ar robot
        std::string robot_desc;                                ///< Description of the robot
        bool is_ui;                                            ///< Flag to indicate if the UI is enabled
        bool is_simulation;                                    ///< Flag to indicate if the hardware is simulated
        bool is_ecat;                                          ///< Flag to indicate if Igh EtherCAT Master is used
        bool b_quit;                                           ///< Flag to indicate if the system should quit
        std::vector<std::shared_ptr<boost::mutex>> comm_mutex; ///< Mutexes for communication
        boost::mutex control_mutex;                            ///< Mutex for control operations
        pthread_cond_t cond = PTHREAD_COND_INITIALIZER;        ///< Condition variable for synchronization
        pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;      ///< Mutex for the condition variable

        std::shared_ptr<ArHardwareInterface> robot; ///< Shared pointer to the Ar hardware interface

        // Last timestamp for the read operation
        double last_timestamp; ///< Last timestamp for the read operation
    };

} // namespace ar_control
