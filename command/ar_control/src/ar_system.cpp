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

#include "tomo_control/tomo_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

double now()
{
	struct timespec n;
	clock_gettime(CLOCK_MONOTONIC, &n);
	return static_cast<double>(n.tv_nsec) / 1e+9 + n.tv_sec;
}

double last_timestamp = 0;

namespace ar_control
{
	CallbackReturn ArSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
	{
		if(hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
			return CallbackReturn::ERROR;
		}

		robot_desc_ = info_.hardware_parameters["desc"];
		is_simulation  = info_.hardware_parameters["sim"] == "False" ? false : true;
		is_ui  = info_.hardware_parameters["ui"] == "False" ? false : true;

		// hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		// hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

		comm_mutex.push_back(std::make_shared<boost::mutex>());
		comm_mutex.push_back(std::make_shared<boost::mutex>());
		comm_mutex.push_back(std::make_shared<boost::mutex>());
		g_quit		= false;
		is_parallel = false;

		RCLCPP_WARN(rclcpp::get_logger("ArSystemHardware"), "Init robot: %s, sim: %s (is_simulation=%d), ui: %s (is_ui=%d)", robot_desc_.c_str(),
				info_.hardware_parameters["sim"].c_str(), is_simulation, info_.hardware_parameters["ui"].c_str(), is_ui);

		robot = std::make_shared<ArHardwareInterface>(comm_mutex, cond, lock, robot_desc_, g_quit, is_simulation, is_ui, is_parallel);

		for(const hardware_interface::ComponentInfo& joint : info_.joints) {
			if(joint.command_interfaces.size() != 1) {
				RCLCPP_FATAL(rclcpp::get_logger("ArSystemHardware"), "Joint '%s' has %zu command interfaces found. 1 expected.",
							 joint.name.c_str(), joint.command_interfaces.size());
				return CallbackReturn::ERROR;
			}

			if(joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
				RCLCPP_FATAL(rclcpp::get_logger("ArSystemHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
							 joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
				return CallbackReturn::ERROR;
			}

			if(joint.state_interfaces.size() != 1) {
				RCLCPP_FATAL(rclcpp::get_logger("ArSystemHardware"), COLOR_CYAN "Joint '%s' has %zu state interface. 1 expected.",
							 joint.name.c_str(), joint.state_interfaces.size());
				return CallbackReturn::ERROR;
			}

			if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
				RCLCPP_FATAL(rclcpp::get_logger("ArSystemHardware"), "Joint '%s' have %s state interface. '%s' expected.",
							 joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
				return CallbackReturn::ERROR;
			}
		}

		return CallbackReturn::SUCCESS;
	}

	CallbackReturn ArSystemHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
	{
		RCLCPP_INFO(rclcpp::get_logger("ArSystemHardware"), "Configuring...Please wait");

		for(auto drive : robot->getDrives()) {
			for(auto joint : drive.second->getJoints()) {
				joint->joint_cmd = joint->joint_pos;
			}
		}

		RCLCPP_INFO(rclcpp::get_logger("ArSystemHardware"), "Successfully configured!");

		return CallbackReturn::SUCCESS;
	}

	std::vector<hardware_interface::StateInterface> ArSystemHardware::export_state_interfaces()
	{
		std::vector<hardware_interface::StateInterface> state_interfaces;

		for(auto drive : robot->getDrives()) {
			for(auto joint : drive.second->getJoints()) {
				// RCLCPP_INFO(rclcpp::get_logger("ArSystemHardware"), "export_state_interfaces %s", joint->joint_name.c_str());
				state_interfaces.emplace_back(
					hardware_interface::StateInterface(joint->joint_name, hardware_interface::HW_IF_POSITION, &joint->joint_pos));
				// state_interfaces.emplace_back(
				// 	hardware_interface::StateInterface(joint->joint_name, hardware_interface::HW_IF_VELOCITY, &joint->joint_vel));
			}
		}

		return state_interfaces;
	}

	std::vector<hardware_interface::CommandInterface> ArSystemHardware::export_command_interfaces()
	{
		std::vector<hardware_interface::CommandInterface> command_interfaces;

		for(auto drive : robot->getDrives()) {
			for(auto joint : drive.second->getJoints()) {
				// RCLCPP_INFO(rclcpp::get_logger("ArSystemHardware"), "export_command_interfaces %s", joint->joint_name.c_str());
				command_interfaces.emplace_back(
					hardware_interface::CommandInterface(joint->joint_name, hardware_interface::HW_IF_POSITION, &joint->joint_cmd));
				// command_interfaces.emplace_back(
				// 	hardware_interface::CommandInterface(joint->joint_name, hardware_interface::HW_IF_VELOCITY, &joint->joint_vel_cmd));
			}
		}

		return command_interfaces;
	}

	CallbackReturn ArSystemHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
	{
		RCLCPP_INFO(rclcpp::get_logger("ArSystemHardware"), "Activating....Please wait");

		for(auto drive : robot->getDrives()) {
			for(auto joint : drive.second->getJoints()) {
				joint->joint_cmd = joint->joint_pos;
			}
		}

		RCLCPP_INFO(rclcpp::get_logger("ArSystemHardware"), "Successfully activated!");

		last_timestamp = now();

		return CallbackReturn::SUCCESS;
	}

	CallbackReturn ArSystemHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
	{
		RCLCPP_INFO(rclcpp::get_logger("ArSystemHardware"), "Deactivating...Please wait");

		RCLCPP_INFO(rclcpp::get_logger("ArSystemHardware"), "Successfully deactivated!");
		return CallbackReturn::SUCCESS;
	}

	bool first = true;
	hardware_interface::return_type ArSystemHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
	{
		// boost::mutex::scoped_lock lock(control_mutex);
		(void) time;
		(void) period;
		fflush(stdout);
		robot->read();
		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type ArSystemHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
	{
        // Fix this to some faster protocol
        (void) time;
		(void) period;
		fflush(stdout);
		robot->write();

		/// Control loop waits in this write function - to sync with the comm loop
		/// Udupa; 12Jan'23
		if(robot->soem_drives)
			pthread_cond_wait(&cond, &lock);

		return hardware_interface::return_type::OK;
	}

	ArSystemHardware::~ArSystemHardware()
	{
		robot->shutdown();
		usleep(1000);

		pthread_cond_destroy(&cond);
		pthread_mutex_destroy(&lock);

		for(auto& mutex : comm_mutex) {
			mutex.reset();	// reset shared_ptr to release ownership
		}
	}

}  // namespace ar_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ar_control::ArSystemHardware, hardware_interface::SystemInterface)
