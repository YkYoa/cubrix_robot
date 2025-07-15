// Joint control class to manage one joint within a drive
// Based on old JointControlInterface class; restructured for modular operation with DriveControl class
// Udupa; 04Dec'20

#pragma once

#include <memory>
#include <string>
#include <vector>

namespace ar_control
{
	class ArJointControl
	{
	public:
		ArJointControl(std::string jointName);
		~ArJointControl(){};

		void getInputActualValueToStatus(std::string& joint_name, std::string& hardware_id, uint32_t& position_actual_value,
										 uint32_t& velocity_actual_value, uint16_t& torque_actual_value);

	public:
		///////////////////////////// Joint Data
		std::string joint_name;
		std::string hardware_id;
		std::string driver_brand;
		double joint_cmd;
		double joint_pos;
		double joint_vel;
		double joint_vel_cmd;
		double joint_eff;
		double joint_torque;
		double joint_cmd_val;

		int home_encoder_offset;
		double torque_gama;

		double upper_limit;
		double lower_limit;
		bool force_limit;

		double pulse_per_revolute;

		bool has_parent;
		int rev_angle_convert_mode;
		int client_id; //for port in slave ethercat

		/////////////////////////////

		friend class ArDriveControl;
	};
}  // namespace ar_control