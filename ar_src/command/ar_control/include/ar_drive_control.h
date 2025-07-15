// Drive control class to handle both single joint and multi joint drives
// Based on old EthercatJointControlInterface class, but remodelled for drive and single/multi joint control
// Udupa; 04Dec'20
// Updated to accommodate PMas/Soem master based comm; Udupa; Apr'23

#pragma once

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <ar_control/ar_drive_config.h>
// #include <ar_control/ar_joint_control.h>
#include <ar_utils/ar_logging.h>

namespace ar_control
{
	class ArDriveControl
	{
	public:
		ArDriveControl(DriveParameter& driveParm, bool hasUi = false);
		~ArDriveControl();

		void AddJoint(int torque_for_emergency_stop, int over_load_level, int over_speed_level, double motor_working_range,
					  int max_motor_speed, int max_torque, JointParameter& jointParm);

		void InitializeDrive();
		void read();
		void write();
		void shutdown();


		std::vector<std::string> getJointNames();
		void getJointCmdToPulses(const std::string& jointName, double& jointPos, double& jointVel);
		void setJointPulsesToPos(TomoJointControl* joint, int position, int velocity = 0);


		int getInputActualValueToStatus(std::vector<std::string>& joint_names, std::vector<std::string>& hardware_ids,
										std::vector<uint32>& position_actual_values, std::vector<uint32>& velocity_actual_values,
										std::vector<uint16>& torque_actual_values);



		inline int jointCount()
		{
			return drive_parm.joint_parameters.size();
		}


		inline std::vector<TomoJointControl*> getJoints()
		{
			return joints;
		}

		inline int getDriveId(){
			return drive_id_;
		}

		inline int getSlaveId()
		{
			return slave_id_;
		}

		std::vector<TomoJointControl*> joints;


	protected:
		DriveParameter& drive_parm;
		bool single_joint_drive;

		tomo_master::DriverInfo driver_info;
		int slave_id_;
		int drive_id_;
		bool homing_done;

		bool has_ui_;

		std::vector<std::ofstream*> log_files;
		int log_cnt;
		int file_cnt;

		uint32 out_cnt;
		uint8 rewrite_count;

		int readLoopPrintCounter;
		int writeLoopPrintCounter;

		void logJointReadData();
		void logJointWriteData();

		double getJointCmd(TomoJointControl* joint, double cmdPos);
		template <typename T>
		void jointCmdToPulses(TomoJointControl* joint, T* position = nullptr, T* velocity = nullptr, T* accel = nullptr,
							  T* decel = nullptr);

		double getJointPos(TomoJointControl* joint, double jointPos);
		template <typename T> void jointPulsesToPos(TomoJointControl* joint, T* position = nullptr, T* velocity = nullptr);

		bool stop_write_dkk_drive_    = false;
		bool log_joint_data_		  = false;

		std::string log_data_path_ = "/tomo_stats/log/";
	};
}  // namespace ar_control