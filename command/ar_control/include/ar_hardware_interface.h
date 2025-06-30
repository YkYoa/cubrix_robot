// Modular hardware interface; Redesigned with drive and joint based control
// This class only handles drives while the TomoDriveControl class takes care of joints
// Not supposed to define any hard coded joint or comm relationships
// Everything is dictated by tomo_drive_config.h
// Udupa 20Nov'20 to 04Dec'20
// Updated to accommodate PMas/Soem master based comm; Udupa; Apr'23

#pragma once

#include <ar_control/ar_drive_control.h>

#include "ar_control/ar_control_server.h"
#include "yaml-cpp/yaml.h"
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include <moveit/dynamics_solver/dynamics_solver.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <ar_comm/topic_string.h>

#include <tomo_utils/tomo_utils.h>

namespace ar_control
{
	class ArHardwareInterface
	{
	public:
		ArHardwareInterface(std::string robotDesc, bool isSimulation = false, bool isUi = false);
		~ArHardwareInterface();

		void shutdown();
		void read();
		void write();

		/*
			This func can set on off from ui
		*/
		tVectorB getDrivesState(const tVectorS jointNames);


		int getInputActualValueToStatus(std::vector<std::string>& joint_names, std::vector<std::string>& hardware_ids,
										std::vector<uint32>& position_actual_values, std::vector<uint32>& velocity_actual_values,
										std::vector<uint16>& torque_actual_values);
		std::map<int, TomoDriveControl*> getDrives();

		std::string open_servo;
		bool is_simulation;
		bool is_ui;

	private:
		TomoDrives tomo_drives;
		urdf::Model urdf_model;
		std::map<int, TomoDriveControl*> drives;
		std::thread* control_server_thread;
		void readDriveConfigFromYaml(std::string yaml_path);
		void InitializeDrives(std::vector<TomoDriveControl*>& drives);
		std::shared_ptr<ArControlServer> ar_control_server;
		void launchControlServer();

		// For config Ethercat Manager
		tomo_master::EtherCatManager* port1Manager;
		std::map<int, std::vector<int>> port_to_slave_id_;
		std::map<int, int> drive_id_to_slave_id_;
		std::map<int, int> slave_id_to_drive_id_;

		// For Pmas
		std::shared_ptr<PmasClient> pmas_client;
		std::vector<double> pmas_joint_states;

		// For get Torques to offset with torque_gama
		std::map<std::string, double*> joint_cmd_for_get_torque_;
		std::map<std::string, tVectorS> group_to_joint_for_get_torque_;
		std::map<std::string, double> torque_cmd_;
		bool use_torque_offset_ = false;
		bool stop_flag_			= false;
		std::map<std::string, dynamics_solver::DynamicsSolverPtr> dynamics_solver_;
		std::shared_ptr<rclcpp::Node> tomo_hardware_interface_node_;
		std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
		moveit::core::RobotModelPtr robot_model_;
		void getTorques(std::map<std::string, double>& torques, std::map<std::string, double*> pos);

		// For calculate Fingers Kinematic
		std::map<std::string, double*> finger_joint_cmd_;
		std::map<std::string, double*> finger_joint_pos_;
		std::map<std::string, tomo_control::ThreeDofFinger> three_dof_finger_solver_;
		bool has_three_dof_finger_ = false;
		void updateThreeDofFingerJointPos();
		void getThreeDofFingerJointCmd();
		std::map<std::string, std::vector<std::string>> three_dof_finger_group_to_joint_;
		bool has_two_dof_finger = false;
		std::map<std::string, std::string> two_dof_finger_child_and_parent_;
		void getJointPosForChild();

		bool& b_quit_;
		bool sync_flag_on;

		int driveId2Int(std::string driveID);
		static inline bool groupFingerComparator(const std::string& joint1, const std::string& joint2)
		{
			return joint1.back() < joint2.back();
		}
		// send error to UI
		std::map<std::string, int> single_joint_name_to_drive_id_;
	};
}  // namespace tomo_control
