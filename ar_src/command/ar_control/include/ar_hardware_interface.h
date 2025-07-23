// Modular hardware interface; Redesigned with drive and joint based control
// This class only handles drives while the TomoDriveControl class takes care of joints
// Not supposed to define any hard coded joint or comm relationships
// Everything is dictated by tomo_drive_config.h
// Udupa 20Nov'20 to 04Dec'20
// Updated to accommodate PMas/Soem master based comm; Udupa; Apr'23

#pragma once


#include <ar_control_server.h>
#include "yaml-cpp/yaml.h"
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>


#include <moveit/move_group_interface/move_group_interface.h>

#include <ar_utils.h>
#include <ar_drive_config.h>
#include <ar_drive_control.h>

namespace ar_control
{
	class ArHardwareInterface
	{
	public:
		ArHardwareInterface(bool isSimulation = false, bool isUi = false);
		~ArHardwareInterface();

		void shutdown();
		void read();
		void write();

		std::map<int, ArDriveControl*> getDrives();

		bool is_simulation;		///< Flag to indicate if the hardware is simulated
		bool is_ui;				///< Flag to indicate if the UI is enabled

	private:
		ArDrives ar_drives;                              		///< Holds all drive parameters and joint names
		urdf::Model urdf_model;									///< URDF model of the robot
		std::map<int, ArDriveControl*> drives;					///< Map of drive ID to ArDriveControl objects

		std::thread* control_server_thread;              		///< Thread for the control server
		std::shared_ptr<ArControlServer> ar_control_server;     ///< Control server instance

		std::shared_ptr<rclcpp::Node> ar_hardware_interface_node_;  ///< Node for the hardware interface



		void readConfigFromYaml(std::string yaml_path);
		void launchControlServer();

		// ar_master::Protocol* protocol_manager_; (add later)
		std::map<int, std::vector<int>> port_to_slave_id_;      ///< Map of port IDs to slave IDs
		std::map<int, int> drive_id_to_slave_id_;  	 			///< Map of drive IDs to slave IDs	
		std::map<int, int> slave_id_to_drive_id_;				///< Map of slave IDs to drive IDs
	};
}  // namespace tomo_control
