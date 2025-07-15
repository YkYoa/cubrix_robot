// Modular hardware interface; Redesigned with drive and joint based control
// This class only handles drives while the TomoDriveControl class takes care of joints
// Not supposed to define any hard coded joint or comm relationships
// Everything is dictated by tomo_drive_config.h
// Udupa 20Nov'20 to 04Dec'20
// Updated to accommodate PMas/Soem master based comm; Udupa; Apr'23

#pragma once


#include "ar_control/ar_control_server.h"
#include "yaml-cpp/yaml.h"
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>


#include <moveit/move_group_interface/move_group_interface.h>

#include <ar_utils/ar_utils.h>

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

		bool is_simulation;
		bool is_ui;

	private:
		ArDrives ar_drives;
		urdf::Model urdf_model;
		std::thread* control_server_thread;
		void readConfigFromYaml(std::string yaml_path);
		std::shared_ptr<ArControlServer> ar_control_server;
		void launchControlServer();

		// For config Protocol Manager
		ar_master::Protocol* protocol_manager_;
		std::map<int, std::vector<int>> port_to_slave_id_;
		std::map<int, int> drive_id_to_slave_id_;
		std::map<int, int> slave_id_to_drive_id_;
	};
}  // namespace tomo_control
