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
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <ar_utils.h>
#include <vector>
#include <ar_drive_config.h>
#include <ar_drive_control.h>
#ifndef NO_ETHERCAT
#include <igh_manager.hpp>
#endif

namespace ar_control
{
	/**
	 * @brief Hardware interface for robot control
	 * 
	 * Modular hardware interface that handles drives while ArDriveControl manages joints.
	 * Everything is configured via ar_drive_config.h. Supports both simulation and
	 * real hardware with EtherCAT communication.
	 */
	class ArHardwareInterface
	{
	public:
		/**
		 * @brief Constructor
		 * @param comm_mutex Vector of mutexes for communication threads
		 * @param cond Condition variable for thread synchronization
		 * @param cond_lock Mutex for condition variable
		 * @param robotDesc Robot description (URDF path or name)
		 * @param bQuit Reference to quit flag
		 * @param isSimulation Enable simulation mode (default: false)
		 * @param isUi Enable UI mode (default: false)
		 * @param isEcat Enable EtherCAT communication (default: true)
		 */
		ArHardwareInterface(std::vector<std::shared_ptr<boost::mutex>> comm_mutex, pthread_cond_t &cond, pthread_mutex_t &cond_lock,
							std::string robotDesc, bool &bQuit, bool isSimulation = false, bool isUi = false, bool isEcat = true);
		
		/**
		 * @brief Destructor
		 */
		~ArHardwareInterface();

		/**
		 * @brief Shutdown the hardware interface
		 * 
		 * Stops all drives, disables motors, and cleans up resources.
		 */
		void shutdown();
		
		/**
		 * @brief Read current joint states from hardware
		 * 
		 * Reads encoder values and converts them to joint positions/velocities.
		 */
		void read();
		
		/**
		 * @brief Write joint commands to hardware
		 * 
		 * Converts joint commands to drive pulses and sends them to the hardware.
		 */
		void write();

		/**
		 * @brief Get all drive controllers
		 * @return Map of drive ID to ArDriveControl pointers
		 */
		std::map<int, ArDriveControl *> getDrives();

		bool is_simulation; ///< Flag to indicate if the hardware is simulated
		bool is_ui;			///< Flag to indicate if the UI is enabled
		bool is_ecat;		///< Flag to indicate if Igh EtherCAT Master is used
		bool &b_quit_;		///< Reference to the quit flag

		std::vector<int> soem_drives; ///< SOEM slave IDs detected (1-based indices)

	private:
		ArDrives ar_drives;						///< Holds all drive parameters and joint names
		urdf::Model urdf_model;					///< URDF model of the robot
		std::map<int, ArDriveControl *> drives; ///< Map of drive ID to ArDriveControl objects

		std::thread *control_server_thread;					///< Thread for the control server
		std::shared_ptr<ArControlServer> ar_control_server; ///< Control server instance

		std::shared_ptr<rclcpp::Node> ar_hardware_interface_node_; ///< Node for the hardware interface

		/**
		 * @brief Initialize all drive controllers
		 * @param drives Vector of drive control pointers to initialize
		 */
		void InitializeDrives(std::vector<ArDriveControl *> &drives);
		
		/**
		 * @brief Read drive configuration from YAML file
		 * 
		 * Loads drive parameters, joint mappings, and communication settings.
		 */
		void readConfigFromYaml();
		
		/**
		 * @brief Wait for UI parameters to be received
		 * 
		 * Blocks until UI sends parameter configuration via ROS2 topic.
		 */
		void waitForUiParams();
		
		/**
		 * @brief Launch the control server in a separate thread
		 * 
		 * Starts the ArControlServer to handle action requests.
		 */
		void launchControlServer();

		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr param_sub_; ///< Subscriber for UI params
		std::string ui_params_yaml_;  ///< YAML string from UI
		bool ui_params_received_ = false;  ///< Flag if UI params received

		int no_of_port_connected_ = 0; ///< Number of connected ports

		master::EthercatManager *portManager; ///< Port manager for handling communication ports (SOEM)
	#ifndef NO_ETHERCAT
	master::IghManager *ighManager;		  ///< IGH EtherCAT manager pointer
#endif
	};
} // namespace ar_control
