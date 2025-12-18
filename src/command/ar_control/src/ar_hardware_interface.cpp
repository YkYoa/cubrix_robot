#include "ar_hardware_interface.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ar_common/common.h>

namespace ar_control
{
    void ArHardwareInterface::launchControlServer()
    {
        RCLCPP_INFO(rclcpp::get_logger("Ar Robot"), "Launching control server...");
        ar_control_server = std::make_shared<ArControlServer>(this);
        ar_control_server->startActionServer();
        rclcpp::spin(ar_control_server);
    }

    void ArHardwareInterface::InitializeDrives(std::vector<ArDriveControl*>& drives)
    {
        for(auto drive : drives){
            drive->InitializeDrive();
        }
    }

    ArHardwareInterface::ArHardwareInterface(std::vector<std::shared_ptr<boost::mutex>> comm_mutex, pthread_cond_t& cond, 
        pthread_mutex_t& cond_lock, std::string robotDesc, bool& bQuit, bool isSimulation, bool isUI, bool isEcat) : b_quit_(bQuit)
    {
        // Initialize the hardware interface
        control_server_thread = nullptr;
        is_simulation = isSimulation;
        is_ui = isUI;
        is_ecat = isEcat;

        std::string robot_description;
        control_server_thread = new std::thread(&ArHardwareInterface::launchControlServer, this);

        ar_hardware_interface_node_ = rclcpp::Node::make_shared("ar_hardware_interface_node",
                            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

        if (!ar_hardware_interface_node_->get_parameter("robot_description", robot_description))
        {
            RCLCPP_ERROR(ar_hardware_interface_node_->get_logger(), "Parameter 'robot_description' not found.");
        }

        if (!urdf_model.initString(robot_description))
        {
            RCLCPP_ERROR(ar_hardware_interface_node_->get_logger(), "Failed to parse URDF model.");
        }

        readConfigFromYaml();

        RCLCPP_INFO(rclcpp::get_logger("Ar"), "[Ar Hardware Interface] Initializing ETHERCAT manager, drives and joints");
        portManager = nullptr;
        ighManager = nullptr;

        if(!is_simulation && !is_ecat) {
            soem_drives.clear();
            for (const auto &driveParm : ar_drives.drive_parameters)
            {
                if (driveParm.port_id == PORT_SOEM)
                {
                    soem_drives.push_back(driveParm.slave_id);
                }
            }

            portManager = new master::EthercatManager((uint8_t) PORT_SOEM, robotDesc, cond, cond_lock, *comm_mutex[PORT_SOEM]);
            if (!portManager->initialize(b_quit_, soem_drives))
            {
                b_quit_ = true;
                RCLCPP_ERROR(rclcpp::get_logger("Ar"), "Failed to initialize SOEM ETHERCAT manager. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("Ar"), "SOEM ETHERCAT manager initialized for %zu SOEM drives", soem_drives.size());
        }
        else if(!is_simulation && is_ecat) {
            RCLCPP_INFO(rclcpp::get_logger("Ar"), "Initializing IGH EtherCAT master...");
            
            ighManager = new master::IghManager(cond, cond_lock, *comm_mutex[PORT_SOEM]);
            
            if(ighManager->openEthercatMaster() != 0) {
                b_quit_ = true;
                RCLCPP_ERROR(rclcpp::get_logger("Ar"), "Failed to open IGH EtherCAT master. Exiting.");
                delete ighManager;
                ighManager = nullptr;
                return;
            }
            
            ighManager->getAllSlavesInfo();
            int num_slaves = ighManager->getNumbOfConnectedSlaves();
            RCLCPP_INFO(rclcpp::get_logger("Ar"), "IGH master configured, detected %d slaves", num_slaves);
            
            if(ighManager->configSlaves() != 0) {
                b_quit_ = true;
                RCLCPP_ERROR(rclcpp::get_logger("Ar"), "Failed to configure IGH slaves. Exiting.");
                delete ighManager;
                ighManager = nullptr;
                return;
            }

            if(ighManager->setCyclicPositionParameters() != 0) {
                RCLCPP_WARN(rclcpp::get_logger("Ar"), "Failed to set cyclic position parameters via SDO");
            }

            
            for(int i = 0; i < num_slaves; i++) {
                if(ighManager->mapDefaultPDOs(ighManager->slave_[i], i) != 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("Ar"), "Failed to map PDOs for slave %d", i);
                }
            }

            ighManager->configDcSyncDefault();
            
            if(ighManager->activateMaster() != 0) {
                b_quit_ = true;
                RCLCPP_ERROR(rclcpp::get_logger("Ar"), "Failed to activate IGH master. Exiting.");
                delete ighManager;
                ighManager = nullptr;
                return;
            }
            
            if(ighManager->registerDomain() != 0) {
                b_quit_ = true;
                RCLCPP_ERROR(rclcpp::get_logger("Ar"), "Failed to register IGH domain. Exiting.");
                delete ighManager;
                ighManager = nullptr;
                return;
            }
            
            if(ighManager->startCyclicCommunication() != 0) {
                b_quit_ = true;
                RCLCPP_ERROR(rclcpp::get_logger("Ar"), "Failed to start IGH cyclic communication. Exiting.");
                delete ighManager;
                ighManager = nullptr;
                return;
            }
            
            if(ighManager->waitForOpMode() != 0) {
                RCLCPP_WARN(rclcpp::get_logger("Ar"), "Some slaves did not reach OPERATIONAL state in time");
            }
            
            RCLCPP_INFO(rclcpp::get_logger("Ar"), "IGH EtherCAT master initialized successfully");
        }

        for(auto& driveParm : ar_drives.drive_parameters){
            drives[driveParm.drive_id] = new ArDriveControl(driveParm, is_ui);

            if(!is_simulation && driveParm.port_id == PORT_SOEM && portManager) {
                RCLCPP_INFO(rclcpp::get_logger("ArHardwareInterface"), 
                    "Initializing drive client for drive %d with slave_id %d", 
                    driveParm.drive_id, driveParm.slave_id);
                drives[driveParm.drive_id]->InitializeDriveClient(portManager, driveParm.slave_id);
            } else if(!is_simulation && is_ecat && ighManager && driveParm.port_id == PORT_SOEM) {
                drives[driveParm.drive_id]->InitializeDriveClient(ighManager, driveParm.slave_id);
            }

            for(auto& jointParm : driveParm.joint_paramters){
                auto urdf_joint = urdf_model.getJoint(jointParm.joint_name);
                drives[driveParm.drive_id]->AddJoint(jointParm);
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("Ar"), "Initializing drive state machines...");
        std::vector<ArDriveControl*> drives_ptr;

        for(auto& drive : drives){
            if(drive.second)
                drives_ptr.push_back(drive.second);
        }
        
        InitializeDrives(drives_ptr);
    }

    ArHardwareInterface::~ArHardwareInterface()
    {
        shutdown();
    }

    void ArHardwareInterface::shutdown()
    {
        for (auto drive : drives)
        {
            if (drive.second)
            {
                delete drive.second;
            }
        }
		if(portManager) {
			RCLCPP_INFO(rclcpp::get_logger("Ar"), "Shutting down SOEM manager");
			delete portManager;
			portManager = nullptr;
		}
		if(ighManager) {
			RCLCPP_INFO(rclcpp::get_logger("Ar"), "Shutting down IGH manager");
			ighManager->shutdown();
			delete ighManager;
			ighManager = nullptr;
		}
		if(control_server_thread)
			control_server_thread->join();

        RCLCPP_INFO(rclcpp::get_logger("\nArHardwareInterface"), "Shut down complete.");
    }

    void ArHardwareInterface::readConfigFromYaml()
    {
        std::string config_path = ar_common::getConfigPath();
        RCLCPP_INFO(rclcpp::get_logger("ArHardwareInterface"), COLOR_DARKYELLOW "Drive config path: %s" COLOR_RESET, config_path.c_str());
        
        YAML::Node config = ar_common::readYamlFile(config_path);
        if (config.IsNull())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArHardwareInterface"), "Failed to load YAML configuration file");
            return;
        }
        YAML::Node port_ids = config["port_ids"];
        YAML::Node drives = config["drives"];
        YAML::Node drive_mode = config["driver_info"];
        soem_drives.clear();

        for (YAML::const_iterator it = port_ids.begin(); it != port_ids.end(); it++)
        {
            DriveParameter driveParam;
            driveParam.drive_mode = drive_mode["driver_mode"].as<int>();
            std::string drive_id = it->first.as<std::string>();
            driveParam.drive_id = ar_utils::stringToId(drive_id);
            driveParam.port_id = it->second.as<int>();
            driveParam.is_dual_axis = drives[drive_id]["is_dual_axis"] ? drives[drive_id]["is_dual_axis"].as<bool>() : false;
            if(is_simulation)
            {
                driveParam.port_id = NO_COMM;
                driveParam.slave_id = -1;
            }
            else if(driveParam.port_id == PORT_SOEM)
            {
                // driveParam.slave_id = static_cast<int>(soem_drives.size()) + 1;
                driveParam.slave_id = static_cast<int>(soem_drives.size());
                soem_drives.push_back(driveParam.slave_id); 
            }
            else
                driveParam.slave_id = -1;

            for (YAML::const_iterator jt = drives[drive_id]["joints"].begin(); jt != drives[drive_id]["joints"].end(); jt++)
            {
                JointParameter jointParam;
                jointParam.joint_name = jt->first.as<std::string>();
                const YAML::Node &joint_node = drives[drive_id]["joints"][jointParam.joint_name];
                if (YAML::Node parm = joint_node["gear_ratio"])
                    jointParam.gear_ratio = parm.as<int>();
                if (YAML::Node parm = joint_node["encoder_res"])
                    jointParam.encoder_res = parm.as<int>();
                if (YAML::Node parm = joint_node["encoder_offset"])
                    jointParam.encoder_offset = parm.as<int>();
                if (YAML::Node parm = joint_node["log_joint"])
                    jointParam.log_joint = parm.as<bool>();
                driveParam.joint_paramters.push_back(jointParam);
            }
            ar_drives.drive_parameters.push_back(driveParam);
        }

        std::sort(ar_drives.drive_parameters.begin(), ar_drives.drive_parameters.end(),
                  [](const auto &a, const auto &b)
                  {
                      return a.drive_id < b.drive_id;
                  });

        // Print out all loaded drive and joint parameters
        std::cout << "Loaded drive parameters:\n";
        for (const auto& drive : ar_drives.drive_parameters) {
            std::cout << "Drive ID: " << drive.drive_id
                      << ", Port ID: " << drive.port_id 
                      << ", Slave Id: " << drive.slave_id
                      << ", Is Dual Axis: " << (drive.is_dual_axis ? "True" : "False")
                      << ", Drive Mode: " << drive.drive_mode
                      << std::endl;
            for (const auto& joint : drive.joint_paramters) {
                std::cout << "  Joint Name: " << joint.joint_name
                          << ", Gear Ratio: " << joint.gear_ratio
                          << ", Encoder Res: " << joint.encoder_res
                          << ", Encoder Offset: " << joint.encoder_offset
                          << ", Log Joint: " << joint.log_joint << std::endl;
            }
        }
    }

    std::map<int, ArDriveControl *> ArHardwareInterface::getDrives()
    {
        return drives;
    }

    void ArHardwareInterface::read()
    {
        for (auto &drive : drives){
            drive.second->read();
        }
    }

    void ArHardwareInterface::write()
    {
        for (auto &drive : drives){
            drive.second->write();
        }
    }
} // namespace ar_control