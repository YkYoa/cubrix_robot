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
            if(drive->jointCount() == 1)
                drive->InitializeDrive();
        }
    }

    ArHardwareInterface::ArHardwareInterface(std::vector<std::shared_ptr<boost::mutex>> comm_mutex, pthread_cond_t& cond, 
        pthread_mutex_t& cond_lock, std::string robotDesc, bool& bQuit, bool isSimulation, bool isUI) : b_quit_(bQuit)
    {
        // Initialize the hardware interface
        control_server_thread = nullptr;
        is_simulation = isSimulation;
        is_ui = isUI;

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

        if(!is_simulation){
            portManager = new master::EthercatManager((uint8_t) PORT_SOEM, robotDesc, cond, cond_lock, *comm_mutex[PORT_SOEM]);
            if(!portManager->initialize(b_quit_, soem_drives)){
                b_quit_ = true;
                return;
            }
        }

        for(auto& driveParm : ar_drives.drive_parameters){
            drives[driveParm.drive_id] = new ArDriveControl(driveParm, is_ui);
            drives[driveParm.drive_id]->InitializeDriveClient(driveParm.port_id);

            for(auto& jointParm : driveParm.joint_paramters){
                auto urdf_joint = urdf_model.getJoint(jointParm.joint_name);
                drives[driveParm.drive_id]->AddJoint(jointParm);
            }
        }

        std::vector<ArDriveControl*> drives_ptr;

        for(auto& drive : drives){
            if(drive.second)
                drives_ptr.push_back(drive.second);
        }

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
        if (control_server_thread)
            control_server_thread->join();

        RCLCPP_INFO(rclcpp::get_logger("ArHardwareInterface"), "Shut down complete.");
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

        for (YAML::const_iterator it = port_ids.begin(); it != port_ids.end(); it++)
        {
            DriveParameter driveParam;
            std::string drive_id = it->first.as<std::string>();
            driveParam.drive_id = ar_utils::stringToId(drive_id);
            driveParam.port_id = it->second.as<int>();
            if(is_simulation)
            {
                driveParam.port_id = NO_COMM;
            }
            else if(driveParam.port_id == PORT_SOEM)
            {
                soem_drives++;
                driveParam.slave_id = soem_drives;
            }
            // else if (driveParam.port_id == DISCONNECTED)
            // {
            //     std::cout << COLOR_DARKYELLOW "Port ID: " << driveParam.port_id << COLOR_RESET << std::endl;
            // }
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
            std::cout << "Drive ID: " << drive.drive_id << ", Port ID: " << drive.port_id << ", Slave Id: " << drive.slave_id << std::endl;
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