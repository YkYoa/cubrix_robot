#include <ar_control/ar_hardware_interface.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ar_control
{
    void ArHardwareInterface::launchControlServer()
    {
        RCLCPP_INFO(rclcpp::get_logger("Ar Robot"), "Launching control server...");
        ar_control_server = std::make_shared<ArControlServer>(this);
        ar_control_server->startActionServer();
        rclcpp::spin(ar_control_server);
    }

    ArHardwareInterface::ArHardwareInterface(bool isSimulation, bool isUI)
    {
        // Initialize the hardware interface
        control_server_thread = nullptr;
        is_simulation = isSimulation;
        is_ui = isUI;

        std::string robot_description;
        control_server_thread = new std::thread(&ArHardwareInterface::launchControlServer, this);

        ar_hardware_interface_node_ = rclcpp::Node::make_shared("ar_hardware_interface_node",
                                                                rclcpp::NodeOptions(), automatically_declare_parameters_from_overrides(true));

        if (!ar_hardware_interface_node_->get_parameter("robot_description", robot_description))
        {
            RCLCPP_ERROR(ar_hardware_interface_node_->get_logger(), "Parameter 'robot_description' not found.");
        }

        if (!urdf_model.initString(robot_description))
        {
            RCLCPP_ERROR(ar_hardware_interface_node_->get_logger(), "Failed to parse URDF model.");
        }

        std::string config_path = ament_index_cpp::get_package_share_directory("ar_control") + "/config/ar_drive.yaml";
        readConfigFromYaml(config_path);

        RCLCPP_INFO(rclcpp::get_logger("Ar"), "[Ar Hardware Interface] Initializing PROTOCOL manager, drives and joints");



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

    void ArHardwareInterface::readConfigFromYaml(std::string yaml_path)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArHardwareInterface"), COLOR_DARKYELLOW "Drive config path: %s" COLOR_RESET, yaml_path.c_str());
        YAML::Node config;
        try
        {
            config = YAML::LoadFile(yaml_path);
        }
        catch (const YAML::BadFile &e)
        {
            std::cout << COLOR_RED "Error: Failed to load the YAML file: " << e.what() << COLOR_RESET << std::endl;
            return;
        }
        catch (const YAML::ParserException &e)
        {
            std::cout << COLOR_RED "Error: YAML parsing error: " << e.what() << COLOR_RESET << std::endl;
            return;
        }
        catch (const std::exception &e)
        {
            std::cout << COLOR_RED "Error: An unexpected error occurred: " << e.what() << COLOR_RESET << std::endl;
            return;
        }
        YAML::Node port_ids = config["port_ids"];
        YAML::Node drives = config["drives"];

        for (YAML::const_iterator it = port_ids.begin(); it != port_ids.end(); it++)
        {
            DriveParameters driveParams;
            std::string drive_id = it->first.as<std::string>();
            driveParams.drive_id = stringToId(drive_id);
            driveParams.port_id = it->second.as<int>();
            if (is_simulation)
            {
                driveParams.port_id = NO_COMM;
            }
            else if (driveParams.port_id == PORT_UART)
            {
                std::cout << COLOR_DARKYELLOW "Port ID: " << driveParams.port_id << COLOR_RESET << std::endl;
            }
            else if (driveParams.port_id == PORT_CAN)
            {
                std::cout << COLOR_DARKYELLOW "Port ID: " << driveParams.port_id << COLOR_RESET << std::endl;
            }
            else
            {
                RCLCPP_ERROR(ar_hardware_interface_node_->get_logger(), "Invalid port ID: %d", driveParams.port_id);
            }
            for (YAML::const_iterator jt = drives[drive_id]["joints"].begin(); jt != drives[drive_id]["joints"].end(); jt++)
            {
                JointParameters jointParams;
                jointParams.joint_name = jt->first.as<std::string>();
                const YAML::Node &joint_node = drives[drive_id]["joints"][jointParams.joint_name];
                if (YAML::Node parm = joint_node["client_id"])
                    jointParams.client_id = parm.as<int>();
                if (YAML::Node parm = joint_node["gear_ratio"])
                    jointParams.gear_ratio = parm.as<int>();
                if (YAML::Node parm = joint_node["encoder_res"])
                    jointParams.encoder_res = parm.as<int>();
                if (YAML::Node parm = joint_node["encoder_offset"])
                    jointParams.encoder_offset = parm.as<int>();
                if (YAML::Node parm = joint_node["log_joint"])
                    jointParams.log_joint = parm.as<bool>();
                driveParams.joint_paramters.push_back(jointParams);
            }
            ar_drives.drive_parameters.push_back(driveParams);
        }

        std::sort(ar_drives.drive_parameters.begin(), ar_drives.drive_parameters.end(),
                  [](const auto &a, const auto &b)
                  {
                      return a.drive_id < b.drive_id;
                  });
    }

} // namespace ar_control