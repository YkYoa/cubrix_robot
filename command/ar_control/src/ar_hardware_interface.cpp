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

    ArHardwareInterface::ArHardwareInterface(std::string robotDesc, bool isSimulation, bool isUI)
    {
        // Initialize the hardware interface
        control_server_thread = nullptr;
        is_simulation = isSimulation;
        is_ui = isUI;

        
        std::string robot_description;
        
    }

}  // namespace ar_control