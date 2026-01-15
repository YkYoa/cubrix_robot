#ifndef DEVICES_SERVER_H
#define DEVICES_SERVER_H

#include "rclcpp/rclcpp.hpp"
#include "message/ar_msg/msgs/joy.hpp"
#include "commlibs/ar_serial/include/serial.hpp"
#include "include/devices_server.h"
#include <mutex>
#include <string>

/**
 * @brief UART server for device communication
 * 
 * ROS2 node that reads from serial port and publishes device data
 * (e.g., joystick input) as ROS2 messages.
 */
class UARTServer : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    UARTServer();
    
    /**
     * @brief Destructor
     */
    ~UARTServer();

private:
    /**
     * @brief Timer callback for periodic operations
     */
    void timer_callback();
    
    /**
     * @brief Process received serial data
     * @param data_line Line of data from serial port
     */
    void process_serial_data(const std::string& data_line);

    Serial serial_;
    rclcpp::Publisher<ar_msg::msg::Joy>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mutex_;
    DevicesServer device_server_;
};


#endif // DEVICES_SERVER_H