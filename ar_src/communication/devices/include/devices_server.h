#ifndef DEVICES_SERVER_H
#define DEVICES_SERVER_H

#include "rclcpp/rclcpp.hpp"
#include "message/ar_msg/msgs/joy.hpp"
#include "commlibs/ar_serial/include/serial.hpp"
#include "include/devices_server.h"
#include <mutex>
#include <string>

class UARTServer : public rclcpp::Node {
public:
    UARTServer();
    ~UARTServer();

private:
    void timer_callback();
    void process_serial_data(const std::string& data_line);

    Serial serial_;
    rclcpp::Publisher<ar_msg::msg::Joy>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mutex_;
    DevicesServer device_server_;
};


#endif // DEVICES_SERVER_H