#include "commlibs/ar_serial/include/serial.hpp"
#include "ar_msg/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "message/ar_msg/msgs/joy.hpp"
#include "devices_server.h"
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>

// Placeholder Serial class definition (to be replaced with actual ar_serial implementation)
class Serial {
public:
    Serial(const std::string& port, int baud_rate) : port_(port), baud_rate_(baud_rate), fd_(-1) {}
    bool open_port() {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        return fd_ != -1;
    }
    bool configure_port() {
        if (fd_ == -1) return false;
        struct termios options;
        tcgetattr(fd_, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        return tcsetattr(fd_, TCSANOW, &options) == 0;
    }
    void close_port() { if (fd_ != -1) ::close(fd_); }
    void read_serial(const std::function<void(const std::string&)>& callback) {
        std::thread([this, callback]() {
            char buffer[256];
            std::string line;
            while (fd_ != -1) {
                ssize_t n = ::read(fd_, buffer, sizeof(buffer) - 1);
                if (n > 0) {
                    buffer[n] = '\0';
                    line += buffer;
                    size_t pos;
                    while ((pos = line.find('\n')) != std::string::npos) {
                        std::string data_line = line.substr(0, pos);
                        line.erase(0, pos + 1);
                        if (!data_line.empty()) callback(data_line);
                    }
                }
                usleep(10000); // 10ms delay
            }
        }).detach();
    }
private:
    std::string port_;
    int baud_rate_;
    int fd_;
};

// Placeholder DevicesServer class definition (to be replaced with actual devices_server.h)
class DevicesServer {
public:
    DevicesServer() { std::cout << "DevicesServer initialized" << std::endl; }
    ~DevicesServer() {}
    void register_device(const std::string& name) {
        std::cout << "Registered device: " << name << std::endl;
    }
};

UARTServer::UARTServer() : Node("uart_server"), serial_("/dev/ttyUSB0", 115200), device_server_() {
    publisher_ = this->create_publisher<ar_msgs::msg::Joy>("joy_data", 10);
    device_server_.register_device("uart_server");

    // Open and configure UART port
    if (!serial_.open_port()) {
        RCLCPP_ERROR(this->get_logger(), "not open port /dev/ttyUSB0");
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(this->get_logger(), "port opened /dev/ttyUSB0");

    if (!serial_.configure_port()) {
        RCLCPP_ERROR(this->get_logger(), "don't conf UART");
        rclcpp::shutdown();
        return;
    }

    // Initialize timer for periodic reading
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&UARTServer::timer_callback, this));

    // Register serial data callback
    serial_.read_serial([this](const std::string& data_line) {
        process_serial_data(data_line);
    });
}

UARTServer::~UARTServer() {
    serial_.close_port();
}

void UARTServer::timer_callback() {
    // The actual reading is handled by the serial class's thread
    // This timer can be used for additional server tasks if needed
}

void UARTServer::process_serial_data(const std::string& data_line) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_line.rfind("DATA:", 0) == 0) {
        try {
            std::string data_str = data_line.substr(5);
            std::stringstream ss(data_str);
            std::string token;

            float x = 0.0, y = 0.0;
            bool button = false;

            std::getline(ss, token, ',');
            x = std::stof(token);

            std::getline(ss, token, ',');
            y = std::stof(token);

            std::getline(ss, token, ',');
            button = (std::stoi(token) == 0);

            ar_msg::msg::Joy msg;
            msg.axes = {x, y}; // Assuming axes is a vector of floats for x, y
            msg.buttons = {static_cast<int>(button)}; // Assuming buttons is a vector of ints
            publisher_->publish(msg);

            // Get timestamp
            auto now = std::chrono::system_clock::now();
            auto tt = std::chrono::system_clock::to_time_t(now);
            struct tm* ptm = localtime(&tt);
            std::stringstream time_str;
            time_str << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");

            RCLCPP_INFO(this->get_logger(),
                       "Thời gian: %s, Gửi: x=%.0f, y=%.0f, button=%s",
                       time_str.str().c_str(), x, y, button ? "PRESSED" : "RELEASED");
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Dữ liệu lỗi: '%s'", data_line.c_str());
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UARTServer>());
    rclcpp::shutdown();
    return 0;
}