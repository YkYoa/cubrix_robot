#include "serial.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <queue>

Serial::Serial(const std::string& port, int baud_rate) : port_(port), baud_rate_(baud_rate) {}

bool Serial::open_port() {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    return fd_ != -1;
}

bool Serial::configure_port() {
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

void Serial::close_port() {
    if (fd_ != -1) close(fd_);
}

void Serial::read_serial(const std::function<void(const std::string&)>& callback) {
    std::thread([this, callback]() {
        char buffer[256];
        std::string line;
        while (fd_ != -1) {
            ssize_t n = read(fd_, buffer, sizeof(buffer) - 1);
            if (n > 0) {
                buffer[n] = '\0';
                line += buffer;
                size_t pos;
                while ((pos = line.find('\n')) != std::string::npos) {
                    std::string data_line = line.substr(0, pos);
                    line.erase(0, pos + 1);
                    if (!data_line.empty()) {
                        callback(data_line);
                    }
                }
            }
            usleep(10000); // 10ms delay
        }
    }).detach();
}