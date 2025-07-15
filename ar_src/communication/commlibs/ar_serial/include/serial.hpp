#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <string>
#include <functional>

class Serial {
public:
    Serial(const std::string& port, int baud_rate);
    bool open_port();
    bool configure_port();
    void close_port();
    void read_serial(const std::function<void(const std::string&)>& callback);

private:
    std::string port_;
    int baud_rate_;
    int fd_ = -1;
};

#endif // SERIAL_HPP