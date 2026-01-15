#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <string>
#include <functional>

/**
 * @brief Serial port communication interface
 * 
 * Provides functionality to open, configure, and read from serial ports.
 */
class Serial {
public:
    /**
     * @brief Constructor
     * @param port Serial port path (e.g., "/dev/ttyUSB0")
     * @param baud_rate Baud rate (e.g., 115200)
     */
    Serial(const std::string& port, int baud_rate);
    
    /**
     * @brief Open the serial port
     * @return true if port opened successfully
     */
    bool open_port();
    
    /**
     * @brief Configure port settings (baud rate, parity, etc.)
     * @return true if configuration succeeded
     */
    bool configure_port();
    
    /**
     * @brief Close the serial port
     */
    void close_port();
    
    /**
     * @brief Read from serial port with callback
     * @param callback Function to call with received data
     */
    void read_serial(const std::function<void(const std::string&)>& callback);

private:
    std::string port_;
    int baud_rate_;
    int fd_ = -1;
};

#endif // SERIAL_HPP