#pragma once 

#include "igh_gobal.hpp"

#undef CONTROL_WORD
#undef STATUS_WORD
#undef MODE_OF_OPERATION
#undef TARGET_POSITION
#undef TOUCH_PROBE_FUNCTION
#undef ERROR_CODE
#undef MODE_OF_OPERATION_DISPLAY
#undef ACTUAL_POSITION
#undef TOUCH_PROBE_STATUS
#undef DIGITAL_INPUTS

#include "ethercat_manager.h"

namespace master {
    class IghErrorHandler;
}

class IghSlave;
#include "igh_slave.hpp"

namespace master
{
/**
 * @brief IGH EtherCAT master manager
 * 
 * Manages EtherCAT communication using the IGH EtherCAT library.
 * Handles master configuration, slave setup, PDO mapping, and cyclic communication.
 */
class IghManager : public EthercatMasterInterface
{
    public:
        /**
         * @brief Constructor
         * @param cond Condition variable for synchronization
         * @param cond_lock Mutex for condition variable
         * @param mutex Communication mutex
         */
        IghManager(pthread_cond_t& cond, pthread_mutex_t& cond_lock, boost::mutex& mutex);
        
        /**
         * @brief Destructor
         */
        ~IghManager();

        static const int max_slave_num_ = 5; ///< Maximum number of slaves
        IghSlave slave_[max_slave_num_]; ///< Array of slave configurations

        /**
         * @brief Configure the EtherCAT master
         * @return 0 on success, error code on failure
         */
        int configMaster();
        
        /**
         * @brief Set slave configuration at position
         * @param slave Slave configuration
         * @param position Position in slave array
         * @return 0 on success
         */
        int setSlaves(IghSlave slave, int position);
        
        /**
         * @brief Configure all slaves
         * @return 0 on success
         */
        int configSlaves();
        
        /**
         * @brief Set profile position parameters
         * @param P Profile position parameters
         * @return 0 on success
         */
        int setProFilePositionParameters(ProFilePositionParm& P);
        
        /**
         * @brief Set cyclic position parameters
         * @return 0 on success
         */
        int setCyclicPositionParameters();
        
        /**
         * @brief Map default PDOs for a slave
         * @param slave Slave to configure
         * @param position Slave position
         * @return 0 on success
         */
        int mapDefaultPDOs(IghSlave& slave, int position);
        
        /**
         * @brief Configure DC sync with default settings
         */
        void configDcSyncDefault();
        
        /**
         * @brief Configure DC sync for specific assignment
         * @param assign_activated DC assignment activation
         * @param position Slave position
         */
        void configDcSync(uint16_t assign_activated, int position);
        
        /**
         * @brief Check master state
         * @return 0 if operational
         */
        int checkMasterState();
        
        /**
         * @brief Check master domain state
         */
        void checkMasterDomainState();
        
        /**
         * @brief Open EtherCAT master device
         * @return 0 on success
         */
        int openEthercatMaster();
        
        /**
         * @brief Get number of connected slaves
         * @return Number of slaves
         */
        int getNumbOfConnectedSlaves();
        
        /**
         * @brief Get information about all slaves
         */
        void getAllSlavesInfo();
        
        /**
         * @brief Deactivate the master
         */
        void deactivateMaster();
        
        /**
         * @brief Release the master
         */
        void releaseMaster();
        
        /**
         * @brief Shutdown the master
         * @return 0 on success
         */
        int shutdown();

        /**
         * @brief Activate the master
         * @return 0 on success
         */
        int activateMaster();
        
        /**
         * @brief Register PDO domain
         * @return 0 on success
         */
        int registerDomain();
        
        /**
         * @brief Initialize PDO domain
         * @return 0 on success
         */
        int initializePdoDomain();
        
        /**
         * @brief Wait for operational mode
         * @return 0 on success
         */
        int waitForOpMode();
        
        /**
         * @brief Read SDO (Service Data Object)
         * @param pack SDO data packet
         * @return Error code (0 = success)
         */
        uint8_t SDOread(SDO_data& pack);
        
        /**
         * @brief Write SDO (Service Data Object)
         * @param pack SDO data packet
         * @return Error code (0 = success)
         */
        uint8_t SDOwrite(SDO_data& pack);
        
        /**
         * @brief Start cyclic communication thread
         * @return 0 on success
         */
        int startCyclicCommunication();
        
        /**
         * @brief Stop cyclic communication
         */
        void stopCyclicCommunication();
        
        /**
         * @brief Cyclic thread entry point
         * @param arg Thread argument (IghManager pointer)
         * @return nullptr
         */
        static void* cyclicThread(void* arg);
        
        /**
         * @brief Main cyclic communication loop
         */
        void cyclicLoop();
        
        /**
         * @brief Write byte to slave output (interface implementation)
         * @param slave_no Slave number (1-based)
         * @param channel Byte offset
         * @param value Byte value
         */
        virtual void write(int slave_no, uint8_t channel, uint8_t value) override;
        
        /**
         * @brief Write buffer to slave output (interface implementation)
         * @param slave_no Slave number (1-based)
         * @param buffer Buffer to write
         * @param size Buffer size
         */
        virtual void writeBuffer(int slave_no, const uint8_t* buffer, int size) override;
        
        /**
         * @brief Read byte from slave input (interface implementation)
         * @param slave_no Slave number (1-based)
         * @param channel Byte offset
         * @return Byte value
         */
        virtual uint8_t readInput(int slave_no, uint8_t channel) const override;
        
        /**
         * @brief Read byte from slave output (interface implementation)
         * @param slave_no Slave number (1-based)
         * @param channel Byte offset
         * @return Byte value
         */
        virtual uint8_t readOutput(int slave_no, uint8_t channel) const override;
        
        /**
         * @brief Get input bits (interface implementation)
         * @param slave_no Slave number (1-based)
         * @return Input bits value
         */
        virtual int getInputBits(int slave_no) const override;
        
        /**
         * @brief Get output bits (interface implementation)
         * @param slave_no Slave number (1-based)
         * @return Output bits value
         */
        virtual int getOutputBits(int slave_no) const override;
        
        /**
         * @brief Wait for specified number of cycles
         * @param num_cycles Number of cycles to wait
         */
        void waitForCycles(int num_cycles);
        
        /**
         * @brief Get current cycle counter
         * @return Cycle counter value
         */
        uint32_t getCycleCounter() const { return cycle_counter_; }

        int fd;
        
        friend class IghErrorHandler;
        
    private:
        int file_description;
        bool stop_flag_;
        pthread_t cyclic_thread_;
        int num_slaves_;
        volatile uint32_t cycle_counter_;
        
        pthread_cond_t& cond_;
        pthread_mutex_t& cond_lock_;
        boost::mutex& iomap_mutex_;
        
        IghErrorHandler* error_handler_;
};

}   // namespace master