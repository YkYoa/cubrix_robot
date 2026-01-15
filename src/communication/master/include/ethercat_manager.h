#ifndef __ETHERCAT_MANAGER_H__
#define __ETHERCAT_MANAGER_H__

#include <string>
#include <vector>
#include <map>

#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <soem_cpp.h>
#include <driver_parameters.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>                                     
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/scoped_array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ar_utils.h>

namespace master
{
    enum MasterType{
        MASTER_IGH = 0,
        MASTER_SOEM,
    };

    // ms, this parameter will be updated dynamically from the ROS controller configuration file
	[[maybe_unused]] static unsigned DRIVER_SYNCH_TIME = 10;
    
    using boost::accumulators::accumulator_set;
	using boost::accumulators::stats;
	using boost::accumulators::tag::max;
	using boost::accumulators::tag::mean;

    struct EtherCatStats
    {
        accumulator_set<double, stats<max, mean>> ec_acc;

        double current_send_receive_period;
        double current_ethercat_loop;

        int overruns;
        int recent_overruns;
        int last_overrun;
        int last_severe_overrun;

        double overrun_loop_sec;
    };

    struct DriverInfo
    {
    public:
        DriverInfo() {
            control_mode = 1;
            is_dual_driver = false;
        };
        ~DriverInfo() {};

        bool operator==(DriverInfo &source) const
        {
            return source.driver_type == driver_type && source.joint_name == joint_name;
        }

        void operator=(const DriverInfo &source)
        {
            driver_type = source.driver_type;
            joint_name = source.joint_name;
        }

        std::string driver_type;
        std::string joint_name;
        bool is_dual_driver;
        int control_mode;

        struct LeadshineDriveData{
            std::string com_port;
            int control_mode;
        };
    };

    class ThreadParameters
    {
    public:
        ThreadParameters(boost::mutex &pmutex, SOEM &psoem, std::vector<int> &slaveIds, bool &pstop_flag, uint8_t &pport_Id,
                         pthread_cond_t &pcond, pthread_mutex_t &pcond_lock, bool &psynch_flag_on, EtherCatStats &ethercat_stats,
                         std::unordered_map<int, DriverInfo> &driverInfos);

        SOEM &soem;
        std::vector<int> slave_ids;
        bool &stop_flag;
        boost::mutex &mutex;
        pthread_cond_t &cond;
        pthread_mutex_t &cond_lock;
        bool &synch_flag_on;
        uint8_t &port_id;
        EtherCatStats &etc_stats;
        std::map<int, std::string> joint_names;

        // Thread-shared state
        int expected_wkc = 0;
        int sent = 0;
        int wkc = 0;
        bool process_data = false;
    };

    /**
     * @brief Interface for EtherCAT master communication
     * 
     * Abstract interface for reading/writing to EtherCAT slaves.
     * Implemented by EthercatManager (SOEM) and IghManager (IGH).
     */
    class EthercatMasterInterface
    {
    public:
        virtual ~EthercatMasterInterface() = default;
        
        /**
         * @brief Write a single byte to slave output
         * @param slave_no Slave number (1-based)
         * @param channel Byte offset in output IOMap
         * @param value Byte value to write
         */
        virtual void write(int slave_no, uint8_t channel, uint8_t value) = 0;
        
        /**
         * @brief Write a buffer to slave output
         * @param slave_no Slave number (1-based)
         * @param buffer Buffer to write
         * @param size Buffer size in bytes
         */
        virtual void writeBuffer(int slave_no, const uint8_t* buffer, int size) = 0;
        
        /**
         * @brief Read a single byte from slave input
         * @param slave_no Slave number (1-based)
         * @param channel Byte offset in input IOMap
         * @return Byte value read
         */
        virtual uint8_t readInput(int slave_no, uint8_t channel) const = 0;
        
        /**
         * @brief Read a single byte from slave output
         * @param slave_no Slave number (1-based)
         * @param channel Byte offset in output IOMap
         * @return Byte value read
         */
        virtual uint8_t readOutput(int slave_no, uint8_t channel) const = 0;
        
        /**
         * @brief Get input bits for a slave
         * @param slave_no Slave number (1-based)
         * @return Input bits value
         */
        virtual int getInputBits(int slave_no) const = 0;
        
        /**
         * @brief Get output bits for a slave
         * @param slave_no Slave number (1-based)
         * @return Output bits value
         */
        virtual int getOutputBits(int slave_no) const = 0;
        
        /**
         * @brief Wait for specified number of EtherCAT cycles
         * @param num_cycles Number of cycles to wait
         */
        virtual void waitForCycles(int num_cycles) = 0;
    };

    /**
     * @brief SOEM-based EtherCAT manager
     * 
     * Manages EtherCAT communication using the SOEM library.
     * Handles slave configuration, PDO mapping, and cyclic data exchange.
     */
    class EthercatManager : public EthercatMasterInterface
    {
    public:
        /**
         * @brief Constructor
         * @param portId EtherCAT port ID
         * @param robotDesc Robot description
         * @param cond Condition variable for synchronization
         * @param cond_lock Mutex for condition variable
         * @param mutex Communication mutex
         */
        EthercatManager(uint8_t portId, std::string robotDesc, pthread_cond_t &cond
                , pthread_mutex_t &cond_lock, boost::mutex &mutex);
        
        /**
         * @brief Destructor
         */
        ~EthercatManager();

        /**
         * @brief Initialize EtherCAT master
         * @param bQuit Reference to quit flag
         * @param slaveIds Vector of expected slave IDs
         * @return true if initialization succeeded
         */
        bool initialize(bool &bQuit, std::vector<int> slaveIds);
        
        /**
         * @brief Destroy and cleanup EtherCAT manager
         * @return true if cleanup succeeded
         */
        bool destroyEthercatManager();

        /**
         * \brief writes 'value' to the 'channel-th' output-register of the given 'slave'
         *
         * @param[in] slave_no The slave number of the device to write to (>= 1)
         * @param[in] channel The byte offset into the output IOMap to write value to
         * @param[in] value The byte value to write
         *
         * This method currently makes no attempt to catch out of bounds errors. Make
         * sure you know your IOMap bounds.
         */
        void write(int slave_no, uint8_t channel, uint8_t value);
        
        /**
         * \brief Atomically writes a buffer to the output registers (for SOEM compatibility)
         */
        void writeBuffer(int slave_no, const uint8_t* buffer, int size);
        
        /**
         * \brief Wait for N complete EtherCAT cycles (for synchronization)
         */
        void waitForCycles(int num_cycles);

        /**
         * \brief Reads the "channel-th" input-register of the given slave no
         *
         * @param[in] slave_no The slave number of the device to read from (>= 1)
         * @param[in] channel The byte offset into the input IOMap to read from
         */
        uint8_t readInput(int slave_no, uint8_t channel) const;

        /**
         * \brief Reads the "channel-th" output-register of the given slave no
         *
         * @param[in] slave_no The slave number of the device to read from (>= 1)
         * @param[in] channel The byte offset into the output IOMap to read from
         */
        uint8_t readOutput(int slave_no, uint8_t channel) const;

        /**
         * \brief write the SDO object of the given slave no
         *
         * @param[in] slave_no The slave number of the device to read from (>= 1)
         * @param[in] index The index address of the parameter in SDO object
         * @param[in] subidx The sub-index address of the parameter in SDO object
         * @param[in] value value to write
         */
        template <typename T>
        uint8_t writeSDO(int slave_no, uint16_t index, uint8_t subidx, T value) const;

        /**
         * \brief read the SDO object of the given slave no
         *
         * @param[in] slave_no The slave number of the device to read from (>= 1)
         * @param[in] index The index address of the parameter in SDO object
         * @param[in] subidx The sub-index address of the parameter in SDO object
         */
        template <typename T>
        T readSDO(int slave_no, uint16_t index, uint8_t subidx) const;

        /**
         * \brief get the number of clients
         */
        int getNumClinets() const;

        /**
         * \brief get the status of clients
         */
        void getStatus(int slave_no, std::string &name, int &eep_man, int &eep_id, int &eep_rev, int &obits, int &ibits, int &state, int &pdelay, int &hasdc, int &activeports, int &configadr) const;
        
        /**
         * @brief Get input bits for a slave (interface implementation)
         * @param slave_no Slave number (1-based)
         * @return Input bits value
         */
        int getInputBits(int slave_no) const;
        
        /**
         * @brief Get output bits for a slave (interface implementation)
         * @param slave_no Slave number (1-based)
         * @return Output bits value
         */
        int getOutputBits(int slave_no) const;

        /**
         * @brief Get driver information for a slave
         * @param slave_no Slave number (1-based)
         * @return Driver information structure
         */
        DriverInfo getDriverInfo(int slave_no) const;

    
    private:
        SOEM soem; ///< SOEM library instance

        /**
         * @brief Initialize SOEM library
         * @param bQuit Reference to quit flag
         * @param slaveIds Vector of expected slave IDs
         * @return true if initialization succeeded
         */
        bool initSoem(bool& bQuit, std::vector<int> slaveIds);

        /**
         * @brief Configure PDO process data for a slave
         * @param slave_num Slave number
         */
        void configPDOProcess(int slave_num);
        
        /**
         * @brief Read slave configuration from YAML file
         * @param slave_no Slave number
         * @return true if read successfully
         */
        bool readFromYamlFile(int slave_no);
        
        /**
         * @brief Configure PDO for profile position mode
         * @param slave_num Slave number
         * @param leadshine_param_ptr Leadshine parameters pointer
         */
        void configPDOProfilePosition(int slave_num, std::shared_ptr<LeadshineParameters> leadshine_param_ptr);
        
        /**
         * @brief Configure PDO for cyclic position mode
         * @param slave_num Slave number
         * @param leadshine_param_ptr Leadshine parameters pointer
         */
        void configPDOCyclicPosition(int slave_num, std::shared_ptr<LeadshineParameters> leadshine_param_ptr);
        
        /**
         * @brief Configure PDO for dual-axis cyclic mode
         * @param slave_num Slave number
         * @param leadshine_param_ptr Leadshine parameters pointer
         */
        void configPDODualCyclic(int slave_num, std::shared_ptr<LeadshineParameters> leadshine_param_ptr);

        
        std::shared_ptr<LeadshineParameters> leadshine_param_ptr;
        std::unordered_map<int, DriverInfo> driver_infos_;
        EtherCatStats etc_stats;
        
        uint8_t port_id;
        uint8_t iomap_[4096];
		bool synch_flag_;
		static bool synch_flag_on_;
		YAML::Node ethercat_config_;
		std::string robot_desc_;
        bool stop_flag;
        int max_slave_count;
        int driver_count;
        
        // Thread
        boost::mutex& iomap_mutex_;
        ThreadParameters* thread_parameter;
        pthread_cond_t& cond_;
		pthread_mutex_t& cond_lock_;
		pthread_t p_cycle_thread_, p_handle_error_thread_;

        //ticket related
    };
}

#endif