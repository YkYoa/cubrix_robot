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

    // ms, this parameter will be updated dynamically from the ROS controller configuration file
	[[maybe_unused]] static unsigned DRIVER_SYNCH_TIME = 10;

    struct EtherCatStats
    {
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
        DriverInfo() {};
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
    };

    class EthercatManager
    {
    public:
        EthercatManager(uint8_t portId, std::string robotDesc, pthread_cond_t &cond, pthread_mutex_t &cond_lock, boost::mutex &mutex);
        ~EthercatManager();

        bool initialize(bool &bQuit, int slaveId);
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
    private:
        SOEM soem;

        bool initSoem(bool& bQuit, int slaveId);

        void processPDOProcess(int slave_num);
        void readFromYamlFile(std::string yaml_path);
        void configProfilePosition(int slave_num, std::shared_ptr<LeadshineParameters> leadshine_param_ptr);

		pthread_t p_cycle_thread_, p_handle_error_thread_;

        std::shared_ptr<LeadshineParameters> leadshine_param_ptr;

        uint8_t port_id;
        uint8_t iomap_[4096];
		bool synch_flag_;
		static bool synch_flag_on_;
		YAML::Node ethercat_config_;
		std::string robot_desc_;
        bool stop_flag;
        
        // Thread
        boost::mutex& iomap_mutex_;
        ThreadParameters* thread_parameter;
        pthread_cond_t& cond_;
		pthread_mutex_t& cond_lock_;

    };
}

#endif