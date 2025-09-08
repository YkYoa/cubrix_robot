#ifndef __ETHERCAT_MANAGER_H__
#define __ETHERCAT_MANAGER_H__



#include <string>
#include <vector>

#include <soem_cpp.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/scoped_array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


namespace master
{
    using boost::accumulators::accumulator_set;

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
        DriverInfo() {};
        ~DriverInfo() {};

        bool operator==(DriverInfo& source) const
        {
            return source.driver_type == driver_type && source.joint_name == joint_name;
        }

        void operator=(const DriverInfo& source)
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
		ThreadParameters(boost::mutex& pmutex, SOEM& psoem, std::vector<int>& slaveIds, bool& pstop_flag, uint8_t& pport_Id,
						 pthread_cond_t& pcond, pthread_mutex_t& pcond_lock, bool& psynch_flag_on, EtherCatStats& ethercat_stats,
						 std::unordered_map<int, DriverInfo>& driverInfos);

		SOEM& soem;
        std::vector<int> slave_ids;
		bool& stop_flag;
		boost::mutex& mutex;
		pthread_cond_t& cond;
		pthread_mutex_t& cond_lock;
		bool& synch_flag_on;
		uint8_t& port_id;
		EtherCatStats& etc_stats;
		std::map<int, std::string> joint_names;

    };

    class EthercatManager
    {

    };
}





#endif 