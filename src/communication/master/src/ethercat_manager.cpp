#include "ethercat_manager.h"
#include <ar_common/common.h>


namespace master
{
    // Define the static member declared in the header
    bool EthercatManager::synch_flag_on_;

    EthercatManager::EthercatManager(uint8_t portId, std::string robotDesc, pthread_cond_t &pcond, pthread_mutex_t &pcond_lock, boost::mutex &mutex)
        : port_id(portId), robot_desc_(robotDesc), cond_(pcond), cond_lock_(pcond_lock), iomap_mutex_(mutex)
    {
        // Param pointer
        leadshine_param_ptr = std::make_shared<LeadshineParameters>();

        synch_flag_on_ = false;
        thread_parameter = nullptr;

        // get update rate from controller_manager
        std::string ros_controller_file;
        try
        {
            ros_controller_file = ament_index_cpp::get_package_share_directory(robot_desc_ + "_moveit_config") + "/config/ar_ros_controllers.yaml";
            YAML::Node ros_controller_config = YAML::LoadFile(ros_controller_file);
            int update_rate = ros_controller_config["controller_manager"]["ros__parameters"]["update_rate"].as<int>();
            if (!update_rate)
            {
                printf(COLOR_RED "Error: Update rate is 0. Please check the configuration file:\n%s\n" COLOR_RESET,
                       ros_controller_file.c_str());
                return;
            }

            DRIVER_SYNCH_TIME = 1000 / update_rate; // ms
            printf(COLOR_YELLOW "\nEtherCatManager initialized on port %d with update rate: %d Hz" COLOR_RESET, port_id, update_rate);
            printf(COLOR_YELLOW "\nDriver synch time: %d ms" COLOR_RESET, DRIVER_SYNCH_TIME);
        }
        catch (const YAML::BadFile &e)
        {
            printf(COLOR_RED "Error: Failed to load the configuration file:\n%s\n" COLOR_RESET, ros_controller_file.c_str());
            return;
        }
        catch (const YAML::Exception &e)
        {
            printf(COLOR_RED "YAML Error: %s\n" COLOR_RESET, e.what());
            return;
        }
    }

    bool EthercatManager::initialize(bool &bQuit, int slaveId)
    {
        if(getuid() != 0) {
			printf(COLOR_RED);
			printf("\n\n\n");
			printf("=============================================================\n");
			printf("||                                                         ||\n");
			printf("|| [EtherCatManager] Error: System must be run    ||\n");
			printf("||                    with sudo or as root.                ||\n");
			printf("||                                                         ||\n");
			printf("=============================================================\n");
			printf("\n\n\n" COLOR_RESET);
			usleep(1000000);
			return false;
		}
        
        std::string yamlpath = ar_common::getConfigPath();
        std::cout << COLOR_DARKYELLOW "EtherCatManager: Reading drive parameters from: " << yamlpath << COLOR_RESET << std::endl;
        YAML::Node config = ar_common::readYamlFile(yamlpath);
        if (config.IsNull())
        {
            std::cout << COLOR_RED "Error: Failed to load YAML configuration file" << COLOR_RESET << std::endl;
            return false;
        }

        p_cycle_thread_ = 0;
        p_handle_error_thread_ = 0;
        stop_flag = false;

        if(initSoem(bQuit, slaveId))
        {
            return false;
        }

        

        return true;
    }

    EthercatManager::~EthercatManager()
    {
        destroyEthercatManager();
    }

    bool EthercatManager::destroyEthercatManager()
    {
        try
        {
            stop_flag = true;
            // Request init operational state for all slaves
            soem.ec_slave[0].state = EC_STATE_INIT;

            /* Request init state for all slaves */
            soem.ec_writestate(0);

            // Stop SOEM, close socket
            soem.ec_close();

            printf(COLOR_YELLOW "\n[%s] Closing EtherCatManager[%d] port" COLOR_RESET "\n",
                   ar_utils::getCurrentTime(false, false).c_str(), port_id);
            fflush(stdout);

            if (p_handle_error_thread_)
            {
                pthread_join(p_handle_error_thread_, NULL);
            }

            if (p_cycle_thread_)
            {
                pthread_join(p_cycle_thread_, NULL);
            }

            if (thread_parameter)
            {
                delete thread_parameter;
                thread_parameter = nullptr; // Set to nullptr after deletion to avoid dangling pointer
            }

            return true; // Return true if everything succeeds
        }
        catch (const std::exception &e)
        {
            printf(COLOR_RED "[EtherCatManager] Error: %s" COLOR_RESET "\n", e.what());
            return false;
        }
        catch (...)
        {
            printf(COLOR_RED "[EtherCatManager] Unknown error occurred during EtherCatManager destruction." COLOR_RESET "\n");
            return false;
        }
    }

    bool EthercatManager::initSoem(bool &bQuit, int slaveId)
    {
        

        
    }

    ThreadParameters::ThreadParameters(boost::mutex &pmutex, SOEM &psoem, std::vector<int> &slaveIds, bool &pstop_flag, uint8_t &pport_Id,
                                       pthread_cond_t &pcond, pthread_mutex_t &pcond_lock, bool &psynch_flag_on,
                                       EtherCatStats &ethercat_stats, std::unordered_map<int, DriverInfo> &driverInfos)
        : mutex(pmutex), soem(psoem), stop_flag(pstop_flag), port_id(pport_Id), cond(pcond), cond_lock(pcond_lock),
          synch_flag_on(psynch_flag_on), etc_stats(ethercat_stats)
    {
        for (auto &info : driverInfos)
            joint_names[info.first] = info.second.joint_name;
        slave_ids.clear();
        for (auto &drive_id : slaveIds)
        {
            slave_ids.push_back(drive_id);
        }
    }

}