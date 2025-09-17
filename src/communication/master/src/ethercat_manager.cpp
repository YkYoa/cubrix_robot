#include "ethercat_manager.h"
#include <ar_common/common.h>


namespace master
{
    // Define the static member declared in the header
    bool EthercatManager::synch_flag_on_;

    // ETC log
    auto ETC = rclcpp::get_logger("EtherCatManager");

    //specified in ns, us
    static const int NSEC_PER_SECOND = 1e+9;
    static const int STACK64 = 64 * 1024;
    static const int EC_TIMEOUTRET_2 = 2000; // us
    static const int EC_TIMEOUTRET_5 = 5000; // us
    static const int EC_TIMEOUTRET_8 = 8000; // us

    void timespecInc(struct timespec *tic, int nsec)
    {
        tic->tv_nsec += nsec;
        while (tic->tv_nsec >= NSEC_PER_SECOND)
        {
            tic->tv_nsec -= NSEC_PER_SECOND;
            tic->tv_sec++;
        }
    }

	void ec_sync(int64 reftime, int64 cycletime, int64* offsettime)
	{
		static int64 integral = 0;
		int64 delta;
		/* set linux sync point 50us later than DC sync, just as example */
		delta = (reftime - 50000) % cycletime;
		if(delta > (cycletime / 2)) {
			delta = delta - cycletime;
		}
		if(delta > 0) {
			integral++;
		}
		if(delta < 0) {
			integral--;
		}
		*offsettime = -(delta / 100) - (integral / 20);
		// gl_delta	= delta;
	}

    double now()
    {
        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);
        return t.tv_sec + t.tv_nsec / NSEC_PER_SECOND;
    }

    template <typename T> T readSDO(int slave_no, uint16_t index, uint8_t subidx, SOEM& soem)
	{
		int ret, l;
		T val;
		l	= sizeof(val);
		ret = soem.ec_SDOread(slave_no, index, subidx, FALSE, &l, &val, EC_TIMEOUTRXM);
		if(ret <= 0) {	// ret = Workcounter from last slave response
			printf("Failed to read from ret:%d, slave_no:%d, index:0x%04x, subidx:0x%02x\n", ret, slave_no, index, subidx);
			fflush(stdout);
		}
		return val;
	}

	template uint16_t readSDO<uint16_t>(int slave_no, uint16_t index, uint8_t subidx, SOEM& soem);
	template uint32_t readSDO<uint32_t>(int slave_no, uint16_t index, uint8_t subidx, SOEM& soem);

        OSAL_THREAD_FUNC handleErrors(void *thread_param)
    {
        ThreadParameters *error_thread_param = (ThreadParameters *) thread_param;
        uint8_t delay_count[error_thread_param->slave_ids.size()] = {0};
        int max_check_operation_mode =5;

        while (!error_thread_param->stop_flag)
        {
            if (error_thread_param->process_data)
            {
                error_thread_param->soem.ec_group[0].docheckstate = false;
                error_thread_param->soem.ec_readstate();

                printf(COLOR_YELLOW "[EtherCatManager] Checking state of slaves..." COLOR_RESET "\n");
                bool stop_flag_on = false;
                for (auto slave : error_thread_param->slave_ids)
                {
                    const char *jointName = error_thread_param->joint_names[slave].c_str();
                    if (error_thread_param->soem.ec_slave[slave].state != EC_STATE_OPERATIONAL)
                    {
                        error_thread_param->soem.ec_group[0].docheckstate = true;
                        if (error_thread_param->soem.ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                        {
                            printf(COLOR_RED "[EtherCatManager] Error: Slave %d (%s) is in SAFE_OP + ERROR, attempting ack." COLOR_RESET "\n", slave, jointName);
                            error_thread_param->soem.ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                            error_thread_param->soem.ec_writestate(slave);
                        }else if(error_thread_param->soem.ec_slave[slave].state == EC_STATE_SAFE_OP){
                            printf(COLOR_RED "[EtherCatManager] Warning: Slave %d (%s) is in SAFE_OP, attempting to change to OPERATIONAL." COLOR_RESET "\n", slave, jointName);
                            error_thread_param->soem.ec_slave[slave].state = EC_STATE_OPERATIONAL;
                            error_thread_param->soem.ec_writestate(slave);
                        }
                        else if (error_thread_param->soem.ec_slave[slave].state > 0)
                        {
                            if(error_thread_param->soem.ec_reconfig_slave(slave, EC_TIMEOUTRET_5)){
                                error_thread_param->soem.ec_slave[slave].islost = false;
                                printf(COLOR_RED "[EtherCatManager] Warning: Slave %d (%s) reconfigured." COLOR_RESET "\n", slave, jointName);
                                
                                int32_t operation_mode = readSDO<int32_t>(slave, 0x6060, 0x00, error_thread_param->soem);
                                uint16_t status_word = readSDO<uint16_t>(slave, 0x6041, 0x00, error_thread_param->soem);
                                uint16_t error_code = readSDO<uint16_t>(slave, 0x603F, 0x00, error_thread_param->soem);
                                printf(COLOR_RED "[EtherCatManager] slave %d (%s) op mode: 0x%04X, status word: 0x%04X, error code: 0x%04X" COLOR_RESET "\n", 
                                                                        slave, jointName, operation_mode, status_word, error_code);

                            }
                        }
                        else if (!error_thread_param->soem.ec_slave[slave].islost)
                        {
                            error_thread_param->soem.ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET_8);
                            if(!error_thread_param->soem.ec_slave[slave].state){
                                error_thread_param->soem.ec_slave[slave].islost = true;
                                delay_count[slave - 1] = 5;
                                printf(COLOR_RED "[EtherCatManager] Error: Slave %d (%s) lost." COLOR_RESET "\n", slave, jointName);
                            }
                        }                        
                    }
                    else if(error_thread_param->soem.ec_slave[slave].islost){
                        stop_flag_on = true;
                        if(!error_thread_param->soem.ec_slave[slave].state){
                            if(delay_count[slave - 1] > 0){
                                delay_count[slave - 1]--;
                            }else{
                                if(error_thread_param->soem.ec_recover_slave(slave, EC_TIMEOUTRET_5)){
                                    error_thread_param->soem.ec_slave[slave].islost = false;
                                    printf(COLOR_RED "[EtherCatManager] Info: Slave %d (%s) recovered." COLOR_RESET "\n", slave, jointName);
                                }else{
                                    stop_flag_on = true;
                                }
                            }
                        }else{
                            error_thread_param->soem.ec_slave[slave].islost = false;
                            printf(COLOR_RED "[EtherCatManager] Info: Slave %d (%s) recovered." COLOR_RESET "\n", slave, jointName);
                        }
                    }
                }
                error_thread_param->process_data = false;

            }
            osal_usleep(20000);
        }
    }

    OSAL_THREAD_FUNC_RT CycleWorkerForArms(void *thread_param)
    {
        ThreadParameters *cycle_thread_param = (ThreadParameters *) thread_param;
        int overrun_cnt = 0;
        cycle_thread_param->synch_flag_on = true;

        int64 toff = 0;
        double period = DRIVER_SYNCH_TIME * 1e+6; // ns
        struct timespec tic;
        clock_gettime(CLOCK_REALTIME, &tic);
        timespecInc(&tic, period);

        while(!cycle_thread_param->stop_flag)
        {
            double start_time = now();

            double start_send_receive = now();
            {
                boost::mutex::scoped_lock lock(cycle_thread_param->mutex);
                cycle_thread_param->sent = cycle_thread_param->soem.ec_send_processdata();
                cycle_thread_param->wkc = cycle_thread_param->soem.ec_receive_processdata(EC_TIMEOUTRET_2);
            }
            pthread_cond_signal(&cycle_thread_param->cond);
            if(!cycle_thread_param->process_data)
                if(cycle_thread_param->wkc >= cycle_thread_param->expected_wkc)
                    cycle_thread_param->process_data = true;
            
            double end_send_receive = now();
            cycle_thread_param->etc_stats.current_send_receive_period = end_send_receive - start_send_receive;
            cycle_thread_param->etc_stats.ec_acc(cycle_thread_param->etc_stats.current_send_receive_period);

            //check overrun
            struct timespec before;
            clock_gettime(CLOCK_REALTIME, &before);
            double overrun_time = (before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND) - (tic.tv_sec + double(tic.tv_nsec) / NSEC_PER_SECOND);
            if(overrun_time > 0.0)
            {
                overrun_cnt++;
                printf(COLOR_RED "[EtherCatManager] Warning: Cycle overrun %d times! Overrun time: %.3f ms\n" COLOR_RESET, overrun_cnt, overrun_time * 1e+3);
                fflush(stdout);
				cycle_thread_param->etc_stats.overrun_loop_sec = overrun_time;
				++cycle_thread_param->etc_stats.overruns;                
            }
            if(overrun_time > 1){
                cycle_thread_param->stop_flag = true;
            }

            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tic, NULL);

            ec_sync(cycle_thread_param->soem.ec_DCtime, period, &toff);

            timespecInc(&tic, period + toff);

            double end_time = now();
            cycle_thread_param->etc_stats.current_ethercat_loop = end_time - start_time;
        }

    }

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

    bool EthercatManager::initialize(bool &bQuit, std::vector<int> slaveIds)
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

        if(initSoem(bQuit, slaveIds))
        {
            thread_parameter = new ThreadParameters(iomap_mutex_, soem, slaveIds, stop_flag, port_id,
                                                    cond_, cond_lock_, synch_flag_on_, etc_stats, driver_infos_);

            // set expected WKC in thread parameter after construction
            int expected_wkc_local = (soem.ec_group[0].outputsWKC * 2) + soem.ec_group[0].inputsWKC;
            if(port_id == 1){
                thread_parameter->expected_wkc = expected_wkc_local;
                osal_thread_create_rt(&p_cycle_thread_, STACK64 * 100, (void *)&CycleWorkerForArms, (void *)thread_parameter);
                osal_thread_create(&p_handle_error_thread_, STACK64 * 200, (void *)&handleErrors, (void *)thread_parameter);
            }
            
            return true;
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

    bool EthercatManager::initSoem(bool &bQuit, std::vector<int> slaveIds)
    {
        (void)bQuit;
        (void)slaveIds;
        // Placeholder init until fully implemented; return true to allow threads to start
        return true;
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
        for (auto &slave_id : slaveIds)
        {
            slave_ids.push_back(slave_id);
        }
    }

}