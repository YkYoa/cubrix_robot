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
                bool stop_flag_on = false;

                // static int count = 0;
                // if(count++ % 100 == 0){
                //     int32_t operation_mode = readSDO<int32_t>(1, 0x6060, 0x00, error_thread_param->soem);
                //     uint16_t status_word = readSDO<uint16_t>(1, 0x6041, 0x00, error_thread_param->soem);
                //     printf(COLOR_RED "[EtherCatManager] slave %d () op mode: 0x%04X, status word: 0x%04X, error code: 0x%04X" COLOR_RESET "\n",
                //            1, operation_mode, status_word, error_code);
                // }
                
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
        ethercat_config_ = ar_common::readYamlFile(yamlpath);

        if (!ethercat_config_["port_info"] || !ethercat_config_["port_info"]["port_name"])
        {
            printf(COLOR_RED "Error: Invalid configuration - missing port_info/port_name\n" COLOR_RESET);
            return false;
        }

        p_cycle_thread_ = 0;
        p_handle_error_thread_ = 0;
        stop_flag = false;

        auto max_slaves_it = std::max_element(slaveIds.begin(), slaveIds.end());
        if(max_slaves_it != slaveIds.end()){
            max_slave_count = *max_slaves_it;
        }

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

    bool EthercatManager::readFromYamlFile(int slave_no)
    {
        DriverInfo driver_info;
        DriverInfo::LeadshineDriveData driver_data;

        driver_data.com_port = ethercat_config_["port_info"]["port_name"].as<std::string>();
        driver_data.control_mode = ethercat_config_["driver_info"]["driver_mode"].as<int>();

        std::string drive_key = "drive_" + std::to_string(slave_no);

        if (ethercat_config_["drives"][drive_key])
        {
            const auto &drive_node = ethercat_config_["drives"][drive_key];

            if (drive_node["driver_name"])
            {
                driver_info.driver_type = drive_node["driver_name"].as<std::string>();
                RCLCPP_INFO(rclcpp::get_logger("EtherCatManager"),
                            "Slave %d driver_name: %s",
                            slave_no, driver_info.driver_type.c_str());
                driver_info.is_dual_driver = drive_node["is_dual_axis"] ? drive_node["is_dual_axis"].as<bool>() : false;
            }
        }

        if(driver_info.driver_type != soem.ec_slave[slave_no].name) {
			printf(COLOR_YELLOW "WARNING the driver %d type is different from config file \n", slave_no);
			std::cout << "driver type: " << soem.ec_slave[slave_no].name << "; expected: " << driver_info.driver_type << COLOR_RESET
					  << std::endl;
			driver_info.driver_type = soem.ec_slave[slave_no].name;
            return false;
		}

		driver_infos_.insert(std::make_pair(slave_no, driver_info));

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
            soem.ec_slave[0].state = EC_STATE_INIT;

            soem.ec_writestate(0);

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
        printf(COLOR_YELLOW "\n[%s] Initializing Soem EtherCatManager[%d]..." COLOR_RESET "\n",
               ar_utils::getCurrentTime(false, false).c_str(), port_id);           
        const static unsigned MAX_BUFF_SIZE = 1024;

        std::string ifName = ethercat_config_["port_info"]["port_name"].as<std::string>();
        // char buffer[MAX_BUFF_SIZE];
        // if(ifName.size() >= sizeof(buffer - 1)){
        //     printf(COLOR_RED "Error: Interface name is too long: %s" COLOR_RESET "\n", ifName.c_str());
        //     return false;
        // }
        // std::strncpy(buffer, ifName.c_str(), sizeof(buffer) - 1);

        if(!soem.ec_init(ifName.c_str())){
            printf(COLOR_RED "Error: Failed to initialize EtherCAT on interface " COLOR_RESET "\n");
            return false;
        }

        // wait for slave 
        int slaveCount = -INT_MAX;

        while(true){
            // find and auto-config slave 
            soem.ec_config_init(FALSE);
            if(slaveCount != soem.ec_slavecount){
                if(soem.ec_slavecount >= max_slave_count){
                    printf(COLOR_YELLOW "[EtherCatManager] Detected %d slaves" COLOR_RESET "\n", soem.ec_slavecount);
                    break;
                }
                slaveCount = max_slave_count - soem.ec_slavecount;
				printf(COLOR_RED "%s Ethercat port %d: Waiting for %d slave%s (expected: %d; found %d)..." COLOR_RESET "\n",
					   ar_utils::getCurrentTime().c_str(), port_id, slaveCount, slaveCount > 1 ? "s" : "", max_slave_count,
					   soem.ec_slavecount);
				fflush(stdout);
				// usleep(1000000);

                slaveCount = soem.ec_slavecount;
				if(slaveCount > 0) {
					soem.ec_slave[0].state = EC_STATE_INIT;
					soem.ec_writestate(0);
				}
            }else
                usleep(1000000);
            if(bQuit)
                return false;
        }

        printf(COLOR_YELLOW "%s SOEM found %d slaves and configured %ld" COLOR_RESET "\n", ar_utils::getCurrentTime().c_str(),
               soem.ec_slavecount, slaveIds.size());
        fflush(stdout);

        driver_count = 0;

        for(auto slave_id : slaveIds){
            printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)soem.ec_slave[slave_id].eep_man, (int)soem.ec_slave[slave_id].eep_id,
                   (int)soem.ec_slave[slave_id].eep_rev);
            driver_count++;
        }
    	printf(COLOR_YELLOW "%s Found %d Drivers" COLOR_RESET "\n", ar_utils::getCurrentTime().c_str(), driver_count);
		fflush(stdout);


        if(soem.ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_PRE_OP) {
			printf(COLOR_RED "%s Could not set EC_STATE_PRE_OP" COLOR_RESET "\n", ar_utils::getCurrentTime().c_str());
			fflush(stdout);
			return false;
		}

        // Pair driver 
        for(auto slave_id : slaveIds){
            readFromYamlFile(slave_id);
        }

        // config PDO 
        // for(auto slave_id : slaveIds) {
		// 	configPDOProcess(slave_id);
        // }


        //Config IO map
        int io_map_size = soem.ec_config_map(&iomap_);
        std::cout << COLOR_YELLOW "[EtherCatManager] IOMap size: " << io_map_size << COLOR_RESET << std::endl;
        
        //locate dc slaves
        soem.ec_configdc();

        uint32_t cycle_time = DRIVER_SYNCH_TIME * 1e+6; // ns
        printf("  DC cycle time: %u ns (%d ms)\n", cycle_time, DRIVER_SYNCH_TIME);
        for (auto slave_id : slaveIds) {
            if (soem.ec_slave[slave_id].hasdc) {
                // SYNC0: activate, cycle time, no phase shift
                soem.ec_dcsync0(slave_id, TRUE, cycle_time, 0);
                printf("  Slave %d: SYNC0 configured\n", slave_id);
            }

            uint8_t sync_type = 2;
            soem.ec_SDOwrite(slave_id, 0x1c32, 0x01, FALSE, sizeof(sync_type), &sync_type, EC_TIMEOUTRXM);
        }

        // '0' here addresses all slaves
		if(soem.ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_SAFE_OP) {
			printf("Could not set EC_STATE_SAFE_OP\n");
			fflush(stdout);
			return false;
		}

        soem.ec_slave[0].state = EC_STATE_OPERATIONAL;
		soem.ec_send_processdata();
		soem.ec_receive_processdata(EC_TIMEOUTRET_8);

		soem.ec_writestate(0);
		int chk = 200;
		do {
			soem.ec_send_processdata();
			soem.ec_receive_processdata(EC_TIMEOUTRET_8);

			soem.ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);	 // 50 ms wait for state check
		} while(chk-- && (soem.ec_slave[0].state != EC_STATE_OPERATIONAL));

		soem.ec_readstate();
		for(auto slave_id : slaveIds) {
			printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n", slave_id,
				   soem.ec_slave[slave_id].name, soem.ec_slave[slave_id].Obits, soem.ec_slave[slave_id].Ibits,
				   soem.ec_slave[slave_id].state, soem.ec_slave[slave_id].pdelay, soem.ec_slave[slave_id].hasdc);
			if(soem.ec_slave[slave_id].hasdc)
				printf(" DCParentport:%d\n", soem.ec_slave[slave_id].parentport);
			printf(" Activeports:%d.%d.%d.%d\n", (soem.ec_slave[slave_id].activeports & 0x01) > 0,
				   (soem.ec_slave[slave_id].activeports & 0x02) > 0, (soem.ec_slave[slave_id].activeports & 0x04) > 0,
				   (soem.ec_slave[slave_id].activeports & 0x08) > 0);
			printf(" Configured address: %4.4x\n", soem.ec_slave[slave_id].configadr);

		}
		fflush(stdout);

        // check PDO sync mode, cycle time, sync0 cycle time, encoder res
        for(auto slave_id : slaveIds){
            int ret = 0, l;
            uint16_t sync_mode;
            uint32_t cycle_time;
            uint32_t minimum_cycle_time;
            uint32_t sync0_cycle_time;
            uint16_t sync_type_support;

            l = sizeof(sync_mode);
            ret += soem.ec_SDOread(slave_id, 0x1c32, 0x01, FALSE, &l, &sync_mode, EC_TIMEOUTRXM);
			l = sizeof(cycle_time);
			ret += soem.ec_SDOread(slave_id, 0x1c32, 0x02, FALSE, &l, &cycle_time, EC_TIMEOUTRXM);
			l = sizeof(minimum_cycle_time);
			ret += soem.ec_SDOread(slave_id, 0x1c32, 0x05, FALSE, &l, &minimum_cycle_time, EC_TIMEOUTRXM);
			l = sizeof(sync0_cycle_time);
			ret += soem.ec_SDOread(slave_id, 0x1c32, 0x0a, FALSE, &l, &sync0_cycle_time, EC_TIMEOUTRXM);
            l = sizeof(sync_type_support);
            ret += soem.ec_SDOread(slave_id, 0x1c32, 0x04, FALSE, &l, &sync_type_support, EC_TIMEOUTRXM);
			printf("PDO syncmode %04x, cycle time %d ns (min %d), sync0 cycle time %d ns, sync type sp %d, ret = %d\n", sync_mode, cycle_time,
				   minimum_cycle_time, sync0_cycle_time, sync_type_support, ret);

        }

        printf(COLOR_YELLOW "\n%s Finished etherCAT master[%d] configured successfully" COLOR_RESET "\n",
			   ar_utils::getCurrentTime().c_str(), port_id);
		fflush(stdout);


        return true;
    }

    DriverInfo EthercatManager::getDriverInfo(int slave_no) const
	{
		auto it = driver_infos_.find(slave_no);
		if(it != driver_infos_.end())
			return it->second;
		else {
			std::cout << "Driver not found or initialized";
			return DriverInfo();
		}
	}


    void EthercatManager::configPDOProcess(int slave_num)
    {
        DriverInfo driver_info = getDriverInfo(slave_num);
        DriverInfo::LeadshineDriveData driver_data;

        printf(COLOR_BLUE "[EthercatManager] Config PDO for driver %s \n" COLOR_RESET ,driver_info.driver_type.c_str());

        if (driver_data.control_mode == 0)
        {

            if (driver_info.is_dual_driver == true)
                configPDODualCyclic(slave_num, leadshine_param_ptr);
            else
                configPDOCyclicPosition(slave_num, leadshine_param_ptr);
        }
        else if (driver_data.control_mode == 1)
        {
            configPDOProfilePosition(slave_num, leadshine_param_ptr);
        }
    }

    void EthercatManager::configPDOCyclicPosition(int slave_num, std::shared_ptr<LeadshineParameters> leadshine_param_ptr)
    {
        // Slave info init
		int ret = 0, l;
		uint8_t num_entries;
		l = sizeof(num_entries);
		ret += soem.ec_SDOread(slave_num, leadshine_param_ptr->RXPDO1.index, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
		printf("len 1 = %d\n", num_entries);
		//------------------------ RPDO Mapping Start ------------------------------
		uint32_t mapping;
		printf("RPDO start = %d\n", ret);
		num_entries = 0;
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x00, FALSE, sizeof(num_entries), &num_entries,
								EC_TIMEOUTRXM);
		printf("RPDO debug = %d\n", ret);
		//  default
		// add control word 6040
		mapping = leadshine_param_ptr->CONTROL_WORD.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x01, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		printf("controlword debug = %d\n", ret);
            
		// add target position 6092 (Leadshine specific)
		mapping = leadshine_param_ptr->VL_TARGET_VELOCITY.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x02, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		printf("target position (6092) debug = %d\n", ret);
        
        // add touch probe function 60b8
		mapping = leadshine_param_ptr->TOUCH_PROBE_FUNCTION.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x03, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		printf("touch probe function debug = %d\n", ret);

        num_entries = 3;
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x00, FALSE, sizeof(num_entries), &num_entries,
								EC_TIMEOUTRXM);


        printf("RPDO end = %d\n", ret);

		//------------------------ RPDO Mapping End ------------------------------

		//------------------------ TPDO Mapping Start ------------------------------

		ret += soem.ec_SDOread(slave_num, leadshine_param_ptr->TXPDO1.index, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
		// printf("len 2 = %d\n", num_entries);
		num_entries = 0;
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->TXPDO1.index, 0x00, FALSE, sizeof(num_entries), &num_entries,
								EC_TIMEOUTRXM);
		// add Error code 603F
		mapping = leadshine_param_ptr->ERROR_CODE.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->TXPDO1.index, 0x01, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		// add Statusword 6041
		mapping = leadshine_param_ptr->STATUS_WORD.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->TXPDO1.index, 0x02, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		// add Modes of operation display
		mapping = leadshine_param_ptr->MODE_OF_OPERATION_DISPLAY.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->TXPDO1.index, 0x03, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		// add Position actual value 6064
		mapping = leadshine_param_ptr->ACTUAL_POSITION.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->TXPDO1.index, 0x04, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		// add Actual velocity value 606C
		mapping = leadshine_param_ptr->TOUCH_PROBE_STATUS.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->TXPDO1.index, 0x05, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		// add Actual motor torque value 6077
		mapping = leadshine_param_ptr->TOUCH_PROBE_1_POSITION_VALUE.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->TXPDO1.index, 0x06, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		// add Digital inputs 60FD
		mapping = leadshine_param_ptr->DIGITAL_INPUTS.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->TXPDO1.index, 0x07, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		// printf("TPD set num = %d\n", ret);
		num_entries = 7;
		l			= sizeof(num_entries);
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->TXPDO1.index, 0x00, FALSE, sizeof(num_entries), &num_entries,
								EC_TIMEOUTRXM);
		// ret += soem.ec_SDOread(slave_num, leadshine_param_ptr->TXPDO1.index, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
		// printf("len = %d\n", num_entries);
		// printf("TPD ret = %d\n", ret);

		// SYNC PDO mapping
		uint8_t num_pdo;
		// set 0 change PDO mapping index
		num_pdo = 0;
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->SYNC_MANAGER_2_PDO.index, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
		// set to default PDO mapping 4
		uint16_t idx_rxpdo = leadshine_param_ptr->RXPDO1.index;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->SYNC_MANAGER_2_PDO.index, 0x01, FALSE, sizeof(idx_rxpdo), &idx_rxpdo, EC_TIMEOUTRXM);
		// set number of assigned PDOs
		num_pdo = 1;
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->SYNC_MANAGER_2_PDO.index, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
		// printf("RxPDO mapping object index %d = %04x ret=%d\n", slave_num, idx_rxpdo, ret);

		// set 0 change PDO mapping index
		num_pdo = 0;
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->SYNC_MANAGER_3_PDO.index, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
		// set to default PDO mapping 4
		uint16_t idx_txpdo = leadshine_param_ptr->TXPDO1.index;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->SYNC_MANAGER_3_PDO.index, 0x01, FALSE, sizeof(idx_txpdo), &idx_txpdo, EC_TIMEOUTRXM);
		// set number of assigned PDOs
		num_pdo = 1;
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->SYNC_MANAGER_3_PDO.index, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
		// printf("TxPDO mapping object index %d = %04x ret=%d\n", slave_num, idx_txpdo, ret);

    }

    void EthercatManager::configPDOProfilePosition(int slave_num, std::shared_ptr<LeadshineParameters> leadshine_param_ptr)
    {   
        // later
    }

    void EthercatManager::configPDODualCyclic(int slave_num, std::shared_ptr<LeadshineParameters> leadshine_param_ptr)
    {
        // Slave info init
		int ret = 0, l;
		uint8_t num_entries;
		l = sizeof(num_entries);
		ret += soem.ec_SDOread(slave_num, leadshine_param_ptr->RXPDO1.index, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
		printf("len 1 = %d\n", num_entries);
		//------------------------ RPDO Mapping Start ------------------------------
		uint32_t mapping;
		printf("RPDO start = %d\n", ret);
		num_entries = 0;
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x00, FALSE, sizeof(num_entries), &num_entries,
								EC_TIMEOUTRXM);
		printf("RPDO debug = %d\n", ret);
		//  default
		// add control word 6040
		mapping = leadshine_param_ptr->CONTROL_WORD.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x01, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		printf("controlword debug = %d\n", ret);
            
		// add target position 6092 (Leadshine specific)
		mapping = leadshine_param_ptr->VL_TARGET_VELOCITY.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x02, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		printf("target position (6092) debug = %d\n", ret);
        
        // add touch probe function 60b8
		mapping = leadshine_param_ptr->TOUCH_PROBE_FUNCTION.address;
		ret +=
			soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x03, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
		printf("touch probe function debug = %d\n", ret);

        num_entries = 3;
		ret += soem.ec_SDOwrite(slave_num, leadshine_param_ptr->RXPDO1.index, 0x00, FALSE, sizeof(num_entries), &num_entries,
								EC_TIMEOUTRXM);


        printf("RPDO end = %d\n", ret);

    }

    int EthercatManager::getInputBits(int slave_no) const
	{
		if(slave_no > max_slave_count) {
			printf(COLOR_RED "%s ERROR : slave_no(%d) is larger than max_slave_id configured(%d)" COLOR_RESET "\n",
				   ar_utils::getCurrentTime().c_str(), slave_no, max_slave_count);
			fflush(stdout);
			exit(1);
		}
		return soem.ec_slave[slave_no].Ibits;
	}

    int EthercatManager::getOutputBits(int slave_no) const
    {
        if(slave_no > max_slave_count) {
			printf(COLOR_RED "%s ERROR : slave_no(%d) is larger than max_slave_id configured(%d)" COLOR_RESET "\n",
				   ar_utils::getCurrentTime().c_str(), slave_no, max_slave_count);
			fflush(stdout);
			exit(1);
		}
		return soem.ec_slave[slave_no].Obits;

    }

	void EthercatManager::write(int slave_no, uint8_t channel, uint8_t value)
	{
		boost::mutex::scoped_lock lock(iomap_mutex_);
		soem.ec_slave[slave_no].outputs[channel] = value;
	}

    uint8_t EthercatManager::readOutput(int slave_no, uint8_t channel) const
	{
		boost::mutex::scoped_lock lock(iomap_mutex_);
		if(slave_no > max_slave_count) {
			printf(COLOR_RED "%s ERROR : slave_no(%d) is larger than max_slave_id configured(%d)" COLOR_RESET "\n",
				   ar_utils::getCurrentTime().c_str(), slave_no, max_slave_count);
			fflush(stdout);
			exit(1);
		}
		if(channel * 8 >= soem.ec_slave[slave_no].Obits) {
			printf(COLOR_RED "%s ERROR : channel(%d) is larger that Output bits (%d)" COLOR_RESET "\n",
				   ar_utils::getCurrentTime().c_str(), channel * 8, soem.ec_slave[slave_no].Obits);
			fflush(stdout);
			exit(1);
		}
		return soem.ec_slave[slave_no].outputs[channel];
	}

    uint8_t EthercatManager::readInput(int slave_no, uint8_t channel) const
	{
		boost::mutex::scoped_lock lock(iomap_mutex_);
		if(slave_no > max_slave_count) {
			// checkErrorProcessed();
			printf(COLOR_RED "%s ERROR : slave_no(%d) is larger than max_slave_id configured(%d)" COLOR_RESET "\n",
				   ar_utils::getCurrentTime().c_str(), slave_no, max_slave_count);
			fflush(stdout);
			exit(1);
		}
		if(channel * 8 >= soem.ec_slave[slave_no].Ibits) {
			printf(COLOR_RED "%s ERROR : channel(%d) is larger than Input bits (%d)" COLOR_RESET "\n", ar_utils::getCurrentTime().c_str(),
				   channel * 8, soem.ec_slave[slave_no].Ibits);
			fflush(stdout);
			exit(1);
		}
		return soem.ec_slave[slave_no].inputs[channel];
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