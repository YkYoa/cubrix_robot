#include "igh_manager.hpp"
#include "igh_error_handler.hpp"

using namespace master;

ec_master_t        * g_master = NULL ;              // EtherCAT master instance
ec_master_state_t    g_master_state = {};           // EtherCAT master state
ec_domain_t        * g_master_domain = NULL;        // Ethercat data passing master domain
ec_domain_state_t    g_master_domain_state = {};    // EtherCAT master domain state
ec_sync_info_t     * slave_sync_info_ = NULL;            // Sync info for DC sync
struct timespec      g_sync_timer ;                 // timer for DC sync
uint32_t             g_sync_ref_counter = 0;        // reference counter for DC sync


IghManager::IghManager(pthread_cond_t& cond, pthread_mutex_t& cond_lock, boost::mutex& mutex)
    : cond_(cond), cond_lock_(cond_lock), iomap_mutex_(mutex), stop_flag_(false), num_slaves_(0), error_handler_(nullptr)
{
    // Create error handler
    error_handler_ = new IghErrorHandler(this);
}

IghManager::~IghManager()
{
    if(error_handler_) {
        delete error_handler_;
        error_handler_ = nullptr;
    }
}

int IghManager::configMaster()
{
    g_master = ecrt_request_master(0);
    if(!g_master){
        printf("Failed to get master!\n");
        return -1;
    }
    g_master_domain = ecrt_master_create_domain(g_master);
    if(!g_master_domain){
        printf("Failed to create domain!\n");
        return -1;
    }
    return 0;
}

void IghManager::getAllSlavesInfo()
{    
    ec_master_info_t master_info;
    if(ecrt_master(g_master, &master_info) != 0) {
        printf("Failed to get master info\n");
        return;
    }
    
    int num_slaves = master_info.slave_count;
    if(num_slaves > max_slave_num_) {
        num_slaves = max_slave_num_;
    }
    
    printf("Detected %d slaves on the bus\n", num_slaves);
    
    for(int i = 0; i < num_slaves; i++) {
        ec_slave_info_t slave_info;
        if(ecrt_master_get_slave(g_master, i, &slave_info) == 0) {
            slave_[i].slave_info_.alias = slave_info.alias;
            slave_[i].slave_info_.position = slave_info.position;
            slave_[i].slave_info_.vendor_id = slave_info.vendor_id;
            slave_[i].slave_info_.product_code = slave_info.product_code;
            strncpy(slave_[i].slave_info_.name, slave_info.name, sizeof(slave_[i].slave_info_.name) - 1);
            
            printf("Slave %d: %s (Vendor: 0x%08x, Product: 0x%08x)\n",
                   i, slave_info.name, slave_info.vendor_id, slave_info.product_code);
        } else {
            printf("Failed to get info for slave %d\n", i);
        }
    }
    
    printf("Configured slave info for %d slaves\n", num_slaves);
    num_slaves_ = num_slaves;
}

int IghManager::configSlaves()
{
    int actual_slaves = num_slaves_;
    
    for (int i = 0; i < actual_slaves; i++){
        printf("Configuring slave %d: alias=%d, pos=%d, vendor=0x%08x, product=0x%08x\n",
               i, slave_[i].slave_info_.alias, slave_[i].slave_info_.position,
               slave_[i].slave_info_.vendor_id, slave_[i].slave_info_.product_code);
               
        slave_[i].slave_config_ = ecrt_master_slave_config(g_master, 
                                                            slave_[i].slave_info_.alias,
                                                            slave_[i].slave_info_.position,
                                                            slave_[i].slave_info_.vendor_id,
                                                            slave_[i].slave_info_.product_code);
        if (!slave_[i].slave_config_){
            printf("Failed to configure slave %d\n", i);
            perror("ecrt_master_slave_config");
            return -1;
        }
    }
    return 0;
}

int IghManager::mapDefaultPDOs(IghSlave &slave, int position)
{
    std::string slave_name(slave.slave_info_.name);
    
    // Use static arrays so they don't go out of scope
    static ec_pdo_entry_info_t cs3e_pdo_entries[] = {
        {0x6040, 0x00, 16}, // CONTROL_WORD
        {0x607A, 0x00, 32}, // TARGET_POSITION
        {0x60B8, 0x00, 16}, // TOUCH_PROBE_FUNCTION

        {0x603F, 0x00, 16}, // ERROR_CODE
        {0x6041, 0x00, 16}, // STATUS_WORD
        {0x6061, 0x00, 8},  // MODE_OF_OPERATION_DISPLAY
        {0x6064, 0x00, 32}, // ACTUAL_POSITION
        {0x60B9, 0x00, 16}, // TOUCH_PROBE_STATUS
        {0x60BA, 0x00, 32}, // TOUCH_PROBE_1_POSITIVE_VALUE
        {0x60FD, 0x00, 32}, // DIGITAL_INPUTS (32-bit fix)
    };

    static ec_pdo_info_t cs3e_pdos[] = {
        {0x1600, 3, &cs3e_pdo_entries[0]}, // RxPDO
        {0x1A00, 7, &cs3e_pdo_entries[3]}, // TxPDO
    };

    static ec_sync_info_t cs3e_syncs[] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, cs3e_pdos + 0, EC_WD_ENABLE},  // Enable DC for outputs
        {3, EC_DIR_INPUT, 1, cs3e_pdos + 1, EC_WD_DISABLE},   // Enable DC for inputs
        {0xff}
    };
    
    static ec_pdo_entry_info_t d403t_pdo_rx_entries[] = {
        {0x6040, 0x00, 16}, // CONTROL_WORD
        {0x607A, 0x00, 32}, // TARGET_POSITION
        {0x60B8, 0x00, 16}, // TOUCH_PROBE_FUNCTION

        {0x6840, 0x00, 16}, // CONTROL_WORD_2
        {0x687A, 0x00, 32}, // TARGET_POSITION_2
        {0x68B8, 0x00, 16}, // TOUCH_PROBE_FUNCTION_2
    };

    static ec_pdo_entry_info_t d403t_pdo_tx_entries[] = {
        {0x603F, 0x00, 16}, // ERROR_CODE
        {0x6041, 0x00, 16}, // STATUS_WORD
        {0x6061, 0x00, 8},  // MODE_OF_OPERATION_DISPLAY
        {0x6064, 0x00, 32}, // ACTUAL_POSITION
        {0x60B9, 0x00, 16}, // TOUCH_PROBE_STATUS
        {0x60BA, 0x00, 32}, // TOUCH_PROBE_1_POSITIVE_VALUE
        {0x60FD, 0x00, 32}, // DIGITAL_INPUTS

        {0x683F, 0x00, 16}, // ERROR_CODE_2
        {0x6841, 0x00, 16}, // STATUS_WORD_2
        {0x6861, 0x00, 8},  // MODE_OF_OPERATION_DISPLAY_2
        {0x6864, 0x00, 32}, // ACTUAL_POSITION_2
        {0x68B9, 0x00, 16}, // TOUCH_PROBE_STATUS_2
        {0x68BA, 0x00, 32}, // TOUCH_PROBE_1_POSITIVE_VALUE_2
        {0x68FD, 0x00, 32}, // DIGITAL_INPUTS_2
    };
    
    static ec_pdo_info_t d403t_pdos[] = {
        {0x1600, 3, d403t_pdo_rx_entries + 0},
        {0x1610, 3, d403t_pdo_rx_entries + 3},
        {0x1A00, 7, d403t_pdo_tx_entries + 0},
        {0x1A10, 7, d403t_pdo_tx_entries + 7},
    };

    static ec_sync_info_t d403t_syncs[] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 2, d403t_pdos + 0, EC_WD_ENABLE},  // Enable DC for outputs
        {3, EC_DIR_INPUT, 2, d403t_pdos + 2, EC_WD_DISABLE},   // Enable DC for inputs
        {0xff}
    };
    
    // Select appropriate sync info based on slave type
    ec_sync_info_t *sync_info = NULL;
    if(slave_name.find("CS3E") != std::string::npos) {
        sync_info = cs3e_syncs;
        printf("Configuring CS3E PDOs for slave %d\n", position);
    }
    else if(slave_name.find("2CL3") != std::string::npos) {
        sync_info = d403t_syncs;
        printf("Configuring 2CL3 PDOs for slave %d\n", position);
    }
    else {
        printf("Unknown slave type: %s\n", slave_name.c_str());
        return -1;
    }
    
    // Configure PDOs
    if(ecrt_slave_config_pdos(slave.slave_config_, EC_END, sync_info)) {
        printf("Failed to config PDOs for slave %d\n", position);
        perror("ecrt_slave_config_pdos");
        return -1;
    }

    // Register PDO entries for CS3E
    if(slave_name.find("CS3E") != std::string::npos) {
        slave.offset_.control_word = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x6040, 0x00, g_master_domain, NULL);
        slave.offset_.target_pos = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x607A, 0x00, g_master_domain, NULL);
        slave.offset_.touch_probe_function = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x60B8, 0x00, g_master_domain, NULL);

        slave.offset_.error_code = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x603F, 0x00, g_master_domain, NULL);
        slave.offset_.status_word = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x6041, 0x00, g_master_domain, NULL);
        slave.offset_.mode_of_operation_display = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x6061, 0x00, g_master_domain, NULL);
        slave.offset_.actual_pos = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x6064, 0x00, g_master_domain, NULL);
        slave.offset_.touch_probe_status = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x60B9, 0x00, g_master_domain, NULL);
        slave.offset_.touch_probe_1_positive_value = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x60BA, 0x00, g_master_domain, NULL);
        slave.offset_.digital_input = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x60FD, 0x00, g_master_domain, NULL);

        printf("Slave %d CS3E PDO offsets registered: ctrl=%u, tpos=%u, err=%u, stat=%u, mode=%u, apos=%u\n",
               position,
               slave.offset_.control_word,
               slave.offset_.target_pos,
               slave.offset_.error_code,
               slave.offset_.status_word,
               slave.offset_.mode_of_operation_display,
               slave.offset_.actual_pos);
        
        // Store base offsets (first entry of each PDO type)
        slave.base_output_offset_ = slave.offset_.control_word;
        slave.base_input_offset_ = slave.offset_.error_code;
        
        printf("Successfully registered CS3E PDO entries for slave %d\n", position);
    }
    // Register PDO entries for 2CL3
    else if(slave_name.find("2CL3") != std::string::npos) {
        slave.offset_.control_word = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x6040, 0x00, g_master_domain, NULL);
        slave.offset_.target_pos = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x607A, 0x00, g_master_domain, NULL);
        slave.offset_.touch_probe_function = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x60B8, 0x00, g_master_domain, NULL);

        slave.offset_.error_code = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x603F, 0x00, g_master_domain, NULL);
        slave.offset_.status_word = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x6041, 0x00, g_master_domain, NULL);
        slave.offset_.mode_of_operation_display = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x6061, 0x00, g_master_domain, NULL);
        slave.offset_.actual_pos = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x6064, 0x00, g_master_domain, NULL);
        slave.offset_.touch_probe_status = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x60B9, 0x00, g_master_domain, NULL);
        slave.offset_.touch_probe_1_positive_value = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x60BA, 0x00, g_master_domain, NULL);
        slave.offset_.digital_input = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, 0x60FD, 0x00, g_master_domain, NULL);

        slave.offset_.control_word_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, CONTROL_WORD_2, g_master_domain, NULL);
        slave.offset_.target_pos_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, TARGET_POSITION_2, g_master_domain, NULL);
        slave.offset_.touch_probe_function_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, TOUCH_PROBE_FUNCTION_2, g_master_domain, NULL);

        slave.offset_.error_code_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, ERROR_CODE_2, g_master_domain, NULL);
        slave.offset_.status_word_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, STATUS_WORD_2, g_master_domain, NULL);
        slave.offset_.mode_of_operation_display_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, MODE_OF_OPERATION_DISPLAY_2, g_master_domain, NULL);
        slave.offset_.actual_pos_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, ACTUAL_POSITION_2, g_master_domain, NULL);
        slave.offset_.touch_probe_status_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, TOUCH_PROBE_STATUS_2, g_master_domain, NULL);
        slave.offset_.touch_probe_1_positive_value_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, TOUCH_PROBE_1_POSITIVE_VALUE_2, g_master_domain, NULL);
        slave.offset_.digital_input_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, DIGITAL_INPUTS_2, g_master_domain, NULL);

        printf("Successfully registered 2CL3 PDO entries for slave %d\n", position);
        printf("  Axis1 offsets: ctrl=%u, tpos=%u, err=%u, stat=%u, mode=%u, apos=%u\n",
               slave.offset_.control_word,
               slave.offset_.target_pos,
               slave.offset_.error_code,
               slave.offset_.status_word,
               slave.offset_.mode_of_operation_display,
               slave.offset_.actual_pos);
        printf("  Axis2 offsets: ctrl=%u, tpos=%u, err=%u, stat=%u, mode=%u, apos=%u\n",
               slave.offset_.control_word_2,
               slave.offset_.target_pos_2,
               slave.offset_.error_code_2,
               slave.offset_.status_word_2,
               slave.offset_.mode_of_operation_display_2,
               slave.offset_.actual_pos_2);
        
        // Store base offsets (first entry of each PDO type)
        slave.base_output_offset_ = slave.offset_.control_word;
        slave.base_input_offset_ = slave.offset_.error_code;
        printf(COLOR_BLUE"DEBUG: Slave %d base_output_offset_=%u, base_input_offset_=%u\n" COLOR_RESET, 
                                position, slave.base_output_offset_, slave.base_input_offset_);
    }

    return 0;
}

void IghManager::configDcSyncDefault()
{
    int actual_slaves = num_slaves_;
    for(int i = 0; i < actual_slaves; i++){
        ecrt_slave_config_dc(slave_[i].slave_config_, 0x0300, PERIOD_NS, slave_[i].sync0_shift_, 0, 0);
    }
}

int IghManager::activateMaster()
{
    if(ecrt_master_activate(g_master)){
        printf("Failed to activate master!\n");
        return -1;
    }
    return 0;
}

int IghManager::registerDomain()
{
    int actual_slaves = num_slaves_;
    for(int i = 0; i < actual_slaves; i++){
        slave_[i].slave_pdo_domain_ = ecrt_domain_data(g_master_domain);
        if(!slave_[i].slave_pdo_domain_){
            printf("Failed to get domain data for slave %d\n", i);
            return -1;
        }
    }
    return 0;
}

int IghManager::setCyclicPositionParameters()
{
    int actual_slaves = num_slaves_;
    printf("[IGH] Setting operation mode to Cyclic Synchronous Position (mode 8) for %d slaves...\n", actual_slaves);
    for(int i = 0; i < actual_slaves; i++){
        if(ecrt_slave_config_sdo8(slave_[i].slave_config_, 0x6060, 0x00, 8))
            printf(COLOR_BLUE "\n Set mode of operation to CSP Mode \n");
        if(ecrt_slave_config_sdo8(slave_[i].slave_config_, 0x60c2, 0x01, 0))
            printf("\n Set up Interpolate time \n" COLOR_RESET);
    }
    return 0;
}

int IghManager::openEthercatMaster()
{
    fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
    if(fd){
        printf( "Opening EtherCAT master...");
        std::system("cd ~; sudo ethercatctl start");
        usleep(2e6);
        fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
        if(fd){
            printf( "Error : EtherCAT device not found.");
            return -1;
            }else {
                return 0 ;
            }
    }
    return configMaster();
}

int IghManager::getNumbOfConnectedSlaves()
{
    return num_slaves_;
}

int IghManager::waitForOpMode()
{
    int actual_slaves = num_slaves_;
    int try_cnt = 0;
    int check_state_cnt = 0;
    int time_out = 20e3;
    while (g_master_state.al_states != EC_AL_STATE_OP)
    {
        if(try_cnt < time_out){
            clock_gettime(CLOCK_MONOTONIC, &g_sync_timer);
            ecrt_master_application_time(g_master, TIMESPEC2NS(g_sync_timer));

            ecrt_master_receive(g_master);
            ecrt_domain_process(g_master_domain);
            usleep(PERIOD_US);

            if(!check_state_cnt){
                checkMasterState();
                checkMasterDomainState();
                for(int i = 0; i < actual_slaves; i++){
                    slave_[i].checkSlaveConfigState();
                }
                check_state_cnt = PERIOD_US;
            }

            ecrt_domain_queue(g_master_domain);
            ecrt_master_sync_slave_clocks(g_master);
            ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(g_sync_timer));
            ecrt_master_send(g_master);

            try_cnt++;
            check_state_cnt--;
        }else{
            ecrt_master_deactivate(g_master);
            ecrt_release_master(g_master);
            return -1;
        }
    }    
    
    return 0;
}

void IghManager::deactivateMaster()
{
    if(g_master){
        ecrt_master_deactivate(g_master);
    }
}

void IghManager::releaseMaster()
{
    if(g_master){
        ecrt_release_master(g_master);
        g_master = NULL;
    }
}

int IghManager::shutdown()
{
    stop_flag_ = true;
    
    if(error_handler_) {
        error_handler_->stopErrorMonitoring();
    }
    
    if(cyclic_thread_){
        pthread_join(cyclic_thread_, NULL);
    }
    
    deactivateMaster();
    releaseMaster();

    // fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
    // if(!fd){
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down EtherCAT master...");
    //     std::system("cd ~; sudo ethercatctl stop\n");
    //     usleep(1e6);
    //     fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
    //     if(fd){
    //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"EtherCAT shut down succesfull.");
    //         return 0;
    //     }else {
    //         RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : EtherCAT shutdown error.");
    //         return -1 ;
    //     }
    // }
    
    printf("IGH EtherCAT master shut down\n");
    return 0;
}

int IghManager::checkMasterState()
{
    int actual_slaves = num_slaves_;
    ecrt_master_state(g_master, &g_master_state);
    
    if(g_master_state.slaves_responding != actual_slaves){
        printf("Warning: Only %u/%d slaves responding\n", 
               g_master_state.slaves_responding, actual_slaves);
        return -1;
    }
    
    return 0;
}

void IghManager::checkMasterDomainState()
{
    ecrt_domain_state(g_master_domain, &g_master_domain_state);
    
    if(g_master_domain_state.working_counter != g_master_domain_state.wc_state){
        printf("Domain state mismatch - working_counter: %u, wc_state: %u\n",
               g_master_domain_state.working_counter, 
               g_master_domain_state.wc_state);
    }
}

uint8_t IghManager::SDOread(SDO_data& pack)
{
    uint8_t ret = ecrt_master_sdo_upload(g_master, pack.slave_position, 
                                          pack.index, pack.subindex,
                                          (uint8_t*)&pack.data, pack.data_sz, 
                                          &pack.result_sz, &pack.error_code);
    if(ret){
        printf("Failed to read SDO: slave=%d, index=0x%04X, subindex=0x%02X, error=%u\n",
               pack.slave_position, pack.index, pack.subindex, pack.error_code);
    }
    return ret;
}

uint8_t IghManager::SDOwrite(SDO_data& pack)
{
    uint8_t ret = ecrt_master_sdo_download(g_master, pack.slave_position,
                                            pack.index, pack.subindex,
                                            (uint8_t*)&pack.data, pack.data_sz,
                                            &pack.error_code);
    if(ret){
        printf("Failed to write SDO: slave=%d, index=0x%04X, subindex=0x%02X, error=%u\n",
               pack.slave_position, pack.index, pack.subindex, pack.error_code);
    }
    return ret;
}

int IghManager::setSlaves(IghSlave slave, int position)
{
    if(position >= 0 && position < max_slave_num_){
        slave_[position] = slave;
        return 0;
    }
    return -1;
}

int IghManager::setProFilePositionParameters(ProFilePositionParm& P)
{
    printf("ProFilePositionParameters not yet implemented\n");
    return 0;
}

void IghManager::configDcSync(uint16_t assign_activated, int position)
{
    if(position >= 0 && position < max_slave_num_){
        ecrt_slave_config_dc(slave_[position].slave_config_, 
                            assign_activated, PERIOD_NS, 0, 0, 0);
    }
}

void* IghManager::cyclicThread(void* arg)
{
    IghManager* mgr = static_cast<IghManager*>(arg);
    mgr->cyclicLoop();
    return NULL;
}

void IghManager::cyclicLoop()
{
    struct timespec wakeup_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    
    printf("IGH cyclic communication thread started at 250Hz\n");
    
    uint32_t cycle_counter = 0;
    uint8_t sync_ref_counter = 0;
    
    while(!stop_flag_) {
        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= 1000000000L) {
            wakeup_time.tv_nsec -= 1000000000L;
            wakeup_time.tv_sec++;
        }
        
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
        ecrt_master_application_time(g_master, TIMESPEC2NS(wakeup_time));
        
        ecrt_master_receive(g_master);
        ecrt_domain_process(g_master_domain);
        
        pthread_cond_signal(&cond_);
        usleep(1000);
        
        if(cycle_counter % 1000 == 0 && g_master_domain_state.wc_state == EC_WC_INCOMPLETE) {
            printf("Domain WC incomplete: %u\n", g_master_domain_state.working_counter);
        }
        
        if (sync_ref_counter) {
            sync_ref_counter--;
        } else {
            sync_ref_counter = 1;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(current_time));
        }
        ecrt_master_sync_slave_clocks(g_master);
        
        {
            boost::mutex::scoped_lock lock(iomap_mutex_);
            ecrt_domain_queue(g_master_domain);
            ecrt_master_send(g_master);
        }
        
        cycle_counter++;
    }
    
    printf("\n IGH cyclic communication thread stopped \n");
}

int IghManager::startCyclicCommunication()
{
    stop_flag_ = false;
    
    pthread_attr_t attr;
    struct sched_param param;
    
    if (pthread_attr_init(&attr) != 0) {
        printf("ERROR: Failed to initialize thread attributes\n");
        return -1;
    }
    
    if (pthread_attr_setstacksize(&attr, 4096 * 64) != 0) {
        printf("ERROR: Failed to set stack size\n");
        pthread_attr_destroy(&attr);
        return -1;
    }
    
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != 0) {
        printf("WARNING: Failed to set SCHED_FIFO policy (root required). Running with normal priority.\n");
        pthread_attr_destroy(&attr);
        
        if(pthread_create(&cyclic_thread_, NULL, &IghManager::cyclicThread, this) != 0) {
            printf("ERROR: Failed to create cyclic communication thread\n");
            return -1;
        }
        printf("IGH cyclic communication started (normal priority)\n");
        return 0;
    }
    
    param.sched_priority = 98;
    if (pthread_attr_setschedparam(&attr, &param) != 0) {
        printf("ERROR: Failed to set scheduling parameters\n");
        pthread_attr_destroy(&attr);
        return -1;
    }
    
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0) {
        printf("ERROR: Failed to set inherit sched\n");
        pthread_attr_destroy(&attr);
        return -1;
    }
    
    if (pthread_create(&cyclic_thread_, &attr, &IghManager::cyclicThread, this) != 0) {
        printf("ERROR: Failed to create cyclic communication thread\n");
        pthread_attr_destroy(&attr);
        return -1;
    }
    
    pthread_attr_destroy(&attr);
    
    printf("IGH cyc lic communication started with SCHED_FIFO priority 98\n");
    
    if(error_handler_ && error_handler_->startErrorMonitoring() != 0) {
        printf("WARNING: Failed to start error monitoring thread\n");
    }
    
    return 0;
}

void IghManager::stopCyclicCommunication()
{
    if(cyclic_thread_) {
        stop_flag_ = true;
        pthread_join(cyclic_thread_, NULL);
        cyclic_thread_ = 0;
        printf("IGH cyclic communication stopped\n");
    }
}

void IghManager::write(int slave_no, uint8_t channel, uint8_t value)
{    
    boost::mutex::scoped_lock lock(iomap_mutex_);
    
    uint8_t* domain = slave_[slave_no].slave_pdo_domain_;
    if(domain) {
        unsigned int offset = slave_[slave_no].base_output_offset_ + channel;
        domain[offset] = value;
    }
}

uint8_t IghManager::readInput(int slave_no, uint8_t channel) const
{    
    uint8_t* domain = slave_[slave_no].slave_pdo_domain_;
    return domain ? domain[slave_[slave_no].base_input_offset_ + channel] : 0;
}

uint8_t IghManager::readOutput(int slave_no, uint8_t channel) const
{    
    uint8_t* domain = slave_[slave_no].slave_pdo_domain_;
    return domain ? domain[slave_[slave_no].base_output_offset_ + channel] : 0;
}

int IghManager::getInputBits(int slave_no) const
{
    if(slave_no <= 2) return 152;
    return 304; 
}

int IghManager::getOutputBits(int slave_no) const
{
    if(slave_no <= 2) return 64;
    return 128;
}

