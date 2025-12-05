#include "igh_manager.hpp"

using namespace master;

ec_master_t        * g_master = NULL ;              // EtherCAT master instance
ec_master_state_t    g_master_state = {};           // EtherCAT master state
ec_domain_t        * g_master_domain = NULL;        // Ethercat data passing master domain
ec_domain_state_t    g_master_domain_state = {};    // EtherCAT master domain state
ec_sync_info_t     * slave_sync_info_ = NULL;            // Sync info for DC sync
struct timespec      g_sync_timer ;                 // timer for DC sync
uint32_t             g_sync_ref_counter = 0;        // reference counter for DC sync


IghManager::IghManager()
{

}

IghManager::~IghManager()
{

}

int IghManager::configMaster()
{
    g_master = erct_request_master(0);
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
    for(int i = 0; i < max_slave_num_; i++){
        ecrt_master_get_slave(g_master, i, &slave_[i].slave_info_);
    }
}

int IghManager::configSlaves()
{
    for (int i = 0; i < max_slave_num_; i++){
        slaves_[i].slave_config_ = ecrt_master_slave_config(g_master, slaves_[i].slave_info_.alias,
                                                            slaves_[i].slave_info_.position,
                                                            slaves_[i].slave_info_.vendor_id,
                                                            slaves_[i].slave_info_.product_code);
        if (!slaves_[i].slave_config_){
            printf("Failed to  configure slave ! ");
            return -1;
        }
    }
    return 0;
}

int IghManager::mapDefaultPDOs(IghSlave &slave, int position)
{
    if(slave.slave_info_.name == std::string("CS3E")){
        ec_pdo_entry_info_t cs3e_pdo_entries[] = {
            {CONTROL_WORD, 16},
            {TARGET_POSITION, 32},
            {TOUCH_PROBE_FUNCTION, 8},
    
            {ERROR_CODE, 16},
            {STATUS_WORD, 16},
            {MODE_OF_OPERATION_DISPLAY, 8},
            {ACTUAL_POSITION, 32},
            {TOUCH_PROBE_FUNCTION, 16},
            {TOUCH_PROBE_1_POSITIVE_VALUE, 32},
            {DIGITAL_INPUTS, 16},
        };
    
        ec_pdo_info_t cs3e_pdos[] = {
            {0x1600, 3, &cs3e_pdo_entries[0]}, // RxPDO
            {0x1A00, 7, &cs3e_pdo_entries[3]}, // TxPDO
        };
    
        ec_sync_info_t cs32_syncs[] = {
            {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
            {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
            {2, EC_DIR_OUTPUT, 1, cs3e_pdos + 0, EC_WD_DISABLE},
            {3, EC_DIR_INPUT, 1, cs3e_pdos + 1, EC_WD_DISABLE},
            {0xff}
        };
        slave.slave_sync_info_ = cs32_syncs;
    }
    else if (slave.slave_info_.name == std::string("2CL3")){
        ec_pdo_entry_info_t d403t_pdo_rx_entries [] = {
            {CONTROL_WORD, 16},
            {TARGET_POSITION, 32},
            {TOUCH_PROBE_FUNCTION, 8},
    
            {CONTROL_WORD_2, 16},
            {TARGET_POSITION_2, 32},
            {TOUCH_PROBE_FUNCTION_2, 8},
        };

        ec_pdo_entry_info_t d403t_pdo_tx_entries [] ={
            {ERROR_CODE, 16},
            {STATUS_WORD, 16},
            {MODE_OF_OPERATION_DISPLAY, 8},
            {ACTUAL_POSITION, 32},
            {TOUCH_PROBE_FUNCTION, 16},
            {TOUCH_PROBE_1_POSITIVE_VALUE, 32},
            {DIGITAL_INPUTS, 16},
    
            {ERROR_CODE_2, 16},
            {STATUS_WORD_2, 16},
            {MODE_OF_OPERATION_DISPLAY_2, 8},
            {ACTUAL_POSITION_2, 32},
            {TOUCH_PROBE_FUNCTION_2, 16},
            {TOUCH_PROBE_1_POSITIVE_VALUE_2, 32},
            {DIGITAL_INPUTS_2, 16},
        };
        ec_pdo_info_t d403t_pdos [] = {
            {0x1600, 3, d403t_pdo_rx_entries + 0},
            {0x1610, 3, d403t_pdo_tx_entries + 3},
            {0x1A00, 7, d403t_pdo_tx_entries + 0},
            {0x1A10, 7, d403t_pdo_tx_entries + 7},
        };
    
        ec_sync_info_t d403t_syncs [5] = {
            {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
            {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
            {2, EC_DIR_OUTPUT, 2, d403t_pdos + 0, EC_WD_DISABLE},
            {3, EC_DIR_INPUT, 2, d403t_pdos + 2, EC_WD_DISABLE},
            {0xff}
        };
        slave.slave_sync_info_ = d403t_syncs;
    }
    
    if(ecrt_slave_config_pdos(slave.slave_config_, EC_END, slave.slave_sync_info_)){
        printf("Failed to map PDOs for slave at position %d\n", position);
        return -1;
    }

    if(slave.slave_info_.name == std::string("CS3E")){
        slave.offset_.actual_pos = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, ACTUAL_POSITION, g_master_domain, NULL);
        slave.offset_.target_pos = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, TARGET_POSITION, g_master_domain, NULL);
        slave.offset_.control_word = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, CONTROL_WORD, g_master_domain, NULL);
        slave.offset_.status_word = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, STATUS_WORD, g_master_domain, NULL);
        slave.offset_.error_code = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, ERROR_CODE, g_master_domain, NULL);
        slave.offset_.mode_of_operation_display = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, MODE_OF_OPERATION_DISPLAY, g_master_domain, NULL);

        if(slave.offset_.actual_pos == 0 ||
           slave.offset_.target_pos == 0 ||
           slave.offset_.control_word == 0 ||
           slave.offset_.status_word == 0 ||
           slave.offset_.error_code == 0 ||
           slave.offset_.mode_of_operation_display == 0){
            printf("Failed to register PDO entries for slave at position %d\n", position);
            return -1;
        }
    }
    else if (slave.slave_info_.name == std::string("2CL3")){
        slave.offset_.actual_pos = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, ACTUAL_POSITION, g_master_domain, NULL);
        slave.offset_.target_pos = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, TARGET_POSITION, g_master_domain, NULL);
        slave.offset_.control_word = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, CONTROL_WORD, g_master_domain, NULL);
        slave.offset_.status_word = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, STATUS_WORD, g_master_domain, NULL);
        slave.offset_.error_code = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, ERROR_CODE, g_master_domain, NULL);
        slave.offset_.mode_of_operation_display = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, MODE_OF_OPERATION_DISPLAY, g_master_domain, NULL);

        slave.offset_.actual_pos_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, ACTUAL_POSITION_2, g_master_domain, NULL);
        slave.offset_.target_pos_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, TARGET_POSITION_2, g_master_domain, NULL);
        slave.offset_.control_word_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, CONTROL_WORD_2, g_master_domain, NULL);
        slave.offset_.status_word_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, STATUS_WORD_2, g_master_domain, NULL);
        slave.offset_.error_code_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, ERROR_CODE_2, g_master_domain, NULL);
        slave.offset_.mode_of_operation_display_2 = ecrt_slave_config_reg_pdo_entry(slave.slave_config_, MODE_OF_OPERATION_DISPLAY_2, g_master_domain, NULL);
        if(slave.offset_.actual_pos == 0 ||
           slave.offset_.target_pos == 0 ||
           slave.offset_.control_word == 0 ||
           slave.offset_.status_word == 0 ||
           slave.offset_.error_code == 0 ||
           slave.offset_.mode_of_operation_display == 0 ||
           slave.offset_.actual_pos_2 == 0 ||
           slave.offset_.target_pos_2 == 0 ||
           slave.offset_.control_word_2 == 0 ||
           slave.offset_.status_word_2 == 0 ||
           slave.offset_.error_code_2 == 0 ||
           slave.offset_.mode_of_operation_display_2 == 0){
            printf("Failed to register PDO entries for slave at position %d\n", position);
            return -1;
        }
    }

    return 0;
}

void IghManager::configDcSyncDefault()
{
    for(int i = 0; i < max_slave_num_; i++){

    }
}
