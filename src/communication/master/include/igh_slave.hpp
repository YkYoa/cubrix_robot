#pragma once 


#include "igh_gobal.hpp"

class IghSlave
{
    public:
        IghSlave();
        ~IghSlave();

    int checkSlaveConfigState();

    uint32_t sync0_shift_ = 0;                     /// Sync0 shift setting 
    /// Slave configuration parameters, assign for each slave
    ec_slave_config_t       * slave_config_;
    /// Slave state handle to check if slave is online + slave state machine status
    ec_slave_config_state_t   slave_config_state_;  // Changed from pointer to value
    /// PDO config
    ec_pdo_info_t           * slave_pdo_info_;
    ec_pdo_entry_info_t     * slave_pdo_entry_info_;
    ec_sync_info_t          * slave_sync_info_;
    ec_pdo_entry_reg_t      * slave_pdo_entry_reg_ ;
    uint8_t                 * slave_pdo_domain_ ;

    int32_t                 motor_state_;
    /**
     * @brief Slave information data structure.
     *      This structure contains all information related to slave.
     *      It will be used to get slave's information from master.
     */
    ec_slave_info_t slave_info_;
    /// Offset for PDO entries to assign pdo registers.
    OffsetPDO offset_;
    /// Received data from servo drivers.
    DataReceived data_;
    
    /// Base offset where this slave's input PDO data starts in the domain
    unsigned int base_input_offset_;
    /// Base offset where this slave's output PDO data starts in the domain
    unsigned int base_output_offset_;
};