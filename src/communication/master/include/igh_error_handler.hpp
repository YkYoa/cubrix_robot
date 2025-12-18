#pragma once

#include <pthread.h>
#include <vector>
#include <ecrt.h>

namespace master {

class IghManager;

/**
 * @brief Error handler for IGH EtherCAT master
 * 
 * Monitors slave states and attempts automatic recovery for:
 * - Slaves in SAFE_OP + ERROR state
 * - Slaves that dropped from OPERATIONAL
 * - Lost slaves (communication failure)
 * 
 * Based on SOEM's handleErrors implementation.
 */
class IghErrorHandler {
public:
    /**
     * @brief Construct error handler
     * @param mgr Pointer to IghManager instance
     */
    explicit IghErrorHandler(IghManager* mgr);
    
    /**
     * @brief Destructor - stops monitoring if running
     */
    ~IghErrorHandler();
    
    /**
     * @brief Start error monitoring thread
     * @return 0 on success, -1 on failure
     */
    int startErrorMonitoring();
    
    /**
     * @brief Stop error monitoring thread
     */
    void stopErrorMonitoring();
    
private:
    /**
     * @brief Error state tracking for each slave
     */
    struct SlaveErrorState {
        bool is_lost;                    // Slave lost communication
        uint8_t recovery_delay_count;    // Delay before attempting recovery
        uint16_t last_al_state;          // Last known AL state
        uint32_t error_count;            // Number of errors detected
        uint32_t recovery_count;         // Number of successful recoveries
    };
    
    /**
     * @brief Thread entry point
     */
    static void* errorHandlerThread(void* arg);
    
    /**
     * @brief Main error monitoring loop
     */
    void errorHandlerLoop();
    
    /**
     * @brief Handle slave in non-operational state
     * @param slave_pos Slave position
     * @param slave_state Current slave state
     */
    void handleNonOperationalSlave(int slave_pos, const ec_slave_config_state_t& slave_state);
    
    /**
     * @brief Attempt to recover a lost slave
     * @param slave_pos Slave position
     * @return true if recovery successful
     */
    bool recoverSlave(int slave_pos);
    
    /**
     * @brief Request slave state transition
     * @param slave_pos Slave position
     * @param target_state Target AL state
     */
    void requestSlaveState(int slave_pos, ec_al_state_t target_state);
    
    IghManager* manager_;                           // Pointer to manager
    pthread_t error_thread_;                        // Error monitoring thread
    bool stop_flag_;                                // Stop flag for thread
    std::vector<SlaveErrorState> slave_states_;     // Per-slave error tracking
    
    static const int CHECK_INTERVAL_US = 20000;      // 20ms check interval (matching SOEM)
    static const uint8_t RECOVERY_DELAY_CYCLES = 50; // Wait 50 cycles (1 second) before recovery
};

} // namespace master
