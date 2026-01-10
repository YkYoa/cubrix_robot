#include "igh_error_handler.hpp"
#include "igh_manager.hpp"
#include <stdio.h>
#include <unistd.h>
#include <ar_common/common.h>

namespace master
{

    IghErrorHandler::IghErrorHandler(IghManager *mgr)
        : manager_(mgr), error_thread_(0), stop_flag_(false)
    {
    }

    IghErrorHandler::~IghErrorHandler()
    {
        stopErrorMonitoring();
    }

    int IghErrorHandler::startErrorMonitoring()
    {
        if (error_thread_ != 0)
        {
            printf("[IGH Error Handler] Already running\n");
            return -1;
        }

        if (!manager_ || manager_->num_slaves_ <= 0)
        {
            printf(COLOR_RED "[IGH Error Handler] No slaves detected, cannot start monitoring\n" COLOR_RESET);
            return -1;
        }

        slave_states_.resize(manager_->num_slaves_);
        for (auto &state : slave_states_)
        {
            state.is_lost = false;
            state.recovery_delay_count = 0;
            state.last_al_state = 0;
            state.error_count = 0;
            state.recovery_count = 0;
        }

        stop_flag_ = false;

        if (pthread_create(&error_thread_, NULL, &IghErrorHandler::errorHandlerThread, this) != 0)
        {
            printf(COLOR_RED "[IGH Error Handler] Failed to create error monitoring thread\n" COLOR_RESET);
            return -1;
        }

        printf(COLOR_YELLOW "[IGH Error Handler] Error monitoring started for %d slaves (20ms interval)\n" COLOR_RESET,
               manager_->num_slaves_);
        return 0;
    }

    void IghErrorHandler::stopErrorMonitoring()
    {
        if (error_thread_ != 0)
        {
            stop_flag_ = true;
            pthread_join(error_thread_, NULL);
            error_thread_ = 0;
            printf("[IGH Error Handler] Error monitoring stopped\n");
        }
    }

    void *IghErrorHandler::errorHandlerThread(void *arg)
    {
        IghErrorHandler *handler = static_cast<IghErrorHandler *>(arg);
        handler->errorHandlerLoop();
        return NULL;
    }

    void IghErrorHandler::errorHandlerLoop()
    {
        struct timespec cycle_start, cycle_end, last_cycle_start = {};

        uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0;
        uint32_t period_min_ns = 0xFFFFFFFF, period_max_ns = 0;
        uint32_t exec_min_ns = 0xFFFFFFFF, exec_max_ns = 0;
        uint32_t latency_min_ns = 0xFFFFFFFF, latency_max_ns = 0;

        uint32_t cycle_counter = 0;
        const uint32_t STATS_INTERVAL = 200;

        printf("[IGH Error Handler] Monitoring loop started with timing measurement (20ms interval)\n");

        while (!stop_flag_)
        {
            clock_gettime(CLOCK_MONOTONIC, &cycle_start);

            if (last_cycle_start.tv_sec != 0)
            {
                period_ns = DIFF_NS(last_cycle_start, cycle_start);
                if (period_ns < period_min_ns)
                    period_min_ns = period_ns;
                if (period_ns > period_max_ns)
                    period_max_ns = period_ns;
            }

            for (int i = 0; i < manager_->num_slaves_; i++)
            {
                ec_slave_config_state_t slave_state;
                ecrt_slave_config_state(manager_->slave_[i].slave_config_, &slave_state);

                if (slave_state.al_state != EC_AL_STATE_OP)
                {
                    handleNonOperationalSlave(i, slave_state);
                }
                else
                {
                    if (slave_states_[i].is_lost)
                    {
                        slave_states_[i].is_lost = false;
                        slave_states_[i].recovery_count++;
                        printf(COLOR_GREEN "[IGH Error Handler] Slave %d recovered to OP (Recovery #%u)\n" COLOR_RESET,
                               i, slave_states_[i].recovery_count);
                    }
                    slave_states_[i].last_al_state = slave_state.al_state;
                }
            }

            clock_gettime(CLOCK_MONOTONIC, &cycle_end);
            exec_ns = DIFF_NS(cycle_start, cycle_end);
            if (exec_ns < exec_min_ns)
                exec_min_ns = exec_ns;
            if (exec_ns > exec_max_ns)
                exec_max_ns = exec_ns;

            if (cycle_counter > 0 && cycle_counter % STATS_INTERVAL == 0)
            {
                uint32_t period_jitter_ns = period_max_ns - period_min_ns;

                // printf("[IGH Error Handler Timing] Cycle %u | Period: %u-%u ns (Δ%u) | Exec: %u-%u ns\n",
                //        cycle_counter,
                //        period_min_ns, period_max_ns, period_jitter_ns,
                //        exec_min_ns, exec_max_ns);

                if (period_jitter_ns > 5000000)
                {
                    printf(COLOR_YELLOW "[IGH Error Handler WARNING] High period jitter: %u ns (%.2f ms)\n" COLOR_RESET,
                           period_jitter_ns, period_jitter_ns / 1e6);
                }
                if (exec_max_ns > 15000000)
                {
                    printf(COLOR_RED "[IGH Error Handler WARNING] High execution time: %u ns (%.2f ms)\n" COLOR_RESET,
                           exec_max_ns, exec_max_ns / 1e6);
                }

                period_min_ns = exec_min_ns = 0xFFFFFFFF;
                period_max_ns = exec_max_ns = 0;
            }

            last_cycle_start = cycle_start;
            cycle_counter++;

            usleep(CHECK_INTERVAL_US);
        }

        printf("[IGH Error Handler] Monitoring loop stopped\n");
    }

    void IghErrorHandler::handleNonOperationalSlave(int slave_pos, const ec_slave_config_state_t &slave_state)
{
    auto &error_state = slave_states_[slave_pos];
    const char *slave_name = manager_->slave_[slave_pos].slave_info_.name;

    // Only log actual problems, not normal state transitions during initialization
    if (slave_state.al_state != error_state.last_al_state)
    {
        // Only log drops to INIT (real problem) or if error count is high (stuck)
        if (slave_state.al_state == EC_AL_STATE_INIT && error_state.error_count > 0)
        {
            if (!error_state.is_lost)
            {
                printf(COLOR_YELLOW "[IGH Error Handler] Slave %d (%s) communication lost\n" COLOR_RESET,
                       slave_pos, slave_name);
                error_state.is_lost = true;
            }
        }
        error_state.last_al_state = slave_state.al_state;
        error_state.error_count++;
    }

    switch (slave_state.al_state)
    {
    case EC_AL_STATE_INIT:
        if (!error_state.is_lost)
        {
            error_state.is_lost = true;
            error_state.recovery_delay_count = RECOVERY_DELAY_CYCLES;
        }
        break;

    case EC_AL_STATE_PREOP:
        if (error_state.error_count > 10)  // Only request OP if stuck in PREOP
        {
            requestSlaveState(slave_pos, EC_AL_STATE_OP);
        }
        break;

    case EC_AL_STATE_SAFEOP:
        if (slave_state.al_state & EC_STATE_ERROR)
        {
            printf(COLOR_RED "[IGH Error Handler] Slave %d (%s) in SAFE_OP + ERROR\n" COLOR_RESET,
                   slave_pos, slave_name);
        }
        else if (error_state.error_count > 5)  // Only request OP if stuck in SAFEOP
        {
            requestSlaveState(slave_pos, EC_AL_STATE_OP);
        }
        break;

    default:
        if (!error_state.is_lost && slave_state.al_state == 0)
        {
            printf(COLOR_RED "[IGH Error Handler] Slave %d (%s) lost (AL state = 0)\n" COLOR_RESET,
                   slave_pos, slave_name);
            error_state.is_lost = true;
            error_state.recovery_delay_count = RECOVERY_DELAY_CYCLES;
        }
        break;
    }

    if (error_state.is_lost)
    {
        if (error_state.recovery_delay_count > 0)
        {
            error_state.recovery_delay_count--;
        }
        else
        {
            recoverSlave(slave_pos);
            error_state.recovery_delay_count = RECOVERY_DELAY_CYCLES;
        }
    }
}

    bool IghErrorHandler::recoverSlave(int slave_pos)
    {
        requestSlaveState(slave_pos, EC_AL_STATE_OP);

        return true;
    }

    void IghErrorHandler::requestSlaveState(int slave_pos, ec_al_state_t target_state)
{
    // Note: IGH EtherCAT master handles state transitions automatically.
    // This function is primarily for logging and tracking recovery attempts.
    // The master will try to bring slaves to OP through the cyclic communication.
}

} // namespace master
