#pragma once

#include <iostream>
#include <cstring>
#include <limits.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h>
#include <chrono>
#include <memory>
#include <vector>

/****************************************************************************/
// IgH EtherCAT library header file the user-space real-time interface library.
// IgH, EtherCAT related functions and data types.
#include "ecrt.h"  

// Object dictionary paramaters PDO index and default values in here.
#include "object_dictionary.hpp"

/****************************************************************************/

#define FREQUENCEY 250;

const uint32_t g_kNsPerSec = 1000000000;                        /// nanoseconds per second
#define PERIOD_NS (g_kNsPerSec / FREQUENCEY)                    /// cycle period in nanoseconds
#define PERIOD_US (PERIOD_NS / 1000)                            /// cycle period in microseconds
#define PERIOD_MS (PERIOD_US / 1000)                            /// cycle period in milliseconds

static volatile sig_atomic_t  sig = 1;
extern ec_master_t *        g_master;                           /// EtherCAT master
extern ec_master_state_t    g_master_state;                     /// EtherCAT master state

extern ec_domain_t *        g_domain;                           /// EtherCAT domain
extern ec_domain_state_t    g_domain_state;                     /// EtherCAT domain state

extern struct timespec      g_sync_timer;                       /// timer for DC sync
const struct timespec       g_cycle_time = {0, PERIOD_NS};      /// timer increment for DC sync
extern uint32_t             g_sync_ref_counter;                 /// reference counter for DC sync

#define TEST_BIT(NUM, N)    ((NUM & (1 << N)) >> N)             /// Check specific bit in the data. 0 or 1.
#define SET_BIT(NUM, N)     (NUM | (1 << N))                    /// Set(1) specific bit in the data.
#define RESET_BIT(NUM, N)   (NUM & ~(1 << N))                   /// Reset(0) specific bit in the data

// Convert timespec struct to nanoseconds
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * g_kNsPerSec + (uint64_t)(T).tv_nsec)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * g_kNsPerSec + (B).tv_nsec - (A).tv_nsec)

#define CLOCK_TO_USE        CLOCK_MONOTONIC                     /// Using Monotonic system-wide clock.
/**
 * @brief Add two timespec struct.
 * 
 * @param time1 Timespec struct 1
 * @param time2 Timespec struct 2
 * @return Addition result
 */
inline struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= g_kNsPerSec)
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - g_kNsPerSec;
    }
    else
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

enum LifeState
{
    FAILURE = -1,
    SUCCESS,
    TRANSITIONING,
};

typedef struct
{
    uint16_t slave_position;                        ///< Slave position in the network
    uint16_t index;                                 ///< Object dictionary index
    uint8_t subindex;                               ///< Object dictionary subindex
    uint32_t data;         
    size_t data_sz;                                 ///< Size of the data
    size_t result_sz;                               ///< Size of the result data
    uint32_t error_code;                            ///< Error code
} SDO_data;

typedef enum OpMode
{
    ProFilePosition = 1,
    CyclicPosition = 8,
};

typedef struct DataReceived
{
    uint16_t com_status;
    std::vector<int32_t> target_pos;
    std::vector<int32_t> target_vel;
    std::vector<uint16_t> control_word;
    std::vector<OpMode> op_mode;

    std::vector<int32_t> actual_pos;
    std::vector<int32_t> actual_vel;
    std::vector<uint16_t> status_word;
    std::vector<uint16_t> error_code;
    std::vector<int8_t> op_mode_display;
    DataReceived(){};
};

typedef struct DataToSend
{
    std::vector<int32_t> target_pos;
    std::vector<int32_t> target_vel;
    std::vector<uint16_t> control_word;
    OpMode op_mode;

    DataToSend(){};
} DataToSend;

enum MotorStates
{
    ReadyToSwitchOn = 1,
    SwitchOn,
    OperationEnable,
    Fault,
    QuickStop,
    SwitchOnDisabled,
    Warning,
    Remote,
    TargetReached,
    InternalLimitActive
};

typedef struct
{
    ec_sdo_request * profile_acc ;    
    ec_sdo_request * profile_dec ;      
    ec_sdo_request * profile_vel ;  
    ec_sdo_request * quick_stop_dec ;
    ec_sdo_request * motion_profile_type ;
    ec_sdo_request * max_profile_vel ;
    ec_sdo_request * max_fol_err ;
    ec_sdo_request * speed_for_switch_search;
    ec_sdo_request * speed_for_zero_search;
    ec_sdo_request * homing_acc;
    ec_sdo_request * home_offset;
    ec_sdo_request * homing_method;		
} SdoRequest;

typedef struct
{
    uint32_t target_pos;
    uint32_t target_vel;
    uint32_t target_tor;
    uint32_t control_word;
    uint32_t profile_acc ;
    uint32_t profile_dec ;
    uint32_t quick_stop_dec ;
    uint32_t profile_vel ;

    uint32_t actual_pos ;
    uint32_t actual_vel ;
    uint32_t status_word ;
    uint32_t op_mode_display ;
    uint32_t error_code ;
} OffsetPDO;

typedef struct
{
    uint32_t profile_target_position;
    uint32_t profile_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
}  ProFilePositionParm;

typedef struct
{
    uint32_t cyclic_target_position;
}  CyclicPositionParm;


