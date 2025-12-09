#pragma once 

#include "igh_gobal.hpp"

// Undefine macros from object_dictionary.hpp that conflict with driver_parameters.h
#undef CONTROL_WORD
#undef STATUS_WORD
#undef MODE_OF_OPERATION
#undef TARGET_POSITION
#undef TOUCH_PROBE_FUNCTION
#undef ERROR_CODE
#undef MODE_OF_OPERATION_DISPLAY
#undef ACTUAL_POSITION
#undef TOUCH_PROBE_STATUS
#undef DIGITAL_INPUTS

#include "ethercat_manager.h"

class IghSlave;
#include "igh_slave.hpp"

namespace master
{
class IghManager : public EthercatMasterInterface
{
    public:
        IghManager(pthread_cond_t& cond, pthread_mutex_t& cond_lock, boost::mutex& mutex);
        ~IghManager();

        static const int max_slave_num_ = 5;
        IghSlave slave_[max_slave_num_];

        int configMaster();
        int setSlaves(IghSlave slave, int position);
        int configSlaves();
        int setProFilePositionParameters(ProFilePositionParm& P);
        int setCyclicPositionParameters();
        int mapDefaultPDOs(IghSlave& slave, int position);
        void configDcSyncDefault();
        void configDcSync(uint16_t assign_activated, int position);
        void toggleDcSync(int slave_position, uint32_t delay_us = 50000);
        int checkMasterState();
        void checkMasterDomainState();
        int openEthercatMaster();
        int getNumbOfConnectedSlaves();
        void getAllSlavesInfo();
        void deactivateMaster();
        void releaseMaster();
        int shutdown();

        int activateMaster();
        int registerDomain();
        int waitForOpMode();
        
        // SDO access methods
        uint8_t SDOread(SDO_data& pack);
        uint8_t SDOwrite(SDO_data& pack);
        
        // Cyclic communication
        int startCyclicCommunication();
        void stopCyclicCommunication();
        static void* cyclicThread(void* arg);
        void cyclicLoop();
        
        // PDO access methods (Interface implementation)
        virtual void write(int slave_no, uint8_t channel, uint8_t value) override;
        virtual uint8_t readInput(int slave_no, uint8_t channel) const override;
        virtual uint8_t readOutput(int slave_no, uint8_t channel) const override;
        virtual int getInputBits(int slave_no) const override;
        virtual int getOutputBits(int slave_no) const override;

        int fd;
        
    private:
        int file_description;
        bool stop_flag_;
        pthread_t cyclic_thread_;
        int num_slaves_;  // Dynamically detected slave count
        
        // Synchronization with controller manager
        pthread_cond_t& cond_;
        pthread_mutex_t& cond_lock_;
        boost::mutex& iomap_mutex_;
};

}   // namespace master