#pragma once 

#include "igh_gobal.hpp"

class IghSlave;
#include "igh_slave.hpp"

namespace master
{
class IghManager
{
    public:
        IghManager();
        ~IghManager();

        int max_slave_num_;
        IghSlave slave_[max_slave_num_];

        int configMaster();
        int setSlaves(IghSlave slave, int position);
        int configSlaves();
        int setProFilePositionParameters(ProFilePositionParm& P);
        int setCyclicPositionParameters(CyclicPositionParm& C);
        int mapDefaultPDOs(IghSlave& slave, int position);
        void configDcSyncDefault();
        void configDcSync(uint16_t assign_activated, int position);
        int checkMasterState();
        void checkMasterDomainState();
        int activateMaster();
        int registerDomain();
        int waitForOpMode();
        int openEthercatMaster();
        int getNumbOfConnectedSlaves();
        void getAllSlavesInfo();
        void deactivateMaster();
        void releaseMaster();
        int shutdown();
        uint8_t SDOread(SDO_data& pack);
        uint8_t SDOwrite(SDO_data& pack);
    private:
        int file_description;
};

}   // namespace master