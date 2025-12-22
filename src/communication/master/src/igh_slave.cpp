#include "igh_slave.hpp"

IghSlave::IghSlave()
{
}

IghSlave::~IghSlave()
{
}

int IghSlave::checkSlaveConfigState()
{
    ecrt_slave_config_state(slave_config_, &slave_config_state_);
    if (slave_config_state_.al_state != 0x08)
    {
        std::cout << " Slave is not operational AL state is :" << std::hex << slave_config_state_.al_state << std::endl;
    }
    return 0;
}
