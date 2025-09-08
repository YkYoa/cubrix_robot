#include <soem_cpp.h>

namespace master
{
    SOEM::SOEM()
    {
    }

    int SOEM::ec_config_init(uint8 usetable)
    {
        return ecx_config_init(&ecx_context, usetable);
    }

    int SOEM::ec_config_map(void *pIOmap)
    {
        return ec_config_map(pIOmap);
    }

    int SOEM::ec_config_overlap_map(void *pIOmap)
    {
        return ec_config_overlap_map(pIOmap);
    }

    int SOEM::ec_config_map_aligned(void *pIOmap)
    {
        return ec_config_map_group_aligned(pIOmap, 0);
    }

    int SOEM::ec_config_map_group(void *pIOmap, uint8 group)
    {
        return ecx_config_map_group(&ecx_context, pIOmap, group);
    }

    int SOEM::ec_config_overlap_map_group(void *pIOmap, uint8 group)
    {
        return ecx_config_overlap_map_group(&ecx_context, pIOmap, group);
    }

    int SOEM::ec_config_map_group_aligned(void *pIOmap, uint8 group)
    {
        return ecx_config_map_group_aligned(&ecx_context, pIOmap, group);
    }

    int SOEM::ec_config(uint8 usetable, void *pIOmap)
    {
        int wkc = ec_config_init(usetable);
        if(wkc){
            ec_config_map(pIOmap);
        }
        return wkc;
    }

    int SOEM::ec_config_overlap(uint8 usetable, void *pIOmap)
    {
        int wkc = ec_config_init(usetable);
        if(wkc){
            ec_config_overlap_map(pIOmap);
        }
        return wkc;
    }

    int SOEM::ec_recover_slave(uint16 slave, int timeout)
    {
        return ecx_recover_slave(&ecx_context, slave, timeout);
    }

    int SOEM::ec_reconfig_slave(uint16 slave, int timeout)
    {
        return ecx_reconfig_slave(&ecx_context, slave, timeout);
    }

    


}