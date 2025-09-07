#ifndef __SOEM_CPP__H__
#define __SOEM_CPP__H__


#include "osal.h"
#include "oshw.h"
#include "stdio.h"
#include "string.h"

namespace master
{
    class SOEM
    {
        SOEM();

        //ec config
        int ecx_config_init();
        int ecx_config_map_group(void *pIOmap, uint8 group);
        int ecx_recover_slave(uint16 slave, int timeout);
        int ecx_reconfig_slave(uint16 slave, int timeout);

        //ec main
        void ecx_ecx_pusherror(const ec_errort *Ec);
        boolean ecx_poperror(ec_errort *Ec);
        boolean ecx_iserror();
        boolean ecx_packeterror(uint16 Slave, uint16 Index, uint8 SubIdx,
                                uint16 ErrorCode);
        int ecx_init(const char *ifname);
        int ecx_init_redundant(ecx_redportt *redport, const char *ifname, char *if2name);
        void ecx_close();
        uint8 ecx_siigetbyte(uint16 slave, uint16 address);
        int16 ecx_siifind(uint16 slave, uint16 cat);
        void ecx_siistring(char *str, uint16 slave, uint16 Sn);
        uint16 ecx_siiFMMU(uint16 slave, ec_eepromFMMUt *FMMU);
        uint16 ecx_siiSM(uint16 slave, ec_eepromSMt *SM);
        uint16 ecx_siiSMnext(uint16 slave, ec_eepromSMt *SM, uint16 n);
        uint32 ecx_siiPDO(uint16 slave, ec_eepromPDOt *PDO, uint8 t);
        int ecx_readstate();
        int ecx_writestate(uint16 slave);
        uint16 ecx_statecheck(uint16 slave, uint16 reqstate, int timeout);
        int ecx_mbxhandler(uint8 group, int limit);
        int ecx_mbxempty(uint16 slave, int timeout);
        int ecx_mbxsend(uint16 slave, ec_mbxbuft *mbx, int timeout);
        int ecx_mbxreceive(uint16 slave, ec_mbxbuft **mbx, int timeout);
        int ecx_mbxENIinitcmds(uint16 slave, uint16_t transition);
        void ecx_esidump(uint16 slave, uint8 *esibuf);
        uint32 ecx_readeeprom(uint16 slave, uint16 eeproma, int timeout);
        int ecx_writeeeprom(uint16 slave, uint16 eeproma, uint16 data, int timeout);
        int ecx_eeprom2master(uint16 slave);
        int ecx_eeprom2pdi(uint16 slave);
        uint64 ecx_readeepromAP(uint16 aiadr, uint16 eeproma, int timeout);
        int ecx_writeeepromAP(uint16 aiadr, uint16 eeproma, uint16 data, int timeout);
        uint64 ecx_readeepromFP(uint16 configadr, uint16 eeproma, int timeout);
        int ecx_writeeepromFP(uint16 configadr, uint16 eeproma, uint16 data, int timeout);
        uint32 ecx_readeeprom2(uint16 slave, int timeout);
        int ecx_receive_processdata_group(uint8 group, int timeout);
        int ecx_send_processdata(int);
        int ecx_receive_processdata(int timeout);
        int ecx_send_processdata_group(uint8 group);

        //ec coe
        void ecx_SDOerror(uint16 Slave, uint16 Index, uint8 SubIdx, int32 AbortCode);
        int ecx_SDOread(uint16 slave, uint16 index, uint8 subindex,
                        boolean CA, int *psize, void *p, int timeout);
        int ecx_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex,
                         boolean CA, int psize, const void *p, int Timeout);
        int ecx_RxPDO(uint16 Slave, uint16 RxPDOnumber, int psize, const void *p);
        int ecx_TxPDO(uint16 slave, uint16 TxPDOnumber, int *psize, void *p, int timeout);
        int ecx_readPDOmap(uint16 Slave, uint32 *Osize, uint32 *Isize);
        int ecx_readPDOmapCA(uint16 Slave, int Thread_n, uint32 *Osize, uint32 *Isize);
        int ecx_readODdescription(uint16 Item, ec_ODlistt *pODlist);
        int ecx_readOEsingle(uint16 Item, uint8 SubI, ec_ODlistt *pODlist, ec_OElistt *pOElist);
        int ecx_readOE(uint16 Item, ec_ODlistt *pODlist, ec_OElistt *pOElist);

        //ec dc
        boolean ecx_configdc();
        void ecx_dcsync0(uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift);
        void ecx_dcsync01(uint16 slave, boolean act, uint32 CyclTime0,
                          uint32 CyclTime1, int32 CyclShift);

        /** Slave state
         * All slave information is put in this structure. Needed for most
         * user interaction with slaves.
         */
        ec_slavet ec_slave[EC_MAXSLAVE];
        /** number of slaves found on the network */
        int ec_slavecount;
        /** slave group structure */
        ec_groupt ec_group[EC_MAXGROUP];
        /** Global variable TRUE if error available in error stack */
        boolean EcatError = FALSE;

        int64 ec_DCtime;

        ecx_portt ecx_port;
        ecx_redportt ecx_redport;

        mutable ecx_contextt ecx_context = {
            &ecx_port,
            &ec_slave[0],
            &ec_slavecount,
            EC_MAXSLAVE,
            &ec_group[0],
            EC_MAXGROUP,
            ec_DCtime,
            {0},
            {0},
            0,
            {0, 0, 0},
            {0},
            {0},
            0,
            {0},
            FALSE,
            FALSE
        };

    };
}