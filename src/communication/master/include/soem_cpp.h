#ifndef SOEM_CPP_H
#define SOEM_CPP_H

#include "osal.h"
#include "oshw.h"
#include <stdio.h>
#include <string.h>
// #include "ethercat.h"

#include <ethercatbase.h>
#include <ethercatcoe.h>
#include <ethercatconfig.h>
#include <ethercatdc.h>
#include <ethercatfoe.h>
#include <ethercatmain.h>
#include <ethercatprint.h>
#include <ethercattype.h>
#include <nicdrv.h>

namespace master
{

/** delay in us for eeprom ready loop */
#define EC_LOCALDELAY 200

	/**
	 * @brief C++ wrapper for SOEM (Simple Open EtherCAT Master) library
	 * 
	 * Provides object-oriented interface to SOEM functions for EtherCAT
	 * master operations including configuration, PDO mapping, and cyclic communication.
	 */
	class SOEM
	{
	public:
		/**
		 * @brief Constructor
		 */
		SOEM();

		// ec config
		int ec_config_init(uint8 usetable);
		int ec_config_map(void* pIOmap);
		int ec_config_overlap_map(void* pIOmap);
		int ec_config_map_group(void* pIOmap, uint8 group);
		int ec_config_overlap_map_group(void* pIOmap, uint8 group);
		int ec_config(uint8 usetable, void* pIOmap);
		int ec_config_overlap(uint8 usetable, void* pIOmap);
		int ec_recover_slave(uint16 slave, int timeout);
		int ec_reconfig_slave(uint16 slave, int timeout);

		// ec main
		void ec_pusherror(const ec_errort* Ec);
		boolean ec_poperror(ec_errort* Ec);
		boolean ec_iserror(void);
		void ec_packeterror(uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode);
		int ec_init(const char* ifname);
		int ec_init_redundant(const char* ifname, char* if2name);
		void ec_close(void);
		uint8 ec_siigetbyte(uint16 slave, uint16 address);
		int16 ec_siifind(uint16 slave, uint16 cat);
		void ec_siistring(char* str, uint16 slave, uint16 Sn);
		uint16 ec_siiFMMU(uint16 slave, ec_eepromFMMUt* FMMU);
		uint16 ec_siiSM(uint16 slave, ec_eepromSMt* SM);
		uint16 ec_siiSMnext(uint16 slave, ec_eepromSMt* SM, uint16 n);
		int ec_siiPDO(uint16 slave, ec_eepromPDOt* PDO, uint8 t);
		int ec_readstate(void);
		int ec_writestate(uint16 slave);
		uint16 ec_statecheck(uint16 slave, uint16 reqstate, int timeout);
		int ec_mbxempty(uint16 slave, int timeout);
		int ec_mbxsend(uint16 slave, ec_mbxbuft* mbx, int timeout);
		int ec_mbxreceive(uint16 slave, ec_mbxbuft* mbx, int timeout);
		void ec_esidump(uint16 slave, uint8* esibuf);
		uint32 ec_readeeprom(uint16 slave, uint16 eeproma, int timeout);
		int ec_writeeeprom(uint16 slave, uint16 eeproma, uint16 data, int timeout);
		int ec_eeprom2master(uint16 slave);
		int ec_eeprom2pdi(uint16 slave);
		uint16 ec_eeprom_waitnotbusyAP(uint16 aiadr, uint16* estat, int timeout);
		uint64 ec_readeepromAP(uint16 aiadr, uint16 eeproma, int timeout);
		int ec_writeeepromAP(uint16 aiadr, uint16 eeproma, uint16 data, int timeout);
		uint64 ec_readeepromFP(uint16 configadr, uint16 eeproma, int timeout);
		uint16 ec_eeprom_waitnotbusyFP(uint16 configadr, uint16* estat, int timeout);
		int ec_writeeepromFP(uint16 configadr, uint16 eeproma, uint16 data, int timeout);
		void ec_readeeprom1(uint16 slave, uint16 eeproma);
		uint32 ec_readeeprom2(uint16 slave, int timeout);
		int ec_send_processdata_group(uint8 group);
		int ec_send_overlap_processdata_group(uint8 group);
		int ec_receive_processdata_group(uint8 group, int timeout);
		int ec_send_processdata(void);
		int ec_send_overlap_processdata(void);
		int ec_receive_processdata(int timeout);

		// ec coe
		void ec_SDOerror(uint16 Slave, uint16 Index, uint8 SubIdx, int32 AbortCode);
		int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean CA, int* psize, void* p, int timeout) const;
		int ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex, boolean CA, int psize, void* p, int Timeout) const;
		int ec_RxPDO(uint16 Slave, uint16 RxPDOnumber, int psize, void* p);
		int ec_TxPDO(uint16 slave, uint16 TxPDOnumber, int* psize, void* p, int timeout);
		int ec_readPDOmap(uint16 Slave, int* Osize, int* Isize);
		int ec_readPDOmapCA(uint16 Slave, int Thread_n, int* Osize, int* Isize);
		int ec_readODlist(uint16 Slave, ec_ODlistt* pODlist);
		int ec_readODdescription(uint16 Item, ec_ODlistt* pODlist);
		int ec_readOEsingle(uint16 Item, uint8 SubI, ec_ODlistt* pODlist, ec_OElistt* pOElist);
		int ec_readOE(uint16 Item, ec_ODlistt* pODlist, ec_OElistt* pOElist);
		int ec_readPDOassignCA(uint16 Slave, uint16 PDOassign, int Thread_n);
		int ec_readPDOassign(uint16 Slave, uint16 PDOassign);

		// ec dc
		boolean ec_configdc();
		void ec_dcsync0(uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift);
		void ec_dcsync01(uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift);

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
			&ecx_port,			// .port          =
			&ec_slave[0],		// .slavelist     =
			&ec_slavecount,		// .slavecount    =
			EC_MAXSLAVE,		// .maxslave      =
			&ec_group[0],		// .grouplist     =
			EC_MAXGROUP,		// .maxgroup      =
			&ec_esibuf[0],		// .esibuf        =
			&ec_esimap[0],		// .esimap        =
			0,					// .esislave      =
			&ec_elist,			// .elist         =
			&ec_idxstack,		// .idxstack      =
			&EcatError,			// .ecaterror     =
			0,					// .DCtO          =
			0,					// .DCl           =
			&ec_DCtime,			// .DCtime        =
			&ec_SMcommtype[0],	// .SMcommtype    =
			&ec_PDOassign[0],	// .PDOassign     =
			&ec_PDOdesc[0],		// .PDOdesc       =
			&ec_SM,				// .eepSM         =
			&ec_FMMU,			// .eepFMMU       =
			NULL,				// .FOEhook()
			NULL,				// .EOEhook()
			0					// .manualstatechange
		};

	private:
		/** Main slave data array.
		 *  Each slave found on the network gets its own record.
		 *  ec_slave[0] is reserved for the master. Structure gets filled
		 *  in by the configuration function ec_config().
		 */

		/** cache for EEPROM read functions */
		uint8 ec_esibuf[EC_MAXEEPBUF];
		/** bitmap for filled cache buffer bytes */
		uint32 ec_esimap[EC_MAXEEPBITMAP];
		/** current slave for EEPROM cache buffer */
		ec_eringt ec_elist;
		ec_idxstackT ec_idxstack;

		/** SyncManager Communication Type struct to store data of one slave */
		ec_SMcommtypet ec_SMcommtype[EC_MAX_MAPT];
		/** PDO assign struct to store data of one slave */
		ec_PDOassignt ec_PDOassign[EC_MAX_MAPT];
		/** PDO description struct to store data of one slave */
		ec_PDOdesct ec_PDOdesc[EC_MAX_MAPT];

		/** buffer for EEPROM SM data */
		ec_eepromSMt ec_SM;
		/** buffer for EEPROM FMMU data */
		ec_eepromFMMUt ec_FMMU;
	};

}  // namespace master

#endif