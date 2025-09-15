#include <soem_cpp.h>

namespace master
{

	SOEM::SOEM()
	{
	}

	/** Enumerate and init all slaves.
	 *
	 * @param[in] usetable     = TRUE when using configtable to init slaves, FALSE otherwise
	 * @return Workcounter of slave discover datagram = number of slaves found
	 * @see ecx_config_init
	 */
	int SOEM::ec_config_init(uint8 usetable)
	{
		return ecx_config_init(&ecx_context, usetable);
	}

	/** Map all PDOs in one group of slaves to IOmap with Outputs/Inputs
	 * in sequential order (legacy SOEM way).
	 *
	 * @param[out] pIOmap     = pointer to IOmap
	 * @param[in]  group      = group to map, 0 = all groups
	 * @return IOmap size
	 * @see ecx_config_map_group
	 */
	int SOEM::ec_config_map_group(void* pIOmap, uint8 group)
	{
		return ecx_config_map_group(&ecx_context, pIOmap, group);
	}

	/** Map all PDOs in one group of slaves to IOmap with Outputs/Inputs
	 * overlapping. NOTE: Must use this for TI ESC when using LRW.
	 *
	 * @param[out] pIOmap     = pointer to IOmap
	 * @param[in]  group      = group to map, 0 = all groups
	 * @return IOmap size
	 * @see ecx_config_overlap_map_group
	 */
	int SOEM::ec_config_overlap_map_group(void* pIOmap, uint8 group)
	{
		return ecx_config_overlap_map_group(&ecx_context, pIOmap, group);
	}

	/** Map all PDOs from slaves to IOmap with Outputs/Inputs
	 * in sequential order (legacy SOEM way).
	 *
	 * @param[out] pIOmap     = pointer to IOmap
	 * @return IOmap size
	 */
	int SOEM::ec_config_map(void* pIOmap)
	{
		return ec_config_map_group(pIOmap, 0);
	}

	/** Map all PDOs from slaves to IOmap with Outputs/Inputs
	 * overlapping. NOTE: Must use this for TI ESC when using LRW.
	 *
	 * @param[out] pIOmap     = pointer to IOmap
	 * @return IOmap size
	 */
	int SOEM::ec_config_overlap_map(void* pIOmap)
	{
		return ec_config_overlap_map_group(pIOmap, 0);
	}

	/** Enumerate / map and init all slaves.
	 *
	 * @param[in] usetable    = TRUE when using configtable to init slaves, FALSE otherwise
	 * @param[out] pIOmap     = pointer to IOmap
	 * @return Workcounter of slave discover datagram = number of slaves found
	 */
	int SOEM::ec_config(uint8 usetable, void* pIOmap)
	{
		int wkc;
		wkc = ec_config_init(usetable);
		if(wkc) {
			ec_config_map(pIOmap);
		}
		return wkc;
	}

	/** Enumerate / map and init all slaves.
	 *
	 * @param[in] usetable    = TRUE when using configtable to init slaves, FALSE otherwise
	 * @param[out] pIOmap     = pointer to IOmap
	 * @return Workcounter of slave discover datagram = number of slaves found
	 */
	int SOEM::ec_config_overlap(uint8 usetable, void* pIOmap)
	{
		int wkc;
		wkc = ec_config_init(usetable);
		if(wkc) {
			ec_config_overlap_map(pIOmap);
		}
		return wkc;
	}

	/** Recover slave.
	 *
	 * @param[in] slave   = slave to recover
	 * @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
	 * @return >0 if successful
	 * @see ecx_recover_slave
	 */
	int SOEM::ec_recover_slave(uint16 slave, int timeout)
	{
		return ecx_recover_slave(&ecx_context, slave, timeout);
	}

	/** Reconfigure slave.
	 *
	 * @param[in] slave   = slave to reconfigure
	 * @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
	 * @return Slave state
	 * @see ecx_reconfig_slave
	 */
	int SOEM::ec_reconfig_slave(uint16 slave, int timeout)
	{
		return ecx_reconfig_slave(&ecx_context, slave, timeout);
	}

	void SOEM::ec_pusherror(const ec_errort* Ec)
	{
		ecx_pusherror(&ecx_context, Ec);
	}

	boolean SOEM::ec_poperror(ec_errort* Ec)
	{
		return ecx_poperror(&ecx_context, Ec);
	}

	boolean SOEM::ec_iserror(void)
	{
		return ecx_iserror(&ecx_context);
	}

	void SOEM::ec_packeterror(uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode)
	{
		ecx_packeterror(&ecx_context, Slave, Index, SubIdx, ErrorCode);
	}

	/** Initialise lib in single NIC mode
	 * @param[in] ifname   = Dev name, f.e. "eth0"
	 * @return >0 if OK
	 * @see ecx_init
	 */
	int SOEM::ec_init(const char* ifname)
	{
		return ecx_init(&ecx_context, ifname);
	}

	/** Initialise lib in redundant NIC mode
	 * @param[in]  ifname   = Primary Dev name, f.e. "eth0"
	 * @param[in]  if2name  = Secondary Dev name, f.e. "eth1"
	 * @return >0 if OK
	 * @see ecx_init_redundant
	 */
	int SOEM::ec_init_redundant(const char* ifname, char* if2name)
	{
		return ecx_init_redundant(&ecx_context, &ecx_redport, ifname, if2name);
	}

	/** Close lib.
	 * @see ecx_close
	 */
	void SOEM::ec_close(void)
	{
		ecx_close(&ecx_context);
	};

	/** Read one byte from slave EEPROM via cache.
	 *  If the cache location is empty then a read request is made to the slave.
	 *  Depending on the slave capabillities the request is 4 or 8 bytes.
	 *  @param[in] slave   = slave number
	 *  @param[in] address = eeprom address in bytes (slave uses words)
	 *  @return requested byte, if not available then 0xff
	 * @see ecx_siigetbyte
	 */
	uint8 SOEM::ec_siigetbyte(uint16 slave, uint16 address)
	{
		return ecx_siigetbyte(&ecx_context, slave, address);
	}

	/** Find SII section header in slave EEPROM.
	 *  @param[in] slave   = slave number
	 *  @param[in] cat     = section category
	 *  @return byte address of section at section length entry, if not available then 0
	 *  @see ecx_siifind
	 */
	int16 SOEM::ec_siifind(uint16 slave, uint16 cat)
	{
		return ecx_siifind(&ecx_context, slave, cat);
	}

	/** Get string from SII string section in slave EEPROM.
	 *  @param[out] str    = requested string, 0x00 if not found
	 *  @param[in]  slave  = slave number
	 *  @param[in]  Sn     = string number
	 *  @see ecx_siistring
	 */
	void SOEM::ec_siistring(char* str, uint16 slave, uint16 Sn)
	{
		ecx_siistring(&ecx_context, str, slave, Sn);
	}

	/** Get FMMU data from SII FMMU section in slave EEPROM.
	 *  @param[in]  slave  = slave number
	 *  @param[out] FMMU   = FMMU struct from SII, max. 4 FMMU's
	 *  @return number of FMMU's defined in section
	 *  @see ecx_siiFMMU
	 */
	uint16 SOEM::ec_siiFMMU(uint16 slave, ec_eepromFMMUt* FMMU)
	{
		return ecx_siiFMMU(&ecx_context, slave, FMMU);
	}

	/** Get SM data from SII SM section in slave EEPROM.
	 *  @param[in]  slave   = slave number
	 *  @param[out] SM      = first SM struct from SII
	 *  @return number of SM's defined in section
	 *  @see ecx_siiSM
	 */
	uint16 SOEM::ec_siiSM(uint16 slave, ec_eepromSMt* SM)
	{
		return ecx_siiSM(&ecx_context, slave, SM);
	}

	/** Get next SM data from SII SM section in slave EEPROM.
	 *  @param[in]  slave  = slave number
	 *  @param[out] SM     = first SM struct from SII
	 *  @param[in]  n      = SM number
	 *  @return >0 if OK
	 *  @see ecx_siiSMnext
	 */
	uint16 SOEM::ec_siiSMnext(uint16 slave, ec_eepromSMt* SM, uint16 n)
	{
		return ecx_siiSMnext(&ecx_context, slave, SM, n);
	}

	/** Get PDO data from SII PDO section in slave EEPROM.
	 *  @param[in]  slave  = slave number
	 *  @param[out] PDO    = PDO struct from SII
	 *  @param[in]  t      = 0=RXPDO 1=TXPDO
	 *  @return mapping size in bits of PDO
	 *  @see ecx_siiPDO
	 */
	int SOEM::ec_siiPDO(uint16 slave, ec_eepromPDOt* PDO, uint8 t)
	{
		return ecx_siiPDO(&ecx_context, slave, PDO, t);
	}

	/** Read all slave states in ec_slave.
	 * @return lowest state found
	 * @see ecx_readstate
	 */
	int SOEM::ec_readstate(void)
	{
		return ecx_readstate(&ecx_context);
	}

	/** Write slave state, if slave = 0 then write to all slaves.
	 * The function does not check if the actual state is changed.
	 * @param[in] slave = Slave number, 0 = master
	 * @return 0
	 * @see ecx_writestate
	 */
	int SOEM::ec_writestate(uint16 slave)
	{
		return ecx_writestate(&ecx_context, slave);
	}

	/** Check actual slave state.
	 * This is a blocking function.
	 * @param[in] slave       = Slave number, 0 = all slaves
	 * @param[in] reqstate    = Requested state
	 * @param[in] timeout     = Timeout value in us
	 * @return Requested state, or found state after timeout.
	 * @see ecx_statecheck
	 */
	uint16 SOEM::ec_statecheck(uint16 slave, uint16 reqstate, int timeout)
	{
		return ecx_statecheck(&ecx_context, slave, reqstate, timeout);
	}

	/** Check if IN mailbox of slave is empty.
	 * @param[in] slave    = Slave number
	 * @param[in] timeout  = Timeout in us
	 * @return >0 is success
	 * @see ecx_mbxempty
	 */
	int SOEM::ec_mbxempty(uint16 slave, int timeout)
	{
		return ecx_mbxempty(&ecx_context, slave, timeout);
	}

	/** Write IN mailbox to slave.
	 * @param[in]  slave      = Slave number
	 * @param[out] mbx        = Mailbox data
	 * @param[in]  timeout    = Timeout in us
	 * @return Work counter (>0 is success)
	 * @see ecx_mbxsend
	 */
	int SOEM::ec_mbxsend(uint16 slave, ec_mbxbuft* mbx, int timeout)
	{
		return ecx_mbxsend(&ecx_context, slave, mbx, timeout);
	}

	/** Read OUT mailbox from slave.
	 * Supports Mailbox Link Layer with repeat requests.
	 * @param[in]  slave      = Slave number
	 * @param[out] mbx        = Mailbox data
	 * @param[in]  timeout    = Timeout in us
	 * @return Work counter (>0 is success)
	 * @see ecx_mbxreceive
	 */
	int SOEM::ec_mbxreceive(uint16 slave, ec_mbxbuft* mbx, int timeout)
	{
		return ecx_mbxreceive(&ecx_context, slave, mbx, timeout);
	}

	/** Dump complete EEPROM data from slave in buffer.
	 * @param[in]  slave    = Slave number
	 * @param[out] esibuf   = EEPROM data buffer, make sure it is big enough.
	 * @see ecx_esidump
	 */
	void SOEM::ec_esidump(uint16 slave, uint8* esibuf)
	{
		ecx_esidump(&ecx_context, slave, esibuf);
	}

	/** Read EEPROM from slave bypassing cache.
	 * @param[in] slave     = Slave number
	 * @param[in] eeproma   = (WORD) Address in the EEPROM
	 * @param[in] timeout   = Timeout in us.
	 * @return EEPROM data 32bit
	 * @see ecx_readeeprom
	 */
	uint32 SOEM::ec_readeeprom(uint16 slave, uint16 eeproma, int timeout)
	{
		return ecx_readeeprom(&ecx_context, slave, eeproma, timeout);
	}

	/** Write EEPROM to slave bypassing cache.
	 * @param[in] slave     = Slave number
	 * @param[in] eeproma   = (WORD) Address in the EEPROM
	 * @param[in] data      = 16bit data
	 * @param[in] timeout   = Timeout in us.
	 * @return >0 if OK
	 * @see ecx_writeeeprom
	 */
	int SOEM::ec_writeeeprom(uint16 slave, uint16 eeproma, uint16 data, int timeout)
	{
		return ecx_writeeeprom(&ecx_context, slave, eeproma, data, timeout);
	}

	/** Set eeprom control to master. Only if set to PDI.
	 * @param[in] slave = Slave number
	 * @return >0 if OK
	 * @see ecx_eeprom2master
	 */
	int SOEM::ec_eeprom2master(uint16 slave)
	{
		return ecx_eeprom2master(&ecx_context, slave);
	}

	int SOEM::ec_eeprom2pdi(uint16 slave)
	{
		return ecx_eeprom2pdi(&ecx_context, slave);
	}

	uint16 ecx_eeprom_waitnotbusyAP(ecx_contextt* context, uint16 aiadr, uint16* estat, int timeout)
	{
		int wkc, cnt = 0, retval = 0;
		osal_timert timer;

		osal_timer_start(&timer, timeout);
		do {
			if(cnt++) {
				osal_usleep(EC_LOCALDELAY);
			}
			*estat = 0;
			wkc	   = ecx_APRD(context->port, aiadr, ECT_REG_EEPSTAT, sizeof(*estat), estat, EC_TIMEOUTRET);
			*estat = etohs(*estat);
		} while(((wkc <= 0) || ((*estat & EC_ESTAT_BUSY) > 0)) && (osal_timer_is_expired(&timer) == FALSE)); /* wait for eeprom ready */
		if((*estat & EC_ESTAT_BUSY) == 0) {
			retval = 1;
		}

		return retval;
	}

	uint16 SOEM::ec_eeprom_waitnotbusyAP(uint16 aiadr, uint16* estat, int timeout)
	{
		return ecx_eeprom_waitnotbusyAP(&ecx_context, aiadr, estat, timeout);
	}

	/** Read EEPROM from slave bypassing cache. APRD method.
	 * @param[in] aiadr       = auto increment address of slave
	 * @param[in] eeproma     = (WORD) Address in the EEPROM
	 * @param[in] timeout     = Timeout in us.
	 * @return EEPROM data 64bit or 32bit
	 */
	uint64 SOEM::ec_readeepromAP(uint16 aiadr, uint16 eeproma, int timeout)
	{
		return ecx_readeepromAP(&ecx_context, aiadr, eeproma, timeout);
	}

	/** Write EEPROM to slave bypassing cache. APWR method.
	 * @param[in] aiadr     = configured address of slave
	 * @param[in] eeproma   = (WORD) Address in the EEPROM
	 * @param[in] data      = 16bit data
	 * @param[in] timeout   = Timeout in us.
	 * @return >0 if OK
	 * @see ecx_writeeepromAP
	 */
	int SOEM::ec_writeeepromAP(uint16 aiadr, uint16 eeproma, uint16 data, int timeout)
	{
		return ecx_writeeepromAP(&ecx_context, aiadr, eeproma, data, timeout);
	}

	uint16 ecx_eeprom_waitnotbusyFP(ecx_contextt* context, uint16 configadr, uint16* estat, int timeout)
	{
		int wkc, cnt = 0, retval = 0;
		osal_timert timer;

		osal_timer_start(&timer, timeout);
		do {
			if(cnt++) {
				osal_usleep(EC_LOCALDELAY);
			}
			*estat = 0;
			wkc	   = ecx_FPRD(context->port, configadr, ECT_REG_EEPSTAT, sizeof(*estat), estat, EC_TIMEOUTRET);
			*estat = etohs(*estat);
		} while(((wkc <= 0) || ((*estat & EC_ESTAT_BUSY) > 0)) && (osal_timer_is_expired(&timer) == FALSE)); /* wait for eeprom ready */
		if((*estat & EC_ESTAT_BUSY) == 0) {
			retval = 1;
		}

		return retval;
	}
	uint16 SOEM::ec_eeprom_waitnotbusyFP(uint16 configadr, uint16* estat, int timeout)
	{
		return ecx_eeprom_waitnotbusyFP(&ecx_context, configadr, estat, timeout);
	}

	/** Read EEPROM from slave bypassing cache. FPRD method.
	 * @param[in] configadr   = configured address of slave
	 * @param[in] eeproma     = (WORD) Address in the EEPROM
	 * @param[in] timeout     = Timeout in us.
	 * @return EEPROM data 64bit or 32bit
	 * @see ecx_readeepromFP
	 */
	uint64 SOEM::ec_readeepromFP(uint16 configadr, uint16 eeproma, int timeout)
	{
		return ecx_readeepromFP(&ecx_context, configadr, eeproma, timeout);
	}

	/** Write EEPROM to slave bypassing cache. FPWR method.
	 * @param[in] configadr   = configured address of slave
	 * @param[in] eeproma     = (WORD) Address in the EEPROM
	 * @param[in] data        = 16bit data
	 * @param[in] timeout     = Timeout in us.
	 * @return >0 if OK
	 * @see ecx_writeeepromFP
	 */
	int SOEM::ec_writeeepromFP(uint16 configadr, uint16 eeproma, uint16 data, int timeout)
	{
		return ecx_writeeepromFP(&ecx_context, configadr, eeproma, data, timeout);
	}

	/** Read EEPROM from slave bypassing cache.
	 * Parallel read step 1, make request to slave.
	 * @param[in] slave       = Slave number
	 * @param[in] eeproma     = (WORD) Address in the EEPROM
	 * @see ecx_readeeprom1
	 */
	void SOEM::ec_readeeprom1(uint16 slave, uint16 eeproma)
	{
		ecx_readeeprom1(&ecx_context, slave, eeproma);
	}

	/** Read EEPROM from slave bypassing cache.
	 * Parallel read step 2, actual read from slave.
	 * @param[in] slave       = Slave number
	 * @param[in] timeout     = Timeout in us.
	 * @return EEPROM data 32bit
	 * @see ecx_readeeprom2
	 */
	uint32 SOEM::ec_readeeprom2(uint16 slave, int timeout)
	{
		return ecx_readeeprom2(&ecx_context, slave, timeout);
	}

	/** Transmit processdata to slaves.
	 * Uses LRW, or LRD/LWR if LRW is not allowed (blockLRW).
	 * Both the input and output processdata are transmitted.
	 * The outputs with the actual data, the inputs have a placeholder.
	 * The inputs are gathered with the receive processdata function.
	 * In contrast to the base LRW function this function is non-blocking.
	 * If the processdata does not fit in one datagram, multiple are used.
	 * In order to recombine the slave response, a stack is used.
	 * @param[in]  group          = group number
	 * @return >0 if processdata is transmitted.
	 * @see ecx_send_processdata_group
	 */
	int SOEM::ec_send_processdata_group(uint8 group)
	{
		return ecx_send_processdata_group(&ecx_context, group);
	}

	/** Transmit processdata to slaves.
	 * Uses LRW, or LRD/LWR if LRW is not allowed (blockLRW).
	 * Both the input and output processdata are transmitted in the overlapped IOmap.
	 * The outputs with the actual data, the inputs replace the output data in the
	 * returning frame. The inputs are gathered with the receive processdata function.
	 * In contrast to the base LRW function this function is non-blocking.
	 * If the processdata does not fit in one datagram, multiple are used.
	 * In order to recombine the slave response, a stack is used.
	 * @param[in]  group          = group number
	 * @return >0 if processdata is transmitted.
	 * @see ecx_send_overlap_processdata_group
	 */
	int SOEM::ec_send_overlap_processdata_group(uint8 group)
	{
		return ecx_send_overlap_processdata_group(&ecx_context, group);
	}

	/** Receive processdata from slaves.
	 * Second part from ec_send_processdata().
	 * Received datagrams are recombined with the processdata with help from the stack.
	 * If a datagram contains input processdata it copies it to the processdata structure.
	 * @param[in]  group          = group number
	 * @param[in]  timeout        = Timeout in us.
	 * @return Work counter.
	 * @see ecx_receive_processdata_group
	 */
	int SOEM::ec_receive_processdata_group(uint8 group, int timeout)
	{
		return ecx_receive_processdata_group(&ecx_context, group, timeout);
	}

	int SOEM::ec_send_processdata(void)
	{
		return ec_send_processdata_group(0);
	}

	int SOEM::ec_send_overlap_processdata(void)
	{
		return ec_send_overlap_processdata_group(0);
	}

	int SOEM::ec_receive_processdata(int timeout)
	{
		return ec_receive_processdata_group(0, timeout);
	}

	/** Report SDO error.
	 *
	 * @param[in]  Slave      = Slave number
	 * @param[in]  Index      = Index that generated error
	 * @param[in]  SubIdx     = Subindex that generated error
	 * @param[in]  AbortCode  = Abortcode, see EtherCAT documentation for list
	 * @see ecx_SDOerror
	 */
	void SOEM::ec_SDOerror(uint16 Slave, uint16 Index, uint8 SubIdx, int32 AbortCode)
	{
		ecx_SDOerror(&ecx_context, Slave, Index, SubIdx, AbortCode);
	}

	/** CoE SDO read, blocking. Single subindex or Complete Access.
	 *
	 * Only a "normal" upload request is issued. If the requested parameter is <= 4bytes
	 * then a "expedited" response is returned, otherwise a "normal" response. If a "normal"
	 * response is larger than the mailbox size then the response is segmented. The function
	 * will combine all segments and copy them to the parameter buffer.
	 *
	 * @param[in]  slave      = Slave number
	 * @param[in]  index      = Index to read
	 * @param[in]  subindex   = Subindex to read, must be 0 or 1 if CA is used.
	 * @param[in]  CA         = FALSE = single subindex. TRUE = Complete Access, all subindexes read.
	 * @param[in,out] psize   = Size in bytes of parameter buffer, returns bytes read from SDO.
	 * @param[out] p          = Pointer to parameter buffer
	 * @param[in]  timeout    = Timeout in us, standard is EC_TIMEOUTRXM
	 * @return Workcounter from last slave response
	 * @see ecx_SDOread
	 */
	int SOEM::ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean CA, int* psize, void* p, int timeout) const
	{
		return ecx_SDOread(&ecx_context, slave, index, subindex, CA, psize, p, timeout);
	}

	/** CoE SDO write, blocking. Single subindex or Complete Access.
	 *
	 * A "normal" download request is issued, unless we have
	 * small data, then a "expedited" transfer is used. If the parameter is larger than
	 * the mailbox size then the download is segmented. The function will split the
	 * parameter data in segments and send them to the slave one by one.
	 *
	 * @param[in]  Slave      = Slave number
	 * @param[in]  Index      = Index to write
	 * @param[in]  SubIndex   = Subindex to write, must be 0 or 1 if CA is used.
	 * @param[in]  CA         = FALSE = single subindex. TRUE = Complete Access, all subindexes written.
	 * @param[in]  psize      = Size in bytes of parameter buffer.
	 * @param[out] p          = Pointer to parameter buffer
	 * @param[in]  Timeout    = Timeout in us, standard is EC_TIMEOUTRXM
	 * @return Workcounter from last slave response
	 * @see ecx_SDOwrite
	 */
	int SOEM::ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex, boolean CA, int psize, void* p, int Timeout) const
	{
		return ecx_SDOwrite(&ecx_context, Slave, Index, SubIndex, CA, psize, p, Timeout);
	}

	/** CoE RxPDO write, blocking.
	 *
	 * A RxPDO download request is issued.
	 *
	 * @param[in]  Slave         = Slave number
	 * @param[in]  RxPDOnumber   = Related RxPDO number
	 * @param[in]  psize         = Size in bytes of PDO buffer.
	 * @param[out] p             = Pointer to PDO buffer
	 * @return Workcounter from last slave response
	 * @see ecx_RxPDO
	 */
	int SOEM::ec_RxPDO(uint16 Slave, uint16 RxPDOnumber, int psize, void* p)
	{
		return ecx_RxPDO(&ecx_context, Slave, RxPDOnumber, psize, p);
	}

	/** CoE TxPDO read remote request, blocking.
	 *
	 * A RxPDO download request is issued.
	 *
	 * @param[in]  slave         = Slave number
	 * @param[in]  TxPDOnumber   = Related TxPDO number
	 * @param[in,out] psize      = Size in bytes of PDO buffer, returns bytes read from PDO.
	 * @param[out] p             = Pointer to PDO buffer
	 * @param[in]  timeout       = Timeout in us, standard is EC_TIMEOUTRXM
	 * @return Workcounter from last slave response
	 * @see ecx_TxPDO
	 */
	int SOEM::ec_TxPDO(uint16 slave, uint16 TxPDOnumber, int* psize, void* p, int timeout)
	{
		return ecx_TxPDO(&ecx_context, slave, TxPDOnumber, psize, p, timeout);
	}

	/** Read PDO assign structure
	 * @param[in]  context       = context struct
	 * @param[in]  Slave         = Slave number
	 * @param[in]  PDOassign     = PDO assign object
	 * @return total bitlength of PDO assign
	 */
	int ecx_readPDOassign(ecx_contextt* context, uint16 Slave, uint16 PDOassign)
	{
		uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
		uint8 subcnt;
		int wkc, bsize = 0, rdl;
		int32 rdat2;

		rdl	 = sizeof(rdat);
		rdat = 0;
		/* read PDO assign subindex 0 ( = number of PDO's) */
		wkc	 = ecx_SDOread(context, Slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
		rdat = etohs(rdat);
		/* positive result from slave ? */
		if((wkc > 0) && (rdat > 0)) {
			/* number of available sub indexes */
			nidx  = rdat;
			bsize = 0;
			/* read all PDO's */
			for(idxloop = 1; idxloop <= nidx; idxloop++) {
				rdl	 = sizeof(rdat);
				rdat = 0;
				/* read PDO assign */
				wkc = ecx_SDOread(context, Slave, PDOassign, (uint8) idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
				/* result is index of PDO */
				idx = etohs(rdat);
				if(idx > 0) {
					rdl	   = sizeof(subcnt);
					subcnt = 0;
					/* read number of subindexes of PDO */
					wkc	   = ecx_SDOread(context, Slave, idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
					subidx = subcnt;
					/* for each subindex */
					for(subidxloop = 1; subidxloop <= subidx; subidxloop++) {
						rdl	  = sizeof(rdat2);
						rdat2 = 0;
						/* read SDO that is mapped in PDO */
						wkc	  = ecx_SDOread(context, Slave, idx, (uint8) subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
						rdat2 = etohl(rdat2);
						/* extract bitlength of SDO */
						if(LO_BYTE(rdat2) < 0xff) {
							bsize += LO_BYTE(rdat2);
						}
						else {
							rdl	 = sizeof(rdat);
							rdat = htoes(0xff);
							/* read Object Entry in Object database */
							//                  wkc = ec_readOEsingle(idx, (uint8)SubCount, pODlist, pOElist);
							bsize += etohs(rdat);
						}
					}
				}
			}
		}
		/* return total found bitlength (PDO) */
		return bsize;
	}

	/** Read PDO assign structure
	 * @param[in]  Slave         = Slave number
	 * @param[in]  PDOassign     = PDO assign object
	 * @return total bitlength of PDO assign
	 */
	int SOEM::ec_readPDOassign(uint16 Slave, uint16 PDOassign)
	{
		return ecx_readPDOassign(&ecx_context, Slave, PDOassign);
	}

	/** Read PDO assign structure in Complete Access mode
	 * @param[in]  context       = context struct
	 * @param[in]  Slave         = Slave number
	 * @param[in]  Thread_n      = Calling thread index
	 * @param[in]  PDOassign     = PDO assign object
	 * @return total bitlength of PDO assign
	 */
	int ecx_readPDOassignCA(ecx_contextt* context, uint16 Slave, int Thread_n, uint16 PDOassign)
	{
		uint16 idxloop, nidx, subidxloop, idx, subidx;
		int wkc, bsize = 0, rdl;

		/* find maximum size of PDOassign buffer */
		rdl							   = sizeof(ec_PDOassignt);
		context->PDOassign[Thread_n].n = 0;
		/* read rxPDOassign in CA mode, all subindexes are read in one struct */
		wkc = ecx_SDOread(context, Slave, PDOassign, 0x00, TRUE, &rdl, &(context->PDOassign[Thread_n]), EC_TIMEOUTRXM);
		/* positive result from slave ? */
		if((wkc > 0) && (context->PDOassign[Thread_n].n > 0)) {
			nidx  = context->PDOassign[Thread_n].n;
			bsize = 0;
			/* for each PDO do */
			for(idxloop = 1; idxloop <= nidx; idxloop++) {
				/* get index from PDOassign struct */
				idx = etohs(context->PDOassign[Thread_n].index[idxloop - 1]);
				if(idx > 0) {
					rdl							 = sizeof(ec_PDOdesct);
					context->PDOdesc[Thread_n].n = 0;
					/* read SDO's that are mapped in PDO, CA mode */
					wkc	   = ecx_SDOread(context, Slave, idx, 0x00, TRUE, &rdl, &(context->PDOdesc[Thread_n]), EC_TIMEOUTRXM);
					subidx = context->PDOdesc[Thread_n].n;
					/* extract all bitlengths of SDO's */
					for(subidxloop = 1; subidxloop <= subidx; subidxloop++) {
						bsize += LO_BYTE(etohl(context->PDOdesc[Thread_n].PDO[subidxloop - 1]));
					}
				}
			}
		}

		/* return total found bitlength (PDO) */
		return bsize;
	}
	/** Read PDO assign structure in Complete Access mode
	 * @param[in]  Slave         = Slave number
	 * @param[in]  PDOassign     = PDO assign object
	 * @param[in]  Thread_n      = Calling thread index
	 * @return total bitlength of PDO assign
	 * @see ecx_readPDOmap
	 */
	int SOEM::ec_readPDOassignCA(uint16 Slave, uint16 PDOassign, int Thread_n)
	{
		return ecx_readPDOassignCA(&ecx_context, Slave, Thread_n, PDOassign);
	}

	/** CoE read PDO mapping.
	 *
	 * CANopen has standard indexes defined for PDO mapping. This function
	 * tries to read them and collect a full input and output mapping size
	 * of designated slave.
	 *
	 * For details, see #ecx_readPDOmap
	 *
	 * @param[in] Slave    = Slave number
	 * @param[out] Osize   = Size in bits of output mapping (rxPDO) found
	 * @param[out] Isize   = Size in bits of input mapping (txPDO) found
	 * @return >0 if mapping succesful.
	 */
	int SOEM::ec_readPDOmap(uint16 Slave, int* Osize, int* Isize)
	{
		return ecx_readPDOmap(&ecx_context, Slave, Osize, Isize);
	}

	/** CoE read PDO mapping in Complete Access mode (CA).
	 *
	 * CANopen has standard indexes defined for PDO mapping. This function
	 * tries to read them and collect a full input and output mapping size
	 * of designated slave. Slave has to support CA, otherwise use ec_readPDOmap().
	 *
	 * @param[in] Slave    = Slave number
	 * @param[in] Thread_n = Calling thread index
	 * @param[out] Osize   = Size in bits of output mapping (rxPDO) found
	 * @param[out] Isize   = Size in bits of input mapping (txPDO) found
	 * @return >0 if mapping succesful.
	 * @see ecx_readPDOmap ec_readPDOmapCA
	 */
	int SOEM::ec_readPDOmapCA(uint16 Slave, int Thread_n, int* Osize, int* Isize)
	{
		return ecx_readPDOmapCA(&ecx_context, Slave, Thread_n, Osize, Isize);
	}

	/** CoE read Object Description List.
	 *
	 * @param[in] Slave      = Slave number.
	 * @param[out] pODlist  = resulting Object Description list.
	 * @return Workcounter of slave response.
	 * @see ecx_readODlist
	 */
	int SOEM::ec_readODlist(uint16 Slave, ec_ODlistt* pODlist)
	{
		return ecx_readODlist(&ecx_context, Slave, pODlist);
	}

	/** CoE read Object Description. Adds textual description to object indexes.
	 *
	 * @param[in] Item           = Item number in ODlist.
	 * @param[in,out] pODlist    = referencing Object Description list.
	 * @return Workcounter of slave response.
	 * @see ecx_readODdescription
	 */
	int SOEM::ec_readODdescription(uint16 Item, ec_ODlistt* pODlist)
	{
		return ecx_readODdescription(&ecx_context, Item, pODlist);
	}

	int SOEM::ec_readOEsingle(uint16 Item, uint8 SubI, ec_ODlistt* pODlist, ec_OElistt* pOElist)
	{
		return ecx_readOEsingle(&ecx_context, Item, SubI, pODlist, pOElist);
	}

	/** CoE read SDO service object entry.
	 *
	 * @param[in] Item           = Item in ODlist.
	 * @param[in] pODlist        = Object description list for reference.
	 * @param[out] pOElist       = resulting object entry structure.
	 * @return Workcounter of slave response.
	 * @see ecx_readOE
	 */
	int SOEM::ec_readOE(uint16 Item, ec_ODlistt* pODlist, ec_OElistt* pOElist)
	{
		return ecx_readOE(&ecx_context, Item, pODlist, pOElist);
	}

	void SOEM::ec_dcsync0(uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift)
	{
		ecx_dcsync0(&ecx_context, slave, act, CyclTime, CyclShift);
	}

	void SOEM::ec_dcsync01(uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift)
	{
		ecx_dcsync01(&ecx_context, slave, act, CyclTime0, CyclTime1, CyclShift);
	}

	boolean SOEM::ec_configdc(void)
	{
		return ecx_configdc(&ecx_context);
	}

}  // namespace master