#pragma once

#define DEVICE_TYPE                                 0x1000,0x00
#define ERROR_REGISTER                              0x1001,0x00
#define DEVICE_NAME                                 0x1008,0x00
#define STORE_PARAMETERS_NUMB_OF_SUB_INDEX          0x1010,0x00
#define SAVE_ALL_PARAMETERS                         0x1010,0x01
#define SAVE_COMMUNICATION_PARAMETERS               0x1010,0x02
#define SAVE_MOTION_PARAMETERS                      0x1010,0x03
#define SAVE_FACTORY_PARAMETERS                     0x1010,0x04
#define RESET_PARAMETERS_NUMB_OF_SUB_INDEX          0x1011,0x00
#define RESET_ALL_PARAMETERS                        0x1011,0x01
#define RESET_COMMUNICATION_PARAMETERS              0x1011,0x02
#define RESET_MOTION_PARAMETERS                     0x1011,0x03
#define RESET_FACTORY_PARAMETERS                    0x1011,0x04
#define IDENTIFY_OBJECT_NUMB_OF_SUB_INDEX           0x1018,0x00
#define VENDOR_ID                                   0x1018,0x01
#define PRODUCT_CODE                                0x1018,0x02
#define REVISION_NUMBER                             0x1018,0x03
#define SERIAL_NUMBER                               0x1018,0x04

#define RX_PDO1_MAPPING_NUMB_OF_SUB_INDEX           0x1600,0x00
#define RX_PDO2_MAPPING_NUMB_OF_SUB_INDEX           0x1601,0x00
#define RX_PDO3_MAPPING_NUMB_OF_SUB_INDEX           0x1602,0x00
#define RX_PDO4_MAPPING_NUMB_OF_SUB_INDEX           0x1603,0x00

#define TX_PDO1_MAPPING_NUMB_OF_SUB_INDEX           0x1A00,0x00
#define TX_PDO2_MAPPING_NUMB_OF_SUB_INDEX           0x1A01,0x00

#define SM_COMMUNICATION_NUMB_OF_INDEX              0x1C00,0x00
#define OUTPUT_TYPE_OF_EMAIL                        0x1C00,0x01       
#define INPUT_TYPE_OF_EMAIL                         0x1C00,0x02
#define OUTPUT_TYPE_OF_PD                           0x1C00,0x03
#define INPUT_TYPE_OF_PD                            0x1C00,0x04

#define SM2_PDO_ASSIGNMENT                          0x1C12,0x00
#define SM3_PDO_ASSIGNMENT                          0x1C13,0x00

#define SM2_SYNC_TYPE                               0x1C32,0x01
#define SM2_CYCLE_TIME                              0x1C32,0x02
#define SM2_SYNC_TYPE_SUPPORT                       0x1C32,0x04
#define SM2_MINIMUM_CYCLE_TIME                      0x1C32,0x05
#define SM2_CALC_AND_COPY_TIME                      0x1C32,0x06
#define SM2_HARDWARE_DELAY                          0x1C32,0x09
#define SM2_SYNC0_CYCLE_TIME                        0x1C32,0x0A
#define SM2_EVENT_MISSED_COUNTER                    0x1C32,0x0B 
#define SM2_CYCLE_TIME_TOO_SMALL_COUNTER            0x1C32,0x0C

#define CONTROL_WORD                                0x6040,0x00
#define STATUS_WORD                                 0x6041,0x00

#define MODE_OF_OPERATION                           0x6060,0x00
#define MODE_OF_OPERATION_DISPLAY                   0x6061,0x00

#define TARGET_POSITION                             0x607A,0x00
#define ACTUAL_POSITION                             0x6064,0x00
#define TOUCH_PROBE_FUNCTION                        0x60B8,0x00
#define TOUCH_PROBE_STATUS                          0x60B9,0x00
#define TOUCH_PROBE_1_POSITION_VALUE                0x60BA,0x01
#define DIGITAL_INPUTS                              0x60FD,0x00

#define INTERPOLATED_TIME_VALUE                     0x60C2,0x01
#define INTERPOLATED_TIME_UNIT                      0x60C2,0x02

/*************************************************************/
// CiA402 State Machine Definitions start.

#define SM_GO_SWITCH_ON_DISABLE                 0x00
#define SM_START                                0x01
#define SM_QUICK_STOP                           0x02
#define SM_READY_TO_SWITCH_ON                   0x06
#define SM_SWITCHED_ON                          0x07
#define SM_OPERATION_ENABLED                    0x0F
#define SM_RUN                                  0x1F

/* From CiA402,  - State coding
	Statusword      |      PDS FSA state
xxxx xxxx x0xx 0000 | Not ready to switch on
xxxx xxxx x1xx 0000 | Switch on disabled
xxxx xxxx x01x 0001 | Ready to switch on
xxxx xxxx x01x 0011 | Switched on
xxxx xxxx x01x 0111 | Operation enabled
xxxx xxxx x00x 0111 | Quick stop active
xxxx xxxx x0xx 1111 | Fault reaction active
xxxx xxxx x0xx 1000 | Fault
*/

#define SM_FSAFROMSTATUSWORD(SW) 		(SW & 0x006f)
#define SM_NOT_READY_TO_SWITCH_ON   	0b00000000
#define SM_NOT_READY_TO_SWITCH_ON_2  	0b00100000
#define SM_SWITCH_ON_DISABLED     		0b01000000
#define SM_SWITCH_ON_DISABLED_2    		0b01100000
#define SM_READY_TO_SWITCH_ON      		0b00100001
#define SM_SWITCHED_ON           		0b00100011
#define SM_OPERATION_ENABLED     		0b00100111
#define SM_QUICK_STOP_ACTIVE      		0b00000111
#define SM_FAULT_REACTION_ACTIVE  		0b00001111
#define SM_FAULTREACTIONACTIVE2 		0b00101111
#define SM_FAULT                		0b00001000
#define SM_FAULT2               		0b00101000

// SatusWord bits :
#define SM_SW_READY_TO_SWITCH_ON    	0x0001
#define SM_SW_SWITCHED_ON          		0x0002
#define SM_SW_OPERATION_ENABLED    		0x0004
#define SM_SW_FAULT               		0x0008
#define SM_SW_VOLTAGE_ENABLED     		0x0010
#define SM_SW_QUICK_STOP           		0x0020
#define SM_SW_SWITCH_ON_DISABLED    	0x0040
#define SM_SW_WARNING             		0x0080
#define SM_SW_REMOTE              		0x0200
#define SM_SW_TARGET_REACHED       		0x0400
#define SM_SW_INTERNAL_LIMIT_ACTIVE 	0x0800

// ControlWord bits :
#define SM_CW_SWITCH_ON        	0x0001
#define SM_CW_ENABLE_VOLTAGE   	0x0002
#define SM_CW_QUICK_STOP       	0x0004
#define SM_CW_ENABLE_OPERATION 	0x0008
#define SM_CW_FAULT_RESET      	0x0080
#define SM_CW_OD_HALT         	0x0100

/*************************************************************/
// CiA402 State Machine Definitions end.

