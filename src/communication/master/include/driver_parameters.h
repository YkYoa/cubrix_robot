#ifndef __DRIVER_PARAMETERS_H__
#define __DRIVER_PARAMETERS_H__

#include <string>
#include <map>
#include <vector>

typedef enum
{
    STRING = 0,
    INTEGER8 = 1,
    UNSIGNED8 = 1,
    INTEGER16 = 2,
    UNSIGNED16 = 2,
    INTEGER32 = 4,
    UNSIGNED32 = 4,
    REAL32 = 4,
} DataBitType;

typedef struct
{
    uint16_t index;
    uint8_t subindex;
    uint8_t data_len;
    uint32_t address;
} ParameterFormat;

class LeadshineParameters
{
public:
    virtual ~LeadshineParameters() = default;

    const std::string DRIVER_TYPE = "Leadshine";

    const ParameterFormat RXPDO1 = {0x1600, 0x00, UNSIGNED16, 0x16000003};
    const ParameterFormat CONTROL_WORD = {0x6040, 0x00, UNSIGNED16, 0x60400000};
    const ParameterFormat TARGET_POSITION = {0x607A, 0x00, INTEGER32, 0x607A0000};
    const ParameterFormat TOUCH_PROBE_FUNCTION = {0x60B8, 0x00, UNSIGNED16, 0x60B80000};
    
    const ParameterFormat TXPDO1 = {0x1A00, 0x00, UNSIGNED16, 0x1A000007};
    const ParameterFormat ERROR_CODE = {0x603F, 0x00, UNSIGNED16, 0x603F0000};
    const ParameterFormat STATUS_WORD = {0x6041, 0x00, UNSIGNED16, 0x60410000};
    const ParameterFormat MODE_OF_OPERATION_DISPLAY = {0x6061, 0x00, INTEGER8, 0x60610000};
    const ParameterFormat ACTUAL_POSITION = {0x6064, 0x00, INTEGER32, 0x60640000};
    const ParameterFormat TOUCH_PROBE_STATUS = {0x60B9, 0x00, UNSIGNED16, 0x60B90000};
    const ParameterFormat TOUCH_PROBE_1_POSITION_VALUE = {0x60BA, 0x01, INTEGER32, 0x60BA0100};
    const ParameterFormat DIGITAL_INPUTS = {0x60FD, 0x00, UNSIGNED32, 0x60FD0000};

    const ParameterFormat SYNC_MANAGER_2_PDO = {0x1C12, 0x00, UNSIGNED8, 0x1C121600};
    const ParameterFormat SYNC_MANAGER_3_PDO = {0x1C13, 0x00, UNSIGNED8, 0x1C131A00};
};

#endif // __DRIVER_PARAMETERS_H__