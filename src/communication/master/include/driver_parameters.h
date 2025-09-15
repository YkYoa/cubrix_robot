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

    const ParameterFormat CONTROL_WORD = {0x6040, 0x00, UNSIGNED16, 0x60400010};
    const ParameterFormat STATUS_WORD = {0x6041, 0x00, UNSIGNED16, 0x60410010};
    const ParameterFormat MODE_OF_OPERATION = {0x6060, 0x00, INTEGER8, 0x60600008};
    const ParameterFormat MODE_OF_OPERATION_DISPLAY = {0x6061, 0x00, INTEGER8, 0x60610008};
};

#endif // __DRIVER_PARAMETERS_H__