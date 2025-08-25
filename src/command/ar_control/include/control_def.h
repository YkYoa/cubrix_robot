#pragma once 


#include <map>
#include <string>
#include <stdint.h>


namespace ar_control
{
    enum ControlCommandType
    {
        CTR_CMD_SEND,
        CTR_CMD_WAIT,
        CTR_CMD_SEND_WAIT,
    };

    enum HostCommand{
        //Adding later
        PLANNING,
        UNKNOWN_CMD
    };

    enum ErrorCode
    {
        //Add later due to driver API
    };

    static std::map<int, std::string> HostCommandToName = {
        {PLANNING, "PLANNING"},
        {UNKNOWN_CMD, "UNKNOWN_CMD"}
    };

    enum ControlErrorCode
    {
        FAILURE = -1,
        SUCCESS,
        HALT
    };

} // namespace ar_control