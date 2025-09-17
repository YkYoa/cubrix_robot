#include <ar_joint_control.h>


namespace ar_control
{
    ArJointControl::ArJointControl(std::string jointName)
    {
        joint_name = jointName;

        profile_position = 10;
        rewrite_count = 0;
        pulse_per_revolution = 1;
    }   

    void ArJointControl::getInputActualValueToStatus(std::string& jointName, std::string& hardwareId, 
                                                            uint32_t& positionActualValue, uint32_t& velocityActualValue)
    {
        jointName = joint_name;
        hardwareId = hardware_id;
        positionActualValue = position_actual_value;
        velocityActualValue = velocity_actual_value;
    }
} // namespace ar_control