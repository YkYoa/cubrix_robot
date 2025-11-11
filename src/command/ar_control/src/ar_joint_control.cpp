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
} // namespace ar_control