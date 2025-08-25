#pragma once

#include <string>
#include <vector>
#include <memory>

namespace ar_control
{
    class ArJointControl
    {
    public:
        ArJointControl(std::string jointName);
        ~ArJointControl() {};

        void getInputActualValueToStatus(std::string &jointName, std::string &hardwareId, 
                                            uint32_t &positionActualValue, uint32_t &velocityActualValue);

    public:
        std::string joint_name;   ///< Name of the joint
        std::string hardware_id;  ///< Hardware ID for the joint
        int client_id;            ///< Client ID for the joint
        int home_encoder_offset;  ///< Home encoder offset for the joint
        int rewrite_count;        ///< Rewrite count for the joint

        double upper_limit;          ///< Upper limit for the joint
        double lower_limit;          ///< Lower limit for the joint
        double pulse_per_revolution; ///< Pulse per revolution for the joint

        double joint_pos_cmd; ///< Pos Command for the joint
        double joint_pos;     ///< Position of the joint
        double joint_vel;     ///< Velocity of the joint
        double joint_vel_cmd; ///< Commanded velocity for the joint

        // Input for drive client we use profile position mode
        uint32_t position_actual_value; ///< Actual position value for the joint   
        uint32_t velocity_actual_value; ///< Actual velocity value for the joint
        int rev_angle_convert_mode; ///< Mode for converting revolution angle

        double profile_position; ///< Profile position for the joint

        friend class ArDriveControl; ///< Allow ArDriveControl to access private members
    };

} // namespace ar_control
