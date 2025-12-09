#include <ar_joint_control.h>


namespace ar_control
{
    ArJointControl::ArJointControl(std::string jointName)
    : joint_name(jointName),
      home_encoder_offset(0),
      rewrite_count(0),
      upper_limit(0.0),
      lower_limit(0.0),
      pulse_per_revolution(1.0),
      joint_pos_cmd(0.0),
      joint_pos(0.0),
      joint_vel(0.0),
      joint_vel_cmd(0.0),
      position_actual_value(0),
      velocity_actual_value(0),
      rev_angle_convert_mode(0),
      profile_position(10.0)
    {
    }   
} // namespace ar_control