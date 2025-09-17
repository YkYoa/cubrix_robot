#pragma once

#include <string>
#include <vector>
#include <urdf/model.h>


enum PortNumber
{
    DISCONNECTED = -1,
    NO_COMM = 0,
    PORT_SOEM = 1,
    PORT_CAN = 2
};

enum ControlModes
{
	ProfilePosition,
    CyclicSynchronousPosition,
};

class JointParameter
{
public:
    JointParameter()
    {
        gear_ratio = 0;
        encoder_res = 0;
        encoder_offset = 0;
        rev_angle_convert_mode = 0;
        log_joint = false;
    }
    std::string joint_name;
    int gear_ratio;             ///< Gear ratio for the joint
    int encoder_res;            ///< Encoder resolution for the joint
    int encoder_offset;         ///< Encoder offset for the joint
    int rev_angle_convert_mode; ///< Mode for converting revolution angle
    bool log_joint;             ///< Flag to indicate if the joint should be logged
    std::shared_ptr<urdf::JointLimits> joint_limits;  ///< Joint limits for the joint get from URDF through luanch file
};


class DriveParameter
{
public:
    DriveParameter()
    {
        slave_id = -1;
        port_id = NO_COMM;
        drive_mode = ProfilePosition;
    }
    int slave_id;
    int drive_id; 
    int drive_mode;
    int port_id;
    std::vector<JointParameter> joint_paramters;
    std::map<std::string, int> joint_name_to_id;
};

class ArDrives
{
public:
    std::vector<DriveParameter> drive_parameters;
	std::vector<std::string> joint_names;
};