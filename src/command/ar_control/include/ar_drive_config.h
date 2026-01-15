#pragma once

#include <string>
#include <vector>
#include <urdf/model.h>

/**
 * @brief Port number enumeration
 */
enum PortNumber
{
    DISCONNECTED = -1, ///< Port is disconnected
    NO_COMM = 0,       ///< No communication
    PORT_SOEM = 1,     ///< SOEM EtherCAT port
    PORT_CAN = 2       ///< CAN bus port
};

/**
 * @brief Control mode enumeration
 */
enum ControlModes
{
    ProfilePosition,              ///< Profile position mode
    CyclicSynchronousPosition,    ///< Cyclic synchronous position mode
};

/**
 * @brief Joint parameter configuration
 * 
 * Contains all parameters needed to configure a single joint
 * including gear ratio, encoder settings, and limits.
 */
class JointParameter
{
public:
    /**
     * @brief Default constructor
     * 
     * Initializes all parameters to default values.
     */
    JointParameter()
    {
        gear_ratio = 0;
        encoder_res = 0;
        encoder_offset = 0;
        rev_angle_convert_mode = 0;
        log_joint = false;
    }
    std::string joint_name;
    int gear_ratio;                                  ///< Gear ratio for the joint
    int encoder_res;                                 ///< Encoder resolution for the joint
    int encoder_offset;                              ///< Encoder offset for the joint
    int rev_angle_convert_mode;                      ///< Mode for converting revolution angle
    bool log_joint;                                  ///< Flag to indicate if the joint should be logged
    std::shared_ptr<urdf::JointLimits> joint_limits; ///< Joint limits for the joint get from URDF through luanch file
};

/**
 * @brief Drive parameter configuration
 * 
 * Contains all parameters needed to configure a drive controller
 * including slave ID, port configuration, and associated joints.
 */
class DriveParameter
{
public:
    /**
     * @brief Default constructor
     * 
     * Initializes all parameters to default values.
     */
    DriveParameter()
    {
        slave_id = -1;
        is_dual_axis = false;
        port_id = NO_COMM;
        drive_mode = CyclicSynchronousPosition;
    }
    int slave_id;
    int drive_id;
    int drive_mode;
    int port_id;
    bool is_dual_axis;
    std::vector<JointParameter> joint_paramters;
    std::map<std::string, int> joint_name_to_id;
};

/**
 * @brief Container for all drive configurations
 * 
 * Holds all drive parameters and joint names for the robot system.
 */
class ArDrives
{
public:
    std::vector<DriveParameter> drive_parameters; ///< Vector of drive parameter configurations
    std::vector<std::string> joint_names;          ///< Vector of joint names
};