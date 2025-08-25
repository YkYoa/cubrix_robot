#pragma once 

#include <ar_drive_config.h>
#include <ar_drive_client.h>
#include <ar_joint_control.h>
#include <ar_utils.h>

namespace ar_control
{
    class ArDriveControl
    {
    public:
        ArDriveControl(DriveParameter& driveParam, bool uiState = false);
        // void InitializeDriveClient(ar_master::Protocol* protocolManager, int slaveId);
        void InitializeDriveClient(int slaveId);
        ~ArDriveControl();
        
        void AddJoint(JointParameter& jointParam);
        void InitializeDrive();
        void read();
        void write();
        
        int getInputActualValueToStatus(tVectorS& jointNames, tVectorS& hardwareIds, 
                                std::vector<uint32_t>& positionActualValues, std::vector<uint32_t>& velocityActualValues);

        inline int portId()
        {
            return drive_parameter.port_id;
        }

        inline int driveId()
        {
            return drive_parameter.drive_id;
        }

        inline int driveMode()
        {
            return drive_parameter.drive_mode;
        }

        inline int jointCount()
        {
            return drive_parameter.joint_paramters.size();
        }

        std::vector<ArJointControl*> getJoints()
        {
            return joints;
        }

        std::vector<ArJointControl* > joints; ///< Vector of joint controls associated with this drive

    protected:
        DriveParameter& drive_parameter; ///< Drive parameters for this control
        std::vector<JointParameter> joint_parameters;  ///< Joint parameters associated with this drive
        bool is_ui_;  ///< Flag to indicate if the UI is enabled
        int drive_id_;  ///< Unique identifier for the drive
        // ar_master::Protocol* protocol_manager_;  ///< Protocol manager for communication (to be added later)


    };

}  // namespace ar_control
