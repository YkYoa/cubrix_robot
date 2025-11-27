#pragma once 

#include "ar_drive_config.h"
#include "ar_joint_control.h"
#include "ar_utils.h"
#include "ethercat_manager.h"
#include "ar_drive_client.h"

namespace ar_control
{
    class ArDriveControl
    {
    public:
        ArDriveControl(DriveParameter& driveParam, bool uiState = false);
        ~ArDriveControl();
        void InitializeDriveClient(master::EthercatManager* manager, int slaveId);
        
        void AddJoint(JointParameter& jointParam);
        void InitializeDrive();
        void read();
        void write();
        void shutdown();
        void motorOn();
        void motorOff();
        
        // int getInputActualValueToStatus(tVectorS& jointNames, tVectorS& hardwareIds, 
        //                         std::vector<uint32_t>& positionActualValues, std::vector<uint32_t>& velocityActualValues);

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

        inline std::vector<ArJointControl*> getJoints()
        {
            return joints;
        }

        // template <typename T> void jointCmdToPulses(ArJointControl* joint, T* position = nullptr, T* velocity = nullptr);
        template <typename OutputType> void jointCmdToPulses(ArJointControl* joint, OutputType* output);

        std::vector<ArJointControl* > joints; ///< Vector of joint controls associated with this drive

    protected:
        DriveParameter& drive_parameter; ///< Drive parameters for this control
        std::vector<JointParameter> joint_parameters;  ///< Joint parameters associated with this drive
        bool is_ui_;  ///< Flag to indicate if the UI is enabled
        bool is_dual_axis_; ///< Flag to indicate if the drive is dual axis
        int drive_id_;  ///< Unique identifier for the drive
        std::unique_ptr<ar_control::ArDriveClient> ar_client; ///< Pointer to the drive client

        DriveInput* driveInput;   ///< Pointer to the drive input structure
        DriveOutput* driveOutput; ///< Pointer to the drive output structure

    private:
        bool new_setpoint_pending_[LEADSHINE_DRIVER_MAX_JOINT_COUNT] = {false};

    };

}  // namespace ar_control
