#pragma once

#include "ar_drive_config.h"
#include "ar_joint_control.h"
#include "ar_utils.h"
#include "ethercat_manager.h"
#include "ar_drive_client.h"

#ifndef NO_ETHERCAT
#include "igh_manager.hpp"
#endif

namespace ar_control
{
    /**
     * @brief Control interface for a single drive/motor controller
     * 
     * Manages communication with a drive controller, handles joint control,
     * and provides read/write operations for cyclic data exchange.
     */
    class ArDriveControl
    {
    public:
        /**
         * @brief Constructor
         * @param driveParam Drive parameters configuration
         * @param uiState Whether UI mode is enabled
         */
        ArDriveControl(DriveParameter &driveParam, bool uiState = false);
        
        /**
         * @brief Destructor
         */
        ~ArDriveControl();
        
        /**
         * @brief Initialize the drive client with EtherCAT manager
         * @param manager Pointer to EtherCAT master interface
         * @param slaveId EtherCAT slave ID for this drive
         */
        void InitializeDriveClient(master::EthercatMasterInterface *manager, int slaveId);

        /**
         * @brief Add a joint to this drive
         * @param jointParam Joint parameters configuration
         */
        void AddJoint(JointParameter &jointParam);
        
        /**
         * @brief Initialize the drive (configure PDOs, set initial states)
         */
        void InitializeDrive();
        
        /**
         * @brief Read current joint states from the drive
         */
        void read();
        
        /**
         * @brief Write joint commands to the drive
         */
        void write();
        
        /**
         * @brief Shutdown the drive (disable motors, clean up)
         */
        void shutdown();
        
        /**
         * @brief Enable motors on this drive
         */
        void motorOn();
        
        /**
         * @brief Disable motors on this drive
         */
        void motorOff();

        // int getInputActualValueToStatus(tVectorS& jointNames, tVectorS& hardwareIds,
        //                         std::vector<uint32_t>& positionActualValues, std::vector<uint32_t>& velocityActualValues);

        /**
         * @brief Get the EtherCAT port ID
         * @return Port ID
         */
        inline int portId()
        {
            return drive_parameter.port_id;
        }

        /**
         * @brief Get the drive ID
         * @return Drive ID
         */
        inline int driveId()
        {
            return drive_parameter.drive_id;
        }

        /**
         * @brief Get the drive mode
         * @return Drive mode value
         */
        inline int driveMode()
        {
            return drive_parameter.drive_mode;
        }

        /**
         * @brief Get the number of joints on this drive
         * @return Joint count
         */
        inline int jointCount()
        {
            return drive_parameter.joint_paramters.size();
        }

        /**
         * @brief Get all joint controls for this drive
         * @return Vector of joint control pointers
         */
        inline std::vector<ArJointControl *> getJoints()
        {
            return joints;
        }

        /**
         * @brief Convert joint command to drive pulses/encoder counts
         * @tparam OutputType Output data type (SingleJointCyclicOutput, DualJointCyclicOutput, etc.)
         * @param joint Joint control to convert
         * @param output Output structure to fill
         */
        template <typename OutputType>
        void jointCmdToPulses(ArJointControl *joint, OutputType *output);

        std::vector<ArJointControl *> joints; ///< Vector of joint controls associated with this drive

    protected:
        DriveParameter &drive_parameter;                      ///< Drive parameters for this control
        std::vector<JointParameter> joint_parameters;         ///< Joint parameters associated with this drive
        bool is_ui_;                                          ///< Flag to indicate if the UI is enabled
        bool is_dual_axis_;                                   ///< Flag to indicate if the drive is dual axis
        int drive_id_;                                        ///< Unique identifier for the drive
        std::unique_ptr<ar_control::ArDriveClient> ar_client; ///< Pointer to the drive client
#ifndef NO_ETHERCAT
        master::IghManager *igh_manager_;                     ///< Pointer to IGH manager for DC sync control
#endif
        int slave_id_;                                        ///< Slave ID for this drive

        DriveInput *driveInput;   ///< Pointer to the drive input structure
        DriveOutput *driveOutput; ///< Pointer to the drive output structure

    private:
        bool new_setpoint_pending_[LEADSHINE_DRIVER_MAX_JOINT_COUNT] = {false};
        bool initialization_in_progress_ = false; ///< Prevents cyclic write interference during init
        bool shutdown_complete_ = false;          ///< Guards against double shutdown
    };

} // namespace ar_control
