#ifndef _AR_DRIVE_CLIENT_H_
#define _AR_DRIVE_CLIENT_H_

#include <ethercat_manager.h>

namespace ar_control
{
#define LEADSHINE_DRIVER_MAX_JOINT_COUNT 2

    struct DriveInput
    {
    };

    struct DriveOutput
    {
    };

    struct SingleJointCyclicInput : DriveInput
    {
        uint16 error_code;
        uint16 status_word;
        uint8 mode_of_operation_display;
        int32 actual_position;
        uint16 touch_probe_status;
        int32 touch_probe_1_positive_value;
        uint32 digital_inputs;
    };

    struct SingleJointCyclicOutput : DriveOutput
    {
        uint16 control_word;
        int32 target_position;
        uint32 touch_probe_function;
    };

    struct TxPDO
    {
        uint16 control_word;
        int32 target_position;
        uint32 touch_probe_function;
    };

    struct RxPDO
    {
        uint16 error_code;
        uint16 status_word;
        uint8 mode_of_operation_display;
        int32 actual_position;
        uint16 touch_probe_status;
        int32 touch_probe_positive_value;
        uint32 digital_inputs;
    };

    struct DualJointCyclicInput : DriveInput
    {
        RxPDO axis[LEADSHINE_DRIVER_MAX_JOINT_COUNT];
    };

    struct DualJointCyclicOutput : DriveOutput
    {
        TxPDO axis[LEADSHINE_DRIVER_MAX_JOINT_COUNT];
    };

    struct TxProFileOutput
    {
        uint16_t control_word;
        int32_t profile_target_position;
        uint32_t profile_velocity;
        uint32_t profile_target_acceleration;
        uint32_t profile_target_deceleration;
        uint8_t mode_of_operation;
    };

    struct DualJointProFileOutput : DriveOutput
    {
        TxProFileOutput axis[LEADSHINE_DRIVER_MAX_JOINT_COUNT];
    };

    struct DualJointProFileInput : DriveInput
    {
        RxPDO axis[LEADSHINE_DRIVER_MAX_JOINT_COUNT];
    };

    /**
     * @brief Client interface for EtherCAT drive communication
     * 
     * Handles low-level communication with drive controllers via EtherCAT.
     * Supports single and dual-axis drives with cyclic and profile position modes.
     */
    class ArDriveClient
    {
    public:
        /**
         * @brief Constructor
         * @param manager Reference to EtherCAT master interface
         * @param slaveId EtherCAT slave ID for this drive
         */
        ArDriveClient(master::EthercatMasterInterface &manager, int slaveId);
        
        /**
         * @brief Destructor
         */
        ~ArDriveClient();

        /**
         * @brief Read input data from single-axis drive
         * @param input Pointer to input structure to fill
         */
        void readInputs(SingleJointCyclicInput *input);
        
        /**
         * @brief Read output data from single-axis drive
         * @param output Pointer to output structure to fill
         */
        void readOutputs(SingleJointCyclicOutput *output);
        
        /**
         * @brief Write output data to single-axis drive
         * @param output Pointer to output structure containing commands
         */
        void writeOutputs(SingleJointCyclicOutput *output);

        /**
         * @brief Read input data from dual-axis drive
         * @param input Pointer to input structure to fill
         */
        void readInputs(DualJointCyclicInput *input);
        
        /**
         * @brief Read output data from dual-axis drive
         * @param output Pointer to output structure to fill
         */
        void readOutputs(DualJointCyclicOutput *output);
        
        /**
         * @brief Write output data to dual-axis drive
         * @param output Pointer to output structure containing commands
         */
        void writeOutputs(DualJointCyclicOutput *output);

        /**
         * @brief Read input data from dual-axis profile drive
         * @param input Pointer to input structure to fill
         */
        void readInputs(DualJointProFileInput *input);
        
        /**
         * @brief Read output data from dual-axis profile drive
         * @param output Pointer to output structure to fill
         */
        void readOutputs(DualJointProFileOutput *output);
        
        /**
         * @brief Write output data to dual-axis profile drive
         * @param output Pointer to output structure containing commands
         */
        void writeOutputs(DualJointProFileOutput *output);

        /**
         * @brief Reset fault on single-axis drive
         * @tparam T Input data type
         * @tparam U Output data type
         * @param input Input structure
         * @param output Output structure to modify
         */
        template <typename T, typename U>
        void resetFaultSingleJoint(T *input, U *output);
        
        /**
         * @brief Reset fault on dual-axis drive
         * @tparam T Input data type
         * @tparam U Output data type
         * @param input Input structure
         * @param output Output structure to modify
         */
        template <typename T, typename U>
        void resetFaultDualJoint(T *input, U *output);
        
        /**
         * @brief Enable motors on dual-axis drive
         * @tparam T Input data type
         * @tparam U Output data type
         * @param input Input structure
         * @param output Output structure to modify
         */
        template <typename T, typename U>
        void dualMotorOn(T *input, U *output);
        
        /**
         * @brief Enable motor on single-axis drive
         * @tparam T Input data type
         * @tparam U Output data type
         * @param input Input structure
         * @param output Output structure to modify
         */
        template <typename T, typename U>
        void singleMotorOn(T *input, U *output);
        
        /**
         * @brief Disable motors on dual-axis drive
         * @tparam T Input data type
         * @tparam U Output data type
         * @param input Input structure
         * @param output Output structure to modify
         */
        template <typename T, typename U>
        void dualMotorOff(T *input, U *output);
        
        /**
         * @brief Disable motor on single-axis drive
         * @tparam T Input data type
         * @tparam U Output data type
         * @param input Input structure
         * @param output Output structure to modify
         */
        template <typename T, typename U>
        void singleMotorOff(T *input, U *output);

        /**
         * @brief Get the EtherCAT master interface
         * @return Reference to the master interface
         */
        inline master::EthercatMasterInterface &getManager()
        {
            return manager_;
        }

        std::shared_ptr<master::DriverInfo> driver_info_;

    private:
        master::EthercatMasterInterface &manager_; ///< Reference to EtherCAT master
        
        /**
         * @brief Handle drive errors
         * @tparam T Input data type
         * @param input Input structure containing error information
         */
        template <typename T>
        void ErrorHandling(const T *input);

        int input_map_size;
        int output_map_size;

        const std::map<uint16_t, std::string> *error_maps;
        const int slave_id_;
    };

}

#endif