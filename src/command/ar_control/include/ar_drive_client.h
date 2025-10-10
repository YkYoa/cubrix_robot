#ifndef _AR_DRIVE_CLIENT_H_
#define _AR_DRIVE_CLIENT_H_

#include <ethercat_manager.h>

namespace ar_control
{
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

    struct DualJointCyclicInput : DriveInput
    {
        uint16 error_code_1;
        uint16 status_word_1;
        uint8 mode_of_operation_display_1;
        int32 actual_position_1;
        uint16 touch_probe_status_1;
        int32 touch_probe_1_positive_value_1;
        uint32 digital_inputs_1;

        uint16 error_code_2;
        uint16 status_word_2;
        uint8 mode_of_operation_display_2;
        int32 actual_position_2;
        uint16 touch_probe_status_2;
        int32 touch_probe_1_positive_value_2;
        uint32 digital_inputs_2;
    };

    struct DualJointCyclicOutput : DriveOutput
    {
        uint16 control_word_1;
        int32 target_position_1;
        uint32 touch_probe_function_1;

        uint16 control_word_2;
        int32 target_position_2;
        uint32 touch_probe_function_2;
    };

    class ArDriveClient
    {
    public:
        ArDriveClient(master::EthercatManager& manager, int slaveId);
        ~ArDriveClient ();

        void readInputs(SingleJointCyclicInput* input);
        void readOutputs(SingleJointCyclicOutput* output);
        void writeOutputs(SingleJointCyclicOutput* output);

        void readInputs(DualJointCyclicInput* input);
        void readOutputs(DualJointCyclicOutput* output);
        void writeOutputs(DualJointCyclicOutput* output);

        template <typename T, typename U> void resetFaultSingleJoint(T* input, U* output);
        template <typename T, typename U> void resetFaultDualJoint(T* input, U* output);
        template <typename T, typename U> void motorOn(T* input, U* output);
        template <typename T, typename U> void motorOff(T* input, U* output);

		inline master::EthercatManager& getManager()
		{
			return manager_;
		}

        std::shared_ptr<master::DriverInfo> driver_info_;

    private:
        master::EthercatManager& manager_;
        // Private members and methods
        template <typename T> void ErrorHandling(const T* input);

        int input_map_size;
        int output_map_size;

        const std::map<uint16_t, std::string>* error_maps;
        const int slave_id_;
    };

}



#endif