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

    class ArDriveClient
    {
    public:
        ArDriveClient(master::EthercatManager& manager, int slaveId);
        ~ArDriveClient ();

        void readInputs(SingleJointCyclicInput* input);
        void readOutputs(SingleJointCyclicOutput* output);
        void writeOutputs(SingleJointCyclicOutput* output);

        template <typename T, typename U> void reset(T* input, U* output);
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