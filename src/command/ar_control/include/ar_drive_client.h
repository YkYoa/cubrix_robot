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
        uint8 displayed_mode_of_operation;
        int32 actual_position;
        int32 actual_velocity;
    };
    
    struct SingleJointCyclicOutput : DriveOutput
    {
        uint16 error_code;
        uint16 control_word;
        uint8 operation_mode;
        int32 target_position;
        int32 target_velocity;
    };

    class ArDriveClient
    {
    public:
        // Constructor
        ArDriveClient(std::shared_ptr<master::EthercatManager> manager, int slaveId);
        ~ArDriveClient ();

        void readInputs(SingleJointCyclicInput* input);
        void readOutputs(SingleJointCyclicOutput* output);
        void writeOutputs(SingleJointCyclicOutput* output);

        template <typename T, typename U> void reset(T* input, U* output);
        template <typename T, typename U> void motorOn(T* input, U* output);
        template <typename T, typename U> void motorOff(T* input, U* output);

        inline master::EthercatManager& getManager() { return *manager_; }

        std::shared_ptr<master::DriverInfo> driver_info_;

    private:
        std::shared_ptr<master::EthercatManager> manager_;
        // Private members and methods
        template <typename T> void ErrorHandling(const T* input);

        int input_map_size;
        int output_map_size;

        const std::map<uint16_t, std::string>* error_maps;
        int slave_id_;
    };

}



#endif