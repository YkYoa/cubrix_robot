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
        ArDriveClient(master::EtherCatManager* manager, int slaveId);
        ~ArDriveClient ();

        void readInputs(SingleJointCyclicInput* input);
        void readOutputs(SingleJointCyclicOutput* output);
        void writeOutputs(SingleJointCyclicOutput* output);

        template <typename T, typename U> void reset(T* input, U* output);
        template <typename T, typename U> void motorOn(T* input, U* output);
        template <typename T, typename U> void motorOff(T* input, U* output);

        inline master::EtherCatManager* getManager() 
        { 
            return manager_; 
        }

        master::DriveInfo driver_info_;
    private:
        // Private members and methods
        template <typename T> vodi ErrorHandling(const T* input);

        master::EthercatManager& manager_;
        
    };

}



#endif