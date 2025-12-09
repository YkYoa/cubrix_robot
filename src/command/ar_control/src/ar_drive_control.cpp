#include <ar_drive_control.h>

double LIMIT_SAFETY_OFFSET = 0.0001; ///< Safety offset for joint limits
bool ENABLE_PRINT = true;            ///< Flag to enable or disable drive logging
FILE* g_debug_log = nullptr;         ///< Log file for debug output

namespace ar_control
{
    ArDriveControl::ArDriveControl(DriveParameter &driveParm, bool uiState) : drive_parameter(driveParm), is_ui_(uiState)
    {
        drive_id_ = driveParm.drive_id;
        is_dual_axis_ = driveParm.is_dual_axis;
        driveInput = nullptr;
        driveOutput = nullptr;
        igh_manager_ = nullptr;
        slave_id_ = -1;
    }

    void ArDriveControl::InitializeDriveClient(master::EthercatMasterInterface *manager, int slaveId)
    {
        printf(COLOR_BLUE "\n[Ar Drive Control] InitializeDriveClient called for drive %d, slaveId %d" COLOR_RESET,
               drive_id_, slaveId);

        ar_client.reset();
        if (manager)
        {
            ar_client = std::make_unique<ArDriveClient>(*manager, slaveId);
            printf(COLOR_BLUE "\n[INIT] Drive ID %d -> Slave ID %d | Input: %d bytes | Output: %d bytes" COLOR_RESET,
                   drive_id_, slaveId,
                   manager->getInputBits(slaveId) / 8,
                   manager->getOutputBits(slaveId) / 8);
            fflush(stdout);
            
            igh_manager_ = dynamic_cast<master::IghManager*>(manager);
            slave_id_ = slaveId;
        }
        else
        {
            printf(COLOR_RED "\n[Ar Drive Control] Manager is null for drive %d" COLOR_RESET, drive_id_);
        }
    }

    void ArDriveControl::AddJoint(JointParameter &jointParam)
    {        
        ArJointControl *joint = new ArJointControl(jointParam.joint_name);
        printf(COLOR_BLUE "\n[Ar Drive Control] Drive %d add joint: %s" COLOR_RESET, drive_id_, jointParam.joint_name.c_str());
        joint->rev_angle_convert_mode = jointParam.rev_angle_convert_mode;

        if (jointParam.joint_limits != NULL)
        {
            joint->upper_limit = jointParam.joint_limits->upper - LIMIT_SAFETY_OFFSET;
            joint->lower_limit = jointParam.joint_limits->lower + LIMIT_SAFETY_OFFSET;
        }
        else
        {
            joint->upper_limit = 0.0;
            joint->lower_limit = 0.0;
        }

        joints.push_back(joint);

        joint->joint_pos_cmd = 0.0;
        joint->joint_pos = 0.0;
        joint->joint_vel = 0.0;
        joint->joint_vel_cmd = 0.0;
        joint->position_actual_value = 0;
        joint->velocity_actual_value = 0;

        if (ar_client == nullptr)
        {
            joint->home_encoder_offset = 0;
        }

        if (abs(jointParam.encoder_offset) > 4000)
        {
            printf(COLOR_RED "\n [Ar Drive Control] Joint '%s' has a large encoder offset: %d. Please check Joint Parameters." COLOR_RESET,
                   jointParam.joint_name.c_str(), jointParam.encoder_offset);

            joint->home_encoder_offset = 0;
        }
        else
        {
            joint->home_encoder_offset = jointParam.encoder_offset;
        }

        if (jointParam.gear_ratio && jointParam.encoder_res)
        {
            joint->pulse_per_revolution = jointParam.gear_ratio * jointParam.encoder_res;
            if (!joint->rev_angle_convert_mode)
            {
                joint->pulse_per_revolution /= (2 * M_PI);
            }
            printf("\n [Ar Drive Control ]Joint '%s' gear ratio: %d, encoder resolution: %d, pulse per rev: %5.1f",
                   jointParam.joint_name.c_str(), jointParam.gear_ratio, jointParam.encoder_res, joint->pulse_per_revolution);
        }

        printf("Joint '%s' added on drive %d, encoder offset: %d",
               jointParam.joint_name.c_str(), drive_id_, joint->home_encoder_offset);
        fflush(stdout);
    }

    void ArDriveControl::InitializeDrive()
    {
        printf(COLOR_DARKYELLOW "\n[Ar Drive Control] ---------------------- Initializing drive %d----------------------" COLOR_RESET,
               drive_id_);

        if (!ar_client)
            return;

        const uint16_t enable_sequence[] = {
            0x0006, // Switch On Disabled → Ready to Switch On
            0x0007, // Ready to Switch On → Switched On
            0x000F // Switched On → Operation Enabled
            // 0x001F  // Switched On → Operation Enabled
        };

        if (drive_parameter.drive_mode == CyclicSynchronousPosition)
        {
            if (drive_parameter.is_dual_axis == true)
            {
                DualJointCyclicInput *input = new DualJointCyclicInput();
                DualJointCyclicOutput *output = new DualJointCyclicOutput();
                driveInput = input;
                driveOutput = output;
                memset(output, 0, sizeof(DualJointCyclicOutput));

                if (ar_client != nullptr)
                {
                    ar_client->resetFaultDualJoint(input, output);
                    for (size_t i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
                    {
                        output->axis[i].target_position = input->axis[i].actual_position;
                        usleep(1000);
                    }
                    ar_client->writeOutputs(output);


                    for (int i = 0; i < 3; i++)
                    {
                        usleep(20000);
                        for (size_t j = 0; j < LEADSHINE_DRIVER_MAX_JOINT_COUNT; j++)
                        {
                            output->axis[j].control_word = enable_sequence[i];
                        }
                        ar_client->writeOutputs(output);
                    }
                }
            }
            else
            {
                SingleJointCyclicInput *input = new SingleJointCyclicInput();
                SingleJointCyclicOutput *output = new SingleJointCyclicOutput();
                driveInput = input;
                driveOutput = output;
                memset(output, 0, sizeof(SingleJointCyclicOutput));

                if (ar_client != nullptr)
                {
                    ar_client->resetFaultSingleJoint(input, output);
                    output->target_position = input->actual_position;
                    ar_client->writeOutputs(output);

                    for (int i = 0; i < 3; i++)
                    {
                        usleep(20000);
                        output->control_word = enable_sequence[i];
                        ar_client->writeOutputs(output);
                    }
                }
            }
        }
        else if (drive_parameter.drive_mode == ProfilePosition)
        {
            DualJointProFileOutput *output = new DualJointProFileOutput();
            DualJointProFileInput *input = new DualJointProFileInput();
            driveInput = input;
            driveOutput = output;

            ar_client->resetFaultDualJoint(input, output);
            for (size_t i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
            {
                output->axis[i].profile_target_position = input->axis[i].actual_position;
                output->axis[i].mode_of_operation = 1; // Profile Position Mode
                usleep(1000);
            }
            ar_client->writeOutputs(output);

            memset(output, 0, sizeof(DualJointProFileOutput));

            for (int i = 0; i < 3; i++)
            {
                usleep(20000);
                for (size_t j = 0; j < LEADSHINE_DRIVER_MAX_JOINT_COUNT; j++)
                {
                    output->axis[j].control_word = enable_sequence[i];
                }
                ar_client->writeOutputs(output);
            }
        }
        return;
    }

    ArDriveControl::~ArDriveControl()
    {
        shutdown();
        for (auto joint : joints)
        {
            delete joint;
        }
        joints.clear();

        if (driveInput)
            delete driveInput;
        if (driveOutput)
            delete driveOutput;

        ar_client.reset();
    }

    void ArDriveControl::shutdown()
    {
        if (ar_client == nullptr)
            return;

        printf(COLOR_DARKYELLOW "\n[Ar Drive Control] Shutting down drive %d" COLOR_RESET, drive_id_);

        if (drive_parameter.is_dual_axis == true)
        {
            DualJointCyclicInput *input = (DualJointCyclicInput *)driveInput;
            DualJointCyclicOutput *output = (DualJointCyclicOutput *)driveOutput;
            ar_client->readInputs(input);
            for (size_t i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
            {
                printf("\n error_code = %04x, status_word %04x, operation_mode = %2d", input->axis[i].error_code, input->axis[i].status_word, input->axis[i].mode_of_operation_display);
            }

            ar_client->resetFaultDualJoint(input, output);
            ar_client->dualMotorOff(input, output);
        }
        else
        {
            SingleJointCyclicInput *input = (SingleJointCyclicInput *)driveInput;
            SingleJointCyclicOutput *output = (SingleJointCyclicOutput *)driveOutput;
            ar_client->readInputs(input);

            printf("\n error_code = %04x, status_word %04x, operation_mode = %2d",
                   input->error_code, input->status_word, input->mode_of_operation_display);

            ar_client->resetFaultSingleJoint(input, output);
            ar_client->singleMotorOff(input, output);
        }
    }

    // template <typename T> void ArDriveControl::jointCmdToPulses(ArJointControl* joint, T* position, T* velocity)
    // {
    // 	if(position)
    // 		*position = int32(joint->joint_pos_cmd * joint->pulse_per_revolution);
    // 	if(velocity)
    // 		*velocity = int32(joint->velocity_actual_value * joint->pulse_per_revolution);
    // }

    template <>
    void ArDriveControl::jointCmdToPulses<SingleJointCyclicOutput>(ArJointControl *joint, SingleJointCyclicOutput *output)
    {
        output->target_position = int32(joint->joint_pos_cmd * joint->pulse_per_revolution);
    }

    template <>
    void ArDriveControl::jointCmdToPulses<DualJointCyclicOutput>(ArJointControl *joint, DualJointCyclicOutput *output)
    {
        (void)joint;
        for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
        {
            output->axis[i].target_position = int32(joints[i]->joint_pos_cmd * joints[i]->pulse_per_revolution);
        }
    }

    template <>
    void ArDriveControl::jointCmdToPulses<DualJointProFileOutput>(ArJointControl *joint, DualJointProFileOutput *output)
    {
        (void)joint;
        for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
        {
            output->axis[i].profile_target_position = int32(joints[i]->joint_pos_cmd * joints[i]->pulse_per_revolution);
        }
    }


    void ArDriveControl::write()
    {
        if (ar_client == nullptr)
        {
            return;
        }

        ArJointControl *joint = joints[0];
        if (drive_parameter.drive_mode == CyclicSynchronousPosition)
        {
            if (is_dual_axis_)
            {
                DualJointCyclicOutput *output = (DualJointCyclicOutput *)driveOutput;
                jointCmdToPulses(joint, output);
                ar_client->writeOutputs(output);

                if (ENABLE_PRINT){
                    if (!g_debug_log){
                        g_debug_log = fopen("/tmp/ar_drive_debug.log", "a");
                    }

                    if (g_debug_log){
                        fprintf(g_debug_log, COLOR_BLUE "\n  [WRITE - Dual Axis]");
                        for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; ++i){
                            fprintf(g_debug_log, "\n  Axis[%zu] | Control Word: 0x%04X | Target Pos: %d | Cmd Pos: %6.3f",
                                    i,
                                    output->axis[i].control_word,
                                    output->axis[i].target_position,
                                    joints[i]->joint_pos_cmd);
                        }
                        fprintf(g_debug_log, "\n" COLOR_RESET);
                        fflush(g_debug_log);
                    }
                }
            }
            else
            {
                SingleJointCyclicOutput *output = (SingleJointCyclicOutput *)driveOutput;
                jointCmdToPulses(joint, output);
                ar_client->writeOutputs(output);

                if (ENABLE_PRINT)
                {
                    if (!g_debug_log){
                        g_debug_log = fopen("/tmp/ar_drive_debug.log", "a");
                    }

                    const char* msg = "\n  [WRITE - Single Axis] Drive: %d | Control Word: 0x%04X | Target Pos: %d | Cmd Pos: %6.3f";
                    
                    if(g_debug_log) {
                        fprintf(g_debug_log, msg, drive_id_, output->control_word, output->target_position, joint->joint_pos_cmd);
                        fprintf(g_debug_log, "\n");
                        fflush(g_debug_log);
                    }
                }
            }
        }
        else
        {
            DualJointProFileOutput *output = (DualJointProFileOutput *)driveOutput;
            DualJointProFileInput *input = (DualJointProFileInput *)driveInput;

            static bool test_initialized = false;
            static int test_phase = 0;
            static int test_counter = 0;

            static bool new_setpoint_pending[LEADSHINE_DRIVER_MAX_JOINT_COUNT] = {false};

            if (!test_initialized)
            {
                printf(COLOR_GREEN "\n[TEST] Starting Profile Position Mode Test for Drive %d" COLOR_RESET, drive_id_);
                test_initialized = true;
            }

            test_counter++;
            if (test_counter >= 500) // Change position every 2 seconds (500 cycles at 250Hz)
            {
                test_phase = (test_phase + 1) % 4;
                test_counter = 0;

                switch (test_phase)
                {
                case 0:
                    for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
                    {
                        joints[i]->joint_pos_cmd = 0.0;
                    }
                    printf(COLOR_GREEN "\n[TEST] Moving to position 0: All joints at 0 rad" COLOR_RESET);
                    break;

                case 1:
                    if (joints.size() > 0)
                        joints[0]->joint_pos_cmd = 0.7;
                    if (joints.size() > 1)
                        joints[1]->joint_pos_cmd = -0.5;
                    printf(COLOR_GREEN "\n[TEST] Moving to position 1: J1=0.5 rad, J2=-0.3 rad" COLOR_RESET);
                    break;

                case 2:
                    if (joints.size() > 0)
                        joints[0]->joint_pos_cmd = -0.5;
                    if (joints.size() > 1)
                        joints[1]->joint_pos_cmd = 0.7;
                    printf(COLOR_GREEN "\n[TEST] Moving to position 2: J1=-0.5 rad, J2=0.3 rad" COLOR_RESET);
                    break;

                case 3:
                    if (joints.size() > 0)
                        joints[0]->joint_pos_cmd = 0.4;
                    if (joints.size() > 1)
                        joints[1]->joint_pos_cmd = -0.4;
                    printf(COLOR_GREEN "\n[TEST] Moving to position 3: J1=0.2 rad, J2=-0.2 rad" COLOR_RESET);
                    break;
                }

                for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
                {
                    new_setpoint_pending[i] = true;
                }
            }

            jointCmdToPulses(joint, output);

            for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
            {
                output->axis[i].mode_of_operation = 1;               // Profile Position Mode
                output->axis[i].profile_velocity = 100000;           // Profile velocity
                output->axis[i].profile_target_acceleration = 10000; // Profile acceleration
                output->axis[i].profile_target_deceleration = 10000; // Profile deceleration

                uint16_t control_word = 0x000F; // Base: operation enabled

                bool setpoint_ack = (input->axis[i].status_word & (1 << 12)) != 0;

                if (new_setpoint_pending[i] && setpoint_ack)
                {
                    new_setpoint_pending[i] = false;
                    printf(COLOR_YELLOW "\n[TEST] Axis[%zu] setpoint acknowledged" COLOR_RESET, i);
                }

                if (new_setpoint_pending[i])
                {
                    control_word |= (1 << 4); // Bit 4: New setpoint
                    control_word |= (1 << 5); // Bit 5: Change set immediately
                    printf(COLOR_CYAN "\n[TEST] Axis[%zu] triggering new setpoint (CW: 0x%04X)" COLOR_RESET, i, control_word);
                }

                output->axis[i].control_word = control_word;
            }

            ar_client->writeOutputs(output);

            static int write_counter = 0;
            if (++write_counter >= 250 && ENABLE_PRINT)
            {
                printf(COLOR_BLUE "\n  [WRITE - PP Mode Dual Axis]");
                for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; ++i)
                {
                    printf("\n  Axis[%zu] | Control: 0x%04X | Mode: %d | Target: %d | Cmd: %6.3f rad | Pending: %s",
                           i,
                           output->axis[i].control_word,
                           output->axis[i].mode_of_operation,
                           output->axis[i].profile_target_position,
                           joints[i]->joint_pos_cmd,
                           new_setpoint_pending[i] ? "YES" : "NO");
                }
                printf("\n" COLOR_RESET);
                fflush(stdout);
                write_counter = 0;
            }
        }
    }
    void ArDriveControl::read()
    {
        if (ar_client == nullptr)
        {
            for (auto joint : joints)
            {
                joint->joint_pos = joint->joint_pos_cmd;
                joint->joint_vel = joint->joint_vel_cmd;
                joint->position_actual_value = 0;
                joint->velocity_actual_value = 0;
            }

            return;
        }

        if (drive_parameter.drive_mode == CyclicSynchronousPosition)
        {
            if (is_dual_axis_)
            {
                DualJointCyclicInput *input = (DualJointCyclicInput *)driveInput;
                ar_client->readInputs(input);

                for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
                {
                    joints[i]->position_actual_value = input->axis[i].actual_position;
                    int32_t actual_pos = static_cast<int32_t>(input->axis[i].actual_position);
                    joints[i]->joint_pos = (actual_pos - joints[i]->home_encoder_offset) / joints[i]->pulse_per_revolution;
                }

                if (ENABLE_PRINT)
                {
                    if(!g_debug_log) {
                        g_debug_log = fopen("/tmp/ar_drive_debug.log", "a");
                    }
                    
                    if(g_debug_log) {
                        fprintf(g_debug_log, "\n[READ - Dual Axis] Drive: %d", drive_id_);
                        
                        for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; ++i)
                        {
                            fprintf(g_debug_log, "\n  Axis[%zu] | Status: 0x%04X | Mode: %2d | Pos: %8d | Joint: %6.3f | Error: 0x%04X",
                                   i,
                                   input->axis[i].status_word,
                                   input->axis[i].mode_of_operation_display,
                                   input->axis[i].actual_position,
                                   joints[i]->joint_pos,
                                   input->axis[i].error_code);
                        }
                        fprintf(g_debug_log, "\n");
                        fflush(g_debug_log);
                    }
                }
            }
            else
            {
                SingleJointCyclicInput *input = (SingleJointCyclicInput *)driveInput;
                ar_client->readInputs(input);
                joints[0]->position_actual_value = input->actual_position;

                int32_t actual_pos = static_cast<int32_t>(input->actual_position);
                joints[0]->joint_pos = (actual_pos - joints[0]->home_encoder_offset) / joints[0]->pulse_per_revolution;

                if (ENABLE_PRINT)
                {
                    if(!g_debug_log) {
                        g_debug_log = fopen("/tmp/ar_drive_debug.log", "a");
                    }
                    
                    if(g_debug_log) {
                        fprintf(g_debug_log, "\n[READ - Single Axis] Drive: %d | Status: 0x%04X | Mode: %2d | Pos: %8d | Joint: %6.3f | Error: 0x%04X",
                               drive_id_, input->status_word, input->mode_of_operation_display, input->actual_position, joints[0]->joint_pos, input->error_code);
                        fprintf(g_debug_log, "\n");
                        fflush(g_debug_log);
                    }
                }
            }
        }

        else
        {
            DualJointProFileInput *input = (DualJointProFileInput *)driveInput;
            ar_client->readInputs(input);

            for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
            {
                joints[i]->position_actual_value = input->axis[i].actual_position;
                int32_t actual_pos = static_cast<int32_t>(input->axis[i].actual_position);
                joints[i]->joint_pos = (actual_pos - joints[i]->home_encoder_offset) / joints[i]->pulse_per_revolution;
            }

            static int print_counter = 0;
            if (++print_counter >= 250 && ENABLE_PRINT)
            {
                printf(COLOR_GREEN "\n[READ - PP Mode Dual Axis]");
                for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; ++i)
                {
                    uint16_t status = input->axis[i].status_word;
                    bool target_reached = (status & (1 << 10)) != 0; // Bit 10: Target reached
                    bool setpoint_ack = (status & (1 << 12)) != 0;   // Bit 12: Setpoint acknowledged

                    printf("\n  Axis[%zu] | Status: 0x%04X | Mode: %2d | Pos: %8d | Joint: %6.3f rad | Error: 0x%04X",
                           i,
                           status,
                           input->axis[i].mode_of_operation_display,
                           input->axis[i].actual_position,
                           joints[i]->joint_pos,
                           input->axis[i].error_code);

                    // Additional status information
                    printf(" | Target: %s | Setpoint: %s",
                           target_reached ? "YES" : "NO",
                           setpoint_ack ? "ACK" : "NO");
                }
                printf("\n" COLOR_RESET);
                fflush(stdout);
                print_counter = 0;            // TEST: Check if mode is correct

            }

            static int mode_check_counter = 0;
            if (++mode_check_counter >= 1000) // Check every 4 seconds
            {
                bool mode_correct = true;
                for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
                {
                    if (input->axis[i].mode_of_operation_display != 1)
                    {
                        mode_correct = false;
                        printf(COLOR_RED "\n[TEST ERROR] Axis[%zu] is in mode %d, expected mode 1 (PP)" COLOR_RESET,
                               i, input->axis[i].mode_of_operation_display);
                    }
                }
                if (mode_correct)
                {
                    printf(COLOR_GREEN "\n[TEST OK] All axes in Profile Position Mode (mode 1)" COLOR_RESET);
                }
                mode_check_counter = 0;
            }
        }
    }

    void ArDriveControl::motorOn()
    {
        if (!ar_client)
        {
            return;
        }
        else
        {
            if (is_dual_axis_)
            {
                ar_client->dualMotorOn((DualJointCyclicInput *)driveInput, (DualJointCyclicOutput *)driveOutput);
            }
            else
            {
                ar_client->singleMotorOn((SingleJointCyclicInput *)driveInput, (SingleJointCyclicOutput *)driveOutput);
            }
        }
    }

    void ArDriveControl::motorOff()
    {
        if (!ar_client)
        {
            return;
        }
        else
        {
            if (is_dual_axis_)
            {
                ar_client->dualMotorOff((DualJointCyclicInput *)driveInput, (DualJointCyclicOutput *)driveOutput);
            }
            else
            {
                ar_client->singleMotorOff((SingleJointCyclicInput *)driveInput, (SingleJointCyclicOutput *)driveOutput);
            }
        }
    }

    template void ArDriveControl::jointCmdToPulses<SingleJointCyclicOutput>(ArJointControl *, SingleJointCyclicOutput *);
    template void ArDriveControl::jointCmdToPulses<DualJointCyclicOutput>(ArJointControl *, DualJointCyclicOutput *);
    template void ArDriveControl::jointCmdToPulses<DualJointProFileOutput>(ArJointControl *, DualJointProFileOutput *);

} // namespace ar_control