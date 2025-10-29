#include <ar_drive_control.h>

double LIMIT_SAFETY_OFFSET = 0.0001; ///< Safety offset for joint limits
bool ENABLE_PRINT = true;            ///< Flag to enable or disable drive logging

namespace ar_control
{
    ArDriveControl::ArDriveControl(DriveParameter &driveParm, bool uiState) : drive_parameter(driveParm), is_ui_(uiState)
    {
        drive_id_ = driveParm.drive_id;
        is_dual_axis_ = driveParm.is_dual_axis;
        driveInput = nullptr;
        driveOutput = nullptr;
    }

    void ArDriveControl::InitializeDriveClient(master::EthercatManager *manager, int slaveId)
    {
        printf(COLOR_BLUE "\n[Ar Drive Control] InitializeDriveClient called for drive %d, slaveId %d, manager %p" COLOR_RESET,
               drive_id_, slaveId, (void *)manager);

        drive_id_ = slaveId;
        ar_client.reset();
        if (manager)
        {
            printf(COLOR_GREEN "\n[Ar Drive Control] Creating ArDriveClient for drive %d" COLOR_RESET, drive_id_);
            ar_client = std::make_unique<ArDriveClient>(*manager, slaveId);
            printf(COLOR_GREEN "\n[Ar Drive Control] ArDriveClient created successfully for drive %d" COLOR_RESET, drive_id_);
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

        if (ar_client == nullptr)
        {
            joint->joint_pos_cmd = joint->joint_pos = joint->joint_vel = joint->joint_vel_cmd = joint->home_encoder_offset = 0;
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

        const uint16_t enable_sequence[] = {
            0x0006, // Switch On Disabled → Ready to Switch On
            0x0007, // Ready to Switch On → Switched On
            0x001F  // Switched On → Operation Enabled
        };

        if (drive_parameter.is_dual_axis == true)
        {
            DualJointCyclicInput *input = new DualJointCyclicInput();
            DualJointCyclicOutput *output = new DualJointCyclicOutput();
            driveInput = input;
            driveOutput = output;

            if (ar_client != nullptr)
            {
                ar_client->resetFaultDualJoint(input, output);
                memset(output, 0, sizeof(DualJointCyclicOutput));

                output->axis[0].control_word = 0x0006;
                output->axis[1].control_word = 0x0006;
                ar_client->writeOutputs(output);
                usleep(50000);
                
                ar_client->readInputs(input);
                printf("\n After control word = 6h:");
                for (size_t i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
                {
                    printf("\n error_code = %04x, status_word %04x, operation_mode = %2d", input->axis[i].error_code, input->axis[i].status_word, input->axis[i].mode_of_operation_display);
                }

                output->axis[0].control_word = 0x0007;
                output->axis[1].control_word = 0x0007;
                ar_client->writeOutputs(output);
                usleep(50000);
                
                ar_client->readInputs(input);
                printf("\n After control word = 7h:");
                for (size_t i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
                {
                    printf("\n error_code = %04x, status_word %04x, operation_mode = %2d", input->axis[i].error_code, input->axis[i].status_word, input->axis[i].mode_of_operation_display);
                }

                output->axis[0].control_word = 0x001f; 
                output->axis[1].control_word = 0x001f;
                ar_client->writeOutputs(output);
                usleep(50000);
                
                ar_client->readInputs(input);
                printf("\n After control word = fh (Enable Operation):");
                for (size_t i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
                {
                    printf("\n error_code = %04x, status_word %04x, operation_mode = %2d", input->axis[i].error_code, input->axis[i].status_word, input->axis[i].mode_of_operation_display);
                }
            }
        }
        else
        {
            SingleJointCyclicInput *input = new SingleJointCyclicInput();
            SingleJointCyclicOutput *output = new SingleJointCyclicOutput();
            driveInput = input;
            driveOutput = output;

            if (ar_client != nullptr)
            {
                ar_client->resetFaultSingleJoint(input, output);
                memset(output, 0, sizeof(SingleJointCyclicOutput));
                                
                for(uint16 cmd : enable_sequence) {
                    output->control_word = cmd;
                    ar_client->writeOutputs(output);
                    usleep(10000);
                }
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
            for(size_t i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++){
                printf("\n error_code = %04x, status_word %04x, operation_mode = %2d", input->axis[i].error_code
                                    , input->axis[i].status_word, input->axis[i].mode_of_operation_display);
            }

            ar_client->resetFaultDualJoint(input, output);
            ar_client->dualMotorOff(input, output);
        }
        else
        {
            SingleJointCyclicInput *input = (SingleJointCyclicInput *)driveInput;
            SingleJointCyclicOutput *output = (SingleJointCyclicOutput *)driveOutput;
            ar_client->resetFaultSingleJoint(input, output);
            // ar_client->singleMotorOff(input,output);
        }
        // ar_client->motorOff(input,output);
    }

    int ArDriveControl::getInputActualValueToStatus(tVectorS &jointNames, tVectorS &hardwareIds,
                                                    std::vector<uint32_t> &positionActualValues, std::vector<uint32_t> &velocityActualValues)
    {
        int nDof = 0;
        for (auto joint : joints)
        {
            joint->getInputActualValueToStatus(joint->joint_name, joint->hardware_id,
                                               joint->position_actual_value, joint->velocity_actual_value);
            jointNames.push_back(joint->joint_name);
            hardwareIds.push_back(joint->hardware_id);
            positionActualValues.push_back(joint->position_actual_value);
            velocityActualValues.push_back(joint->velocity_actual_value);
            nDof++;
        }
        return nDof;
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

                static int write_counter = 0;
                if (++write_counter >= 100 && ENABLE_PRINT)
                {
                    printf(COLOR_BLUE "\n  [WRITE - Dual Axis]");
                    for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; ++i)
                    {
                        printf("\n  Axis[%zu] | Control Word: 0x%04X | Target Pos: %d | Cmd Pos: %6.3f",
                               i,
                               output->axis[i].control_word,
                               output->axis[i].target_position,
                               joints[i]->joint_pos_cmd);
                    }
                    printf("\n" COLOR_RESET);
                    fflush(stdout);
                    write_counter = 0;
                }
            }
            else
            {
                SingleJointCyclicOutput *output = (SingleJointCyclicOutput *)driveOutput;
                jointCmdToPulses(joint, output);
                ar_client->writeOutputs(output);


                static int write_counter_single = 0;
                if (++write_counter_single >= 100 && ENABLE_PRINT)
                {
                    printf(COLOR_BLUE "\n  [WRITE - Single Axis] Control Word: 0x%04X | Target Pos: %d | Cmd Pos: %6.3f"
                           , output->control_word, output->target_position, joint->joint_pos_cmd);
                    printf("\n" COLOR_RESET);
                    fflush(stdout);
                    write_counter_single = 0;
                }
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

                static int print_counter = 0;
                if (++print_counter >= 100 && ENABLE_PRINT)
                {
                    printf(COLOR_GREEN "\n[READ - Dual Axis]");
                    for (size_t i = 0; i < joints.size() && i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; ++i)
                    {
                        printf("\n  Axis[%zu] | Status: 0x%04X | Mode: %2d | Pos: %8d | Joint: %6.3f",
                               i,
                               input->axis[i].status_word,
                               input->axis[i].mode_of_operation_display,
                               input->axis[i].actual_position,
                               joints[i]->joint_pos);
                    }
                    printf("\n" COLOR_RESET);
                    fflush(stdout);
                    print_counter = 0;
                }
            }
            else
            {
                SingleJointCyclicInput *input = (SingleJointCyclicInput *)driveInput;
                ar_client->readInputs(input);
                joints[0]->position_actual_value = input->actual_position;

                int32_t actual_pos = static_cast<int32_t>(input->actual_position);
                joints[0]->joint_pos = (actual_pos - joints[0]->home_encoder_offset) / joints[0]->pulse_per_revolution;

                static int print_counter_single = 0;
                if (++print_counter_single >= 100 && ENABLE_PRINT)
                {
                    printf(COLOR_GREEN "\n[READ - Single Axis] Status: 0x%04X | Mode: %2d | Pos: %8d | Joint: %6.3f"
                           , input->status_word, input->mode_of_operation_display, input->actual_position, joints[0]->joint_pos);
                    printf("\n" COLOR_RESET);
                    fflush(stdout);
                    print_counter_single = 0;
                }
            }
        }
    }

    template void ArDriveControl::jointCmdToPulses<SingleJointCyclicOutput>(ArJointControl *, SingleJointCyclicOutput *);
    template void ArDriveControl::jointCmdToPulses<DualJointCyclicOutput>(ArJointControl *, DualJointCyclicOutput *);

} // namespace ar_control