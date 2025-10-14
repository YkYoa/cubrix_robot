#include <ar_drive_control.h>


double LIMIT_SAFETY_OFFSET = 0.0001; ///< Safety offset for joint limits

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

        if (drive_parameter.is_dual_axis == true)
        {
            DualJointCyclicInput *input = new DualJointCyclicInput();
            DualJointCyclicOutput *output = new DualJointCyclicOutput();
            driveInput = input;
            driveOutput = output;

            if (ar_client != nullptr)
            {
                ar_client->resetFaultDualJoint(input, output);
                                
                output->control_word_1 = 0x0006;
                usleep(10000);
                output->control_word_1 = 0x0007;
                usleep(10000);
                output->control_word_1 = 0x000f;
                usleep(10000);
                output->control_word_2 = 0x0006;
                usleep(10000);
                output->control_word_2 = 0x0007;
                usleep(10000);
                output->control_word_2 = 0x000f;
                usleep(10000);
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
                                
                output->control_word = 0x0006;
                usleep(10000);
                output->control_word = 0x0007;
                usleep(10000);
                output->control_word = 0x000f;
                usleep(10000);
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
            printf("error_code = %04x, status_word %04x, operation_mode = %2d", input->error_code_1, input->status_word_1);

            ar_client->dualMotorOff(input, output);
            ar_client->resetFaultDualJoint(input, output);
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
        output->target_position_1 = int32(joint->joint_pos_cmd * joint->pulse_per_revolution * 4);
        

        if (joints.size() > 1)
        {
            output->target_position_2 = int32(joints[1]->joint_pos_cmd * joints[1]->pulse_per_revolution * 4);
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
                
                // Read inputs to get status information
                DualJointCyclicInput *input = (DualJointCyclicInput *)driveInput;
                ar_client->readInputs(input);

                static int write_counter = 0;
                if (++write_counter >= 50) {
                    printf("\nWRITE - Drive %d: Joint1 Cmd: %5.3f, Target Pos: %d | Joint2 Cmd: %5.3f, Target Pos: %d",
                           drive_id_, joints[0]->joint_pos_cmd, output->target_position_1,
                           joints.size() > 1 ? joints[1]->joint_pos_cmd : 0.0,
                           joints.size() > 1 ? output->target_position_2 : 0);
                    printf("\n  Control Word 1: 0x%04X, Control Word 2: 0x%04X", output->control_word_1, output->control_word_2);
                    printf("\n  Status Word 1: 0x%04X, Status Word 2: 0x%04X", input->status_word_1, input->status_word_2);
                    printf("\n  Operation Mode 1: %d, Operation Mode 2: %d", input->mode_of_operation_display_1, input->mode_of_operation_display_2);
                    printf("\n  Error Code 1: 0x%04X, Error Code 2: 0x%04X", input->error_code_1, input->error_code_2);
                    printf("\n  Target Pos 1: %d, Target Pos 2: %d", output->target_position_1, output->target_position_2);
                    printf("\n  Actual Pos 1: %d, Actual Pos 2: %d", input->actual_position_1, input->actual_position_2);
                    
                    // Temporary test: Send larger commands to see movement
                    static int test_counter = 0;
                    if (++test_counter >= 100) { // Every 100 iterations
                        joints[0]->joint_pos_cmd = 1.0; // 1 radian (57 degrees)
                        if (joints.size() > 1) {
                            joints[1]->joint_pos_cmd = -0.5; // -0.5 radian (-28 degrees)
                        }
                        printf("\n  TEST: Sending larger commands - Joint1: 1.0 rad, Joint2: -0.5 rad");
                        test_counter = 0;
                    }
                    
                    fflush(stdout);
                    write_counter = 0;
                }
            }
            else
            {
                SingleJointCyclicOutput *output = (SingleJointCyclicOutput *)driveOutput;
                jointCmdToPulses(joint, output);
                ar_client->writeOutputs(output);
                
                // Read inputs to get status information
                SingleJointCyclicInput *input = (SingleJointCyclicInput *)driveInput;
                ar_client->readInputs(input);
                
                static int write_counter_single = 0;
                if (++write_counter_single >= 50) {
                    printf("\nWRITE - Drive %d: Joint Cmd: %5.3f, Target Pos: %d",
                           drive_id_, joints[0]->joint_pos_cmd, output->target_position);
                    printf("\n  Control Word: 0x%04X, Status Word: 0x%04X", output->control_word, input->status_word);
                    fflush(stdout);
                    write_counter_single = 0;
                }
            }
        }
    }

    void ArDriveControl::read()
    {
        ArJointControl *joint = joints[0];
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
                joints[0]->position_actual_value = input->actual_position_1;

                int32_t actual_pos_1 = static_cast<int32_t>(input->actual_position_1);
                joints[0]->joint_pos = (actual_pos_1 - joints[0]->home_encoder_offset) / joints[0]->pulse_per_revolution;

                if (joints.size() > 1)
                {
                    joints[1]->position_actual_value = input->actual_position_2;
                    int32_t actual_pos_2 = static_cast<int32_t>(input->actual_position_2);
                    joints[1]->joint_pos = (actual_pos_2 - joints[1]->home_encoder_offset) / joints[1]->pulse_per_revolution;
                }

                 static int print_counter = 0;
                 if (++print_counter >= 50)
                 {
                     printf("\nREAD - Drive %d: Joint1: %s | Actual Pos: %d | Joint Pos: %5.3f | "
                            "Joint2: %s | Actual Pos: %d | Joint Pos: %5.3f",
                            drive_id_, joints[0]->joint_name.c_str(),
                            joints[0]->position_actual_value, joints[0]->joint_pos,
                            joints.size() > 1 ? joints[1]->joint_name.c_str() : "N/A",
                            joints.size() > 1 ? joints[1]->position_actual_value : 0,
                            joints.size() > 1 ? joints[1]->joint_pos : 0.0);
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
                 if (++print_counter_single >= 50)
                 {
                     printf("\nREAD - Drive %d: Joint: %s | Actual Pos: %d | Joint Pos: %5.3f",
                            drive_id_, joints[0]->joint_name.c_str(), joints[0]->position_actual_value, joints[0]->joint_pos);
                     fflush(stdout);
                     print_counter_single = 0;
                 }
            }
        }
    }

    template void ArDriveControl::jointCmdToPulses<SingleJointCyclicOutput>(ArJointControl *, SingleJointCyclicOutput *);
    template void ArDriveControl::jointCmdToPulses<DualJointCyclicOutput>(ArJointControl *, DualJointCyclicOutput *);

} // namespace ar_control