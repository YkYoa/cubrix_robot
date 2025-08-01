#include <ar_drive_control.h>


double LIMIT_SAFETY_OFFSET = 0.0001; ///< Safety offset for joint limits

namespace ar_control
{
    ArDriveControl::ArDriveControl(DriveParameter& driveParm,bool uiState) : drive_parameter(driveParm), is_ui_(uiState)
    {
        drive_id_ = driveParm.drive_id;
    }

    void ArDriveControl::InitializeDriveClient(int slaveId)
    {
        // This function will be implemented later
        // Initialize the drive client with the given slave ID
        // protocol_manager_->initializeClient(slaveId);

        return;
    }

    void ArDriveControl::AddJoint(JointParameter& jointParam)
    {
        ArJointControl* joint = new ArJointControl(jointParam.joint_name);
		printf(COLOR_BLUE "\n[Tomo Drive Control] Drive %d add joint: %s" COLOR_RESET, drive_id_, jointParam.joint_name.c_str());
        joint->rev_angle_convert_mode = jointParam.rev_angle_convert_mode;

        if(jointParam.joint_limits != NULL){
            joint->upper_limit = jointParam.joint_limits->upper - LIMIT_SAFETY_OFFSET;
            joint->lower_limit = jointParam.joint_limits->lower + LIMIT_SAFETY_OFFSET;
        }else{
            joint->upper_limit = 0.0;
            joint->lower_limit = 0.0;
        }

        if(jointCount() > 1){
            joint->client_id = jointParam.client_id;
        }

        joints.push_back(joint);

        if((jointCount() > 1) && (joint->client_id = -1)){
            joint->joint_pos_cmd = joint->joint_pos = joint->joint_vel = joint->joint_vel_cmd = joint->home_encoder_offset = 0;
        }

        if(abs(jointParam.encoder_offset) > 4000){
            printf(COLOR_RED "\n [Ar Drive Control] Joint '%s' has a large encoder offset: %d. Please check Joint Parameters." COLOR_RESET, 
                        jointParam.joint_name.c_str(), jointParam.encoder_offset);
            
            joint->home_encoder_offset = 0;
        }else{
            joint->home_encoder_offset = jointParam.encoder_offset;
        }

        if(jointParam.gear_ratio && jointParam.encoder_res){
            joint->pulse_per_revolution = jointParam.gear_ratio * jointParam.encoder_res;
            if(!joint->rev_angle_convert_mode){
                joint->pulse_per_revolution /= (2 * M_PI);
            }
            printf("\n [Ar Drive Control ]Joint '%s' gear ratio: %d, encoder resolution: %d, pulse per rev: %5.1f", 
                jointParam.joint_name.c_str(), jointParam.gear_ratio, jointParam.encoder_res, joint->pulse_per_revolution);
        }

        printf( "Joint '%s' added on drive %d with client Id: %d at encodere offset: %d", 
                    jointParam.joint_name.c_str(), drive_id_, joint->client_id, joint->home_encoder_offset);
        fflush(stdout);
    }

    void ArDriveControl::InitializeDrive()
    {
		printf(COLOR_DARKYELLOW "\n[Ar Drive Control] ---------------------- Initializing drive %d----------------------" COLOR_RESET,
			   drive_id_);
        // Now no drive client yet so return
        return;
    }

    ArDriveControl::~ArDriveControl()
    {
        printf("\n[Ar Drive Control] [Drive %d] destruction", drive_id_);
    }

    int ArDriveControl::getInputActualValueToStatus(tVectorS& jointNames, tVectorS& hardwareIds, 
                                std::vector<uint32_t>& positionActualValues, std::vector<uint32_t>& velocityActualValues)
    {
        int nDof = 0;
        for(auto joint : joints) {
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

    void ArDriveControl::write()
    {
        // No drive client yet so return
        return;
    }

    void ArDriveControl::read()
    {
        // In simulation mode, update joint states from commands
        for(auto joint : joints) {
            // Copy command position to actual position for simulation
            joint->joint_pos = joint->joint_pos_cmd;
            // Set velocity to 0 for simulation (or calculate from position difference)
            joint->joint_vel = 0.0;
        }
    }

} // namespace ar_control