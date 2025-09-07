#include <iostream>
#include <stdio.h>
#include <ar_drive_client.h>

namespace ar_control
{
    ArDriveClient::ArDriveClient(master::EthercatManager& manager, int slaveId) : manager_(manager), slave_id(slaveId)
    {
		input_map_size	= (int) (manager_.getInputBits(slave_id_) / 8);
		output_map_size = (int) (manager_.getOutputBits(slave_id_) / 8);
		driver_info_	= manager_.getDriverInfo(slave_id_);
    }

    ArDriveClient::~ArDriveClient()
    {
    }

    void ArDriveClient::readInputs(SingleJointCyclicInput* input)
    {
        uint8_t map[input_map_size] = {0};

		input->error_code			  = *(uint16*) (map + INPUT_MAP_ERROR_CODE_OFFSET);
		input->statusword			  = *(uint16*) (map + INPUT_MAP_STATUS_WORD_OFFSET);
		input->operation_mode		  = *(uint8*) (map + INPUT_MAP_OPERATION_MODE_OFFSET);
		input->position_actual_value  = *(uint32*) (map + INPUT_MAP_ACTUAL_POSITION_OFFSET);
		input->velocity_actual_value  = *(uint32*) (map + INPUT_MAP_ACTUAL_VELOCITY_OFFSET);
    }

    void ArDriveClient::readOutputs(SingleJointCyclicOutput* output)
    {
        uint8_t map[output_map_size] = {0};
        output->error_code			 = *(uint16*) (map + OUTPUT_MAP_ERROR_CODE_OFFSET);
        output->controlword			 = *(uint16*) (map + OUTPUT_MAP_CONTROL_WORD_OFFSET);
        output->operation_mode		 = *(uint8*) (map + OUTPUT_MAP_OPERATION_MODE_OFFSET);
        output->position_target_value = *(uint32*) (map + OUTPUT_MAP_TARGET_POSITION_OFFSET);
        output->velocity_target_value = *(uint32*) (map + OUTPUT_MAP_TARGET_VELOCITY_OFFSET);
    }

}


