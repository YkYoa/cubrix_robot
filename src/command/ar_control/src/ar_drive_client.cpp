#include <iostream>
#include <stdio.h>
#include <ar_drive_client.h>

namespace ar_control
{
  ArDriveClient::ArDriveClient(std::shared_ptr<master::EthercatManager> manager, int slaveId) : manager_(manager), slave_id_(slaveId)
  {
    // input_map_size = (int)(manager_.getInputBits(slave_id_) / 8);
    // output_map_size = (int)(manager_.getOutputBits(slave_id_) / 8);
    // driver_info_ = manager_.getDriverInfo(slave_id_);

    error_maps = nullptr;
  }

  ArDriveClient::~ArDriveClient()
  {
  }

  void ArDriveClient::readInputs(SingleJointCyclicInput *input)
  {
    uint8_t map[input_map_size] = {0};

    // input->statusword = *(uint16 *)(map + );
    // input->operation_mode = *(uint8 *)(map + );
    // input->position_actual_value = *(uint32 *)(map + );
    // input->velocity_actual_value = *(uint32 *)(map + );


  }

  void ArDriveClient::readOutputs(SingleJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};
    // output->error_code = *(uint16 *)(map + );
    // output->controlword = *(uint16 *)(map + );
    // output->operation_mode = *(uint8 *)(map + );
    // output->position_target_value = *(uint32 *)(map + );
    // output->velocity_target_value = *(uint32 *)(map + );
  }

  void ArDriveClient::writeOutputs(SingleJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};
    // *(uint16 *)(map + ) = output->controlword;
    // *(uint8 *)(map + ) = output->operation_mode;
    // *(uint32 *)(map + ) = output->position_target_value;
    // *(uint32 *)(map + ) = output->velocity_target_value;

    // for(int i = 0; i < output_map_size; i++)
    // {
    //   manager_.writeOutputs(slave_id_, map, output_map_size);
    // }
  }

}

