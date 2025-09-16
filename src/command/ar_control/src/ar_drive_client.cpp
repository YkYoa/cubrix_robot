#include <iostream>
#include <stdio.h>
#include <ar_drive_client.h>

namespace ar_control
{
  ArDriveClient::ArDriveClient(std::shared_ptr<master::EthercatManager> manager, int slaveId) : manager_(manager), slave_id_(slaveId)
  {
    

    error_maps = nullptr;
  }

  ArDriveClient::~ArDriveClient()
  {
  }

  void ArDriveClient::readInputs(SingleJointCyclicInput *input)
  {
    uint8_t map[input_map_size] = {0};


  }

  void ArDriveClient::readOutputs(SingleJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};


  }

  void ArDriveClient::writeOutputs(SingleJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};
    
  }

}

