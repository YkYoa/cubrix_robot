#include <iostream>
#include <stdio.h>
#include "ar_drive_client.h"

namespace ar_control
{
  ArDriveClient::ArDriveClient(master::EthercatManager &manager, int slaveId) : manager_(manager), slave_id_(slaveId)
  {
    input_map_size = (int)(manager_.getInputBits(slave_id_) / 8);
    output_map_size = (int)(manager_.getOutputBits(slave_id_) / 8);

    error_maps = nullptr;
  }

  ArDriveClient::~ArDriveClient()
  {
  }

  void ArDriveClient::readInputs(SingleJointCyclicInput *input)
  {
    uint8_t map[input_map_size] = {0};
    for (int i = 0; i < input_map_size; ++i)
    {
      map[i] = manager_.readInput(slave_id_, i);
    }

    input->error_code = *(uint16 *)(map + 0);
    input->status_word = *(uint16 *)(map + 2);
    input->mode_of_operation_display = *(uint8 *)(map + 4);
    input->actual_position = *(int32 *)(map + 5);
    input->touch_probe_status = *(uint16 *)(map + 9);
    input->touch_probe_1_positive_value = *(int32 *)(map + 11);
    input->digital_inputs = *(uint32 *)(map + 15);
  }

  void ArDriveClient::readOutputs(SingleJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};

    for (int i = 0; i < output_map_size; ++i)
    {
      map[i] = manager_.readOutput(slave_id_, i);
    }
    output->control_word = *(uint16 *)(map + 0);
    output->target_position = *(int32 *)(map + 2);
    output->touch_probe_function = *(uint16 *)(map + 6);
  }

  void ArDriveClient::writeOutputs(SingleJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};

    map[0] = (output->control_word) & 0x00ff;
    map[1] = (output->control_word >> 8) & 0x00ff;
    map[2] = (output->target_position) & 0x00ff;
    map[3] = (output->target_position >> 8) & 0x00ff;
    map[4] = (output->target_position >> 16) & 0x00ff;
    map[5] = (output->target_position >> 24) & 0x00ff;
    map[6] = (output->touch_probe_function) & 0x00ff;
    map[7] = (output->touch_probe_function >> 8) & 0x00ff;

    for (int i = 0; i < output_map_size; ++i)
    {
      manager_.write(slave_id_, i, map[i]);
    }
  }

  void ArDriveClient::readInputs(DualJointCyclicInput *input)
  {
    uint8_t map[input_map_size] = {0};
    for (int i = 0; i < input_map_size; ++i)
    {
      map[i] = manager_.readInput(slave_id_, i);
    }

    input->error_code_1 = *(uint16 *)(map + 0);
    input->status_word_1 = *(uint16 *)(map + 2);
    input->mode_of_operation_display_1 = *(uint8 *)(map + 4);
    input->actual_position_1 = *(int32 *)(map + 5);
    input->touch_probe_status_1 = *(uint16 *)(map + 9);
    input->touch_probe_1_positive_value_1 = *(int32 *)(map + 11);
    input->digital_inputs_1 = *(uint32 *)(map + 15);

    input->error_code_2 = *(uint16 *)(map + 19);
    input->status_word_2 = *(uint16 *)(map + 21);
    input->mode_of_operation_display_2 = *(uint8 *)(map + 23);
    input->actual_position_2 = *(int32 *)(map + 24);
    input->touch_probe_status_2 = *(uint16 *)(map + 28);
    input->touch_probe_1_positive_value_2 = *(int32 *)(map + 30);
    input->digital_inputs_2 = *(uint32 *)(map + 34);

  }

  void ArDriveClient::readOutputs(DualJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};

    for (int i = 0; i < output_map_size; ++i)
    {
      map[i] = manager_.readOutput(slave_id_, i);
    }
    output->control_word_1 = *(uint16 *)(map + 0);
    output->target_position_1 = *(int32 *)(map + 2);
    output->touch_probe_function_1 = *(uint16 *)(map + 6);

    output->control_word_2 = *(uint16 *)(map + 8);
    output->target_position_2 = *(int32 *)(map + 10);
    output->touch_probe_function_2 = *(uint16 *)(map + 14);
  }

  void ArDriveClient::writeOutputs(DualJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};

    map[0] = (output->control_word_1) & 0x00ff;
    map[1] = (output->control_word_1 >> 8) & 0x00ff;
    map[2] = (output->target_position_1) & 0x00ff;
    map[3] = (output->target_position_1 >> 8) & 0x00ff;
    map[4] = (output->target_position_1 >> 16) & 0x00ff;
    map[5] = (output->target_position_1 >> 24) & 0x00ff;
    map[6] = (output->touch_probe_function_1) & 0x00ff;
    map[7] = (output->touch_probe_function_1 >> 8) & 0x00ff;

    map[8] = (output->control_word_2) & 0x00ff;
    map[9] = (output->control_word_2 >> 8) & 0x00ff;
    map[10] = (output->target_position_2) & 0x00ff;
    map[11] = (output->target_position_2 >> 8) & 0x00ff;
    map[12] = (output->target_position_2 >> 16) & 0x00ff;
    map[13] = (output->target_position_2 >> 24) & 0x00ff;
    map[14] = (output->touch_probe_function_2) & 0x00ff;
    map[15] = (output->touch_probe_function_2 >> 8) & 0x00ff;

    for (int i = 0; i < output_map_size; ++i)
    {
      manager_.write(slave_id_, i, map[i]);
    }
  }

  template <typename T, typename U>
  void ArDriveClient::resetFaultSingleJoint(T *input, U *output)
  {
    readInputs(input);
    fflush(stdout);
    if (input->error_code == 0)
      return;

    output->control_word = 0x80;
    writeOutputs(output);
  }

  template <typename T, typename U>
  void ArDriveClient::resetFaultDualJoint(T *input, U *output)
  {
    readInputs(input);
    fflush(stdout);
    if (input->error_code_1 == 0 && input->error_code_2 == 0)
      return;

    output->control_word_1 = 0x80;
    output->control_word_2 = 0x80;
    writeOutputs(output);
  }

  template <typename T, typename U>
  void motorOff(T *input, U *output)
  {
  }



	//------------------------------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------------------------------

  template void ArDriveClient::resetFaultSingleJoint<SingleJointCyclicInput, SingleJointCyclicOutput>(SingleJointCyclicInput* input,
																					 SingleJointCyclicOutput* output);
  
  template void ArDriveClient::resetFaultDualJoint<DualJointCyclicInput, DualJointCyclicOutput>(DualJointCyclicInput* input,
																					 DualJointCyclicOutput* output);


}
