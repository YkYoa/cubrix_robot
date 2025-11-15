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

    for (int i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
    {
      input->axis[i].error_code = *(uint16 *)(map + i * 19);
      input->axis[i].status_word = *(uint16 *)(map + i * 19 + 2);
      input->axis[i].mode_of_operation_display = *(uint8 *)(map + i * 19 + 4);
      input->axis[i].actual_position = *(int32 *)(map + i * 19 + 5);
      input->axis[i].touch_probe_status = *(uint16 *)(map + i * 19 + 9);
      input->axis[i].touch_probe_positive_value = *(int32 *)(map + i * 19 + 11);
      input->axis[i].digital_inputs = *(uint32 *)(map + i * 19 + 15);
    }
  }

  void ArDriveClient::readOutputs(DualJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};

    for (int i = 0; i < output_map_size; ++i)
    {
      map[i] = manager_.readOutput(slave_id_, i);
    }

    for (int i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
    {
      output->axis[i].control_word = *(uint16 *)(map + i * 10);
      output->axis[i].target_position = *(int32 *)(map + i * 10 + 2);
      output->axis[i].touch_probe_function = *(uint16 *)(map + i * 10 + 6);
    }
  }

  void ArDriveClient::writeOutputs(DualJointCyclicOutput *output)
  {
    uint8_t map[output_map_size] = {0};

    for (int i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
    {
      map[i * 8] = (output->axis[i].control_word) & 0x00ff;
      map[i * 8 + 1] = (output->axis[i].control_word >> 8) & 0x00ff;

      map[i * 8 + 2] = (output->axis[i].target_position) & 0x00ff;
      map[i * 8 + 3] = (output->axis[i].target_position >> 8) & 0x00ff;
      map[i * 8 + 4] = (output->axis[i].target_position >> 16) & 0x00ff;
      map[i * 8 + 5] = (output->axis[i].target_position >> 24) & 0x00ff;
      
      map[i * 8 + 6] = (output->axis[i].touch_probe_function) & 0x00ff;
      map[i * 8 + 7] = (output->axis[i].touch_probe_function >> 8) & 0x00ff;
    }

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

    bool axis0_fault = (input->axis[0].error_code == 0);
    bool axis1_fault = (input->axis[0].error_code == 0);
    if (axis0_fault && axis1_fault)
      return;

    for (int i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
    {
      output->axis[i].control_word = 0x80; // fault reset
    }
    writeOutputs(output);
    usleep(10000);

    int loop = 0;
    while (loop++ < 100)
    {
      readInputs(input);
      axis0_fault = (input->axis[0].error_code == 0);
      axis1_fault = (input->axis[1].error_code == 0);
      if (axis0_fault && axis1_fault)
        break;
      if (loop % 10 == 0)
      {
        printf("[resetFaultDualJoint] loop %d: axis0 err=0x%04x stat=0x%04x mode=%d pos=%08x | axis1 err=0x%04x stat=0x%04x mode=%d pos=%08x\n",
               loop,
               input->axis[0].error_code, input->axis[0].status_word, input->axis[0].mode_of_operation_display, input->axis[0].actual_position,
               input->axis[1].error_code, input->axis[1].status_word, input->axis[1].mode_of_operation_display, input->axis[1].actual_position);
      }

      if (!axis0_fault)
      {
        output->axis[0].control_word = 0x80; // fault reset
        writeOutputs(output);
        usleep(10000);
      }
      if (!axis1_fault)
      {
        output->axis[1].control_word = 0x80; // fault reset
        writeOutputs(output);
        usleep(10000);
      }
    }
  }

  template <typename T, typename U>
  void ArDriveClient::dualMotorOff(T *input, U *output)
  {
    const uint16 sequence[] = {0x00, 0x80, 0x00};

    for (uint16 cmd : sequence)
    {
      for (int i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++)
      {
        output->axis[i].control_word = cmd;
      }
      writeOutputs(output);
      usleep(50000);
    }

    printf("\n[dualMotorOff] Motors for all axes are now OFF (complete).\n");
  }

  template <typename T, typename U>
  void ArDriveClient::singleMotorOff(T *input, U *output)
  {
    const uint16 sequence[] = {0x00, 0x80, 0x00};

    for (uint16 cmd : sequence)
    {
      output->control_word = cmd;
      writeOutputs(output);
      usleep(50000);
    }
  }

  template <typename T, typename U>
  void ArDriveClient::dualMotorOn(T *input, U *output)
  {
    for(size_t i = 0; i < LEADSHINE_DRIVER_MAX_JOINT_COUNT; i++){
      output->axis[i].control_word |= 0x04;
    }
    writeOutputs(output);
  }

  template <typename T, typename U>
  void ArDriveClient::singleMotorOn(T *input, U *output)
  {
    output->control_word |= 0x04;
    writeOutputs(output);
  }

  //------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------

  template void ArDriveClient::resetFaultSingleJoint<SingleJointCyclicInput, SingleJointCyclicOutput>(SingleJointCyclicInput *input,
                                                                                                      SingleJointCyclicOutput *output);

  template void ArDriveClient::resetFaultDualJoint<DualJointCyclicInput, DualJointCyclicOutput>(DualJointCyclicInput *input,
                                                                                                DualJointCyclicOutput *output);

  template void ArDriveClient::dualMotorOn<DualJointCyclicInput, DualJointCyclicOutput>(DualJointCyclicInput *input,
                                                                                        DualJointCyclicOutput *output);

  template void ArDriveClient::singleMotorOn<SingleJointCyclicInput, SingleJointCyclicOutput>(SingleJointCyclicInput *input,
                                                                                              SingleJointCyclicOutput *output);

  template void ArDriveClient::dualMotorOff<DualJointCyclicInput, DualJointCyclicOutput>(DualJointCyclicInput *input,
                                                                                         DualJointCyclicOutput *output);

  template void ArDriveClient::singleMotorOff<SingleJointCyclicInput, SingleJointCyclicOutput>(SingleJointCyclicInput *input,
                                                                                               SingleJointCyclicOutput *output);
}
