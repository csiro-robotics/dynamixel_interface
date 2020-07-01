/* CSIRO Open Source Software License Agreement (variation of the BSD / MIT License)
 * Copyright (c) 2017, Commonwealth Scientific and Industrial Research Organisation (CSIRO) ABN 41 687 119 230.
 * All rights reserved. CSIRO is willing to grant you a license to the dynamixel_actuator ROS packages on the following
 * terms, except where otherwise indicated for third party material. Redistribution and use of this software in source
 * and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *   disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of CSIRO nor the names of its contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission of CSIRO.
 *
 * EXCEPT AS EXPRESSLY STATED IN THIS AGREEMENT AND TO THE FULL EXTENT PERMITTED BY APPLICABLE LAW, THE SOFTWARE IS
 * PROVIDED "AS-IS". CSIRO MAKES NO REPRESENTATIONS, WARRANTIES OR CONDITIONS OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO ANY REPRESENTATIONS, WARRANTIES OR CONDITIONS REGARDING THE CONTENTS OR ACCURACY OF THE SOFTWARE,
 * OR OF TITLE, MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, THE ABSENCE OF LATENT OR OTHER
 * DEFECTS, OR THE PRESENCE OR ABSENCE OF ERRORS, WHETHER OR NOT DISCOVERABLE.
 * TO THE FULL EXTENT PERMITTED BY APPLICABLE LAW, IN NO EVENT SHALL CSIRO BE LIABLE ON ANY LEGAL THEORY (INCLUDING,
 * WITHOUT LIMITATION, IN AN ACTION FOR BREACH OF CONTRACT, NEGLIGENCE OR OTHERWISE) FOR ANY CLAIM, LOSS, DAMAGES OR
 * OTHER LIABILITY HOWSOEVER INCURRED.  WITHOUT LIMITING THE SCOPE OF THE PREVIOUS SENTENCE THE EXCLUSION OF LIABILITY
 * SHALL INCLUDE: LOSS OF PRODUCTION OR OPERATION TIME, LOSS, DAMAGE OR CORRUPTION OF DATA OR RECORDS; OR LOSS OF
 * ANTICIPATED SAVINGS, OPPORTUNITY, REVENUE, PROFIT OR GOODWILL, OR OTHER ECONOMIC LOSS; OR ANY SPECIAL, INCIDENTAL,
 * INDIRECT, CONSEQUENTIAL, PUNITIVE OR EXEMPLARY DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT, ACCESS
 * OF THE SOFTWARE OR ANY OTHER DEALINGS WITH THE SOFTWARE, EVEN IF CSIRO HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * CLAIM, LOSS, DAMAGES OR OTHER LIABILITY. APPLICABLE LEGISLATION SUCH AS THE AUSTRALIAN CONSUMER LAW MAY APPLY
 * REPRESENTATIONS, WARRANTIES, OR CONDITIONS, OR IMPOSES OBLIGATIONS OR LIABILITY ON CSIRO THAT CANNOT BE EXCLUDED,
 * RESTRICTED OR MODIFIED TO THE FULL EXTENT SET OUT IN THE EXPRESS TERMS OF THIS CLAUSE ABOVE "CONSUMER GUARANTEES".
 * TO THE EXTENT THAT SUCH CONSUMER GUARANTEES CONTINUE TO APPLY, THEN TO THE FULL EXTENT PERMITTED BY THE APPLICABLE
 * LEGISLATION, THE LIABILITY OF CSIRO UNDER THE RELEVANT CONSUMER GUARANTEE IS LIMITED (WHERE PERMITTED AT CSIRO'S
 * OPTION) TO ONE OF FOLLOWING REMEDIES OR SUBSTANTIALLY EQUIVALENT REMEDIES:
 * (a)  THE REPLACEMENT OF THE SOFTWARE, THE SUPPLY OF EQUIVALENT SOFTWARE, OR SUPPLYING RELEVANT SERVICES AGAIN;
 * (b)  THE REPAIR OF THE SOFTWARE;
 * (c)  THE PAYMENT OF THE COST OF REPLACING THE SOFTWARE, OF ACQUIRING EQUIVALENT SOFTWARE, HAVING THE RELEVANT
 *      SERVICES SUPPLIED AGAIN, OR HAVING THE SOFTWARE REPAIRED.
 * IN THIS CLAUSE, CSIRO INCLUDES ANY THIRD PARTY AUTHOR OR OWNER OF ANY PART OF THE SOFTWARE OR MATERIAL DISTRIBUTED
 * WITH IT.  CSIRO MAY ENFORCE ANY RIGHTS ON BEHALF OF THE RELEVANT THIRD PARTY.
 *
 * Third Party Components:
 *
 * The following third party components are distributed with the Software.  You agree to comply with the license terms
 * for these components as part of accessing the Software.  Other third party software may also be identified in
 * separate files distributed with the Software.
 * ___________________________________________________________________
 *
 * dynamixel_interface_driver and dynamixel_interface_controller packages are forked from projects authored by Brian
 * Axelrod (on behalf of Willow Garage):
 *
 * https://github.com/baxelrod/dynamixel_pro_controller
 * https://github.com/baxelrod/dynamixel_pro_driver
 *
 * Thus they retain the following notice:
 *
 * Software License Agreement (BSD License)
 * Copyright (c) 2013, Willow Garage
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *  - Neither the name of Willow Garage nor the names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ___________________________________________________________________
 */

 /**
 * @file   dynamixel_interface_driver.cpp
 * @author Tom Molnar (Tom.Molnar@data61.csiro.au), Brian Axelrod
 * @date   January, 2017
 * @brief  Implements the hardware abstraction for communicating with dynamixels
 */

#include "yaml-cpp/yaml.h"
#include <ros/package.h>
#include <dynamixel_interface_driver/dynamixel_interface_driver.h>


namespace dynamixel_interface_driver
{

/// Constructor. Initialises port and packet handling objects and sets the
/// baud rate.
/// @param[in] device The serial port to connect to
/// @param[in] baud The baud rate to use
/// @param[optional] use_legacy_protocol Whether to use legacy 1.0 protocol (default false)
/// @param[optional] use_group_read Whether to use bulk protocol methods with bulk getters (default true)
/// @param[optional] use_group_read Whether to use bulk protocol methods with bulk setters (default true)
DynamixelInterfaceDriver::DynamixelInterfaceDriver(std::string device="/dev/ttyUSB0", int baud=1000000,
    bool use_legacy_protocol = false, bool use_group_read=true, bool use_group_write=true)
{

  ROS_INFO("Device is '%s' with baud rate '%d', using protocol %d", device.c_str(), baud, static_cast<uint8_t>(use_legacy_protocol)+1);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device.c_str());

  // set indicator for using group comms
  use_legacy_protocol_ = use_legacy_protocol;
  use_group_read_ = use_group_read;
  use_group_write_ = use_group_write;

  // intialise failsafe fallback counter
  single_read_fallback_counter_ = 0;

  // set packet handler for requested protocol version
  if (use_legacy_protocol_)
  {
    // Initialize PacketHandler instances for each protocol version
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
  }
  else
  {
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
  }

  // read in motor data yaml file
  // load the file containing model info, we're not using the param server here
  std::string path = ros::package::getPath("dynamixel_interface_controller");
  path += "/config/motor_data.yaml";
  YAML::Node doc = YAML::LoadFile(path);

  //read in the dynamixel model information from the motor data file
  for (int i = 0; i < doc.size(); i++)
  {
    DynamixelSpec spec;

    // Load the basic specs of this motor type
    spec.name = doc[i]["name"].as<std::string>();
    spec.model_number = doc[i]["model_number"].as<uint>();

    std::string type = doc[i]["type"].as<std::string>();
    if (type == "M")
    {
      spec.type = DXL_MX;
    }
    else if (type == "LM")
    {
      spec.type = DXL_LEGACY_MX;
    }
    else if (type ==  "X")
    {
      spec.type = DXL_X;
    }
    else if (type == "A")
    {
      spec.type = DXL_AX;
    }
    else if (type == "R")
    {
      spec.type = DXL_RX;
    }
    else if (type == "P")
    {
      spec.type = DXL_P;
    }
    else if (type == "LP")
    {
      spec.type = DXL_LEGACY_PRO;
    }
    else
    {
      spec.type = DXL_UNKNOWN;
    }

    spec.cpr = doc[i]["cpr"].as<uint>();
    spec.gear_conversion = doc[i]["gear_conversion"].as<double>();
    spec.effort_ratio = doc[i]["effort_ratio"].as<double>();
    spec.current_ratio = doc[i]["current_ratio"].as<double>();

    model_specs_[spec.model_number] = spec;
  }

  // Open port
  if (portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the port(%s)!", device.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the port!");
    ros::shutdown();
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud))
  {
    ROS_INFO("Succeeded to change the baudrate(%d)\n!", portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
    ros::shutdown();
  }
}

/// Destructor. Closes and releases serial port.
DynamixelInterfaceDriver::~DynamixelInterfaceDriver()
{
  portHandler_->closePort();
  delete portHandler_;
  delete packetHandler_;
}

/// Ping the specified id, used to check if a dynamixel with that ID is on the bus
/// @param[in] servo_id The ID to ping on the bus.
/// @return True if a dynamixel responds, false otherwise.
bool DynamixelInterfaceDriver::ping(int servo_id)
{
  uint8_t error;
  int dxl_comm_result;

  //ping dynamixel on bus
  dxl_comm_result = packetHandler_->ping(portHandler_, servo_id, &error);

  //CHECK IF PING SUCCEEDED
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/// Retrieves the model number from the dynamixel's eeprom
/// @param[in] servo_id The ID of the servo to retrieve from
/// @param[out] model_number Stores the model_number returned
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::getModelNumber(int servo_id, uint16_t* model_number)
{
  uint8_t error;
  int dxl_comm_result;

  //read in first 2 bytes of eeprom (same for all series)
  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, servo_id, 0, model_number, &error);

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/// Retrieves the maximum torque limit from the dynamixel's eeprom.
/// @param[in] servo_id The ID of the servo to retrieve from
/// @param[in] type the type of the servo to read from
/// @param[out] max_torque Stores the value returned
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::getMaxTorque(int servo_id, DynamixelSeriesType type, uint16_t* max_torque)
{
  uint8_t error;
  int dxl_comm_result = -1;

  //Read address and size always depends on servo series
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_MAX_TORQUE,
        (uint8_t*) max_torque, &error);
      break;

    case DXL_X:
    case DXL_MX:
      dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, servo_id, DXL_STANDARD_CURRENT_LIMIT,
        max_torque, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, servo_id, DXL_P_CURRENT_LIMIT,
        max_torque, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_MAX_TORQUE,
        max_torque, &error);
      break;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/// Retrieves the torque enabled value from the dynamixel's ram.
/// @param[in] servo_id The ID of the servo to retrieve from
/// @param[out] torque_enabled Stores the status of torque enable
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::getTorqueEnabled(int servo_id, DynamixelSeriesType type, bool *torque_enabled)
{

  uint8_t error;
  int dxl_comm_result;
  uint8_t data = 0;

  //Read address and size always depends on servo series
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_TORQUE_ENABLE,
        &data, &error);
      break;

    case DXL_X:
    case DXL_MX:
      dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, servo_id, DXL_STANDARD_TORQUE_ENABLE,
        &data, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, servo_id, DXL_P_TORQUE_ENABLE,
        &data, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_TORQUE_ENABLE,
        &data, &error);
      break;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    *torque_enabled = (data > 0);
    return true;
  }
  else
  {
    return false;
  }

}

/// Retrieves the current target_velocity from the dynamixel's ram.
/// @param servo_id The ID of the servo to retrieve from
/// @param target_velocity Stores the value returned
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::getTargetTorque(int servo_id, DynamixelSeriesType type, int16_t* target_torque)
{

  uint8_t error;
  int dxl_comm_result;

  //Read address and size always depends on servo series
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_GOAL_TORQUE,
        (uint16_t*) &target_torque, &error);
      break;

    case DXL_X:
    case DXL_MX:
      dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, servo_id, DXL_STANDARD_GOAL_CURRENT,
        (uint16_t*) &target_torque, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, servo_id, DXL_P_GOAL_CURRENT,
        (uint16_t*) &target_torque, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_GOAL_TORQUE,
        (uint16_t*) &target_torque, &error);
      break;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}

/// Retrieves arbitrary register readings from the Dynamixel, can be used in cases where provided getters are
/// insufficient or to read multiple contiguous values at once.
/// @param servo_id The ID of the servo to retrieve from
/// @param address The address value in the control table the dynamixel will start reading from
/// @param length The number of bytes to read consecutively from the control table
/// @param response Array to store the raw dynamixel response.
bool DynamixelInterfaceDriver::readRegisters(int servo_id, uint16_t address, uint16_t length,
    std::vector<uint8_t> *response)
{

  uint8_t error;
  int dxl_comm_result;

  // resize output vector for data
  response->resize(length);

  dxl_comm_result = packetHandler_->readTxRx(portHandler_, servo_id, address, length, response->data(), &error);

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}

// *********************** BULK_READ METHODS *************************** //

/// Bulk Reads the Present Position, Present Velocity and Present Current in one instruction. If the group read fails
/// the function will fall back on reading each motor individually. Optionally, the group comms can be disabled on
/// initialisation of the driver (by setting use_group_read to false) in which case the function will always read
/// from each dynamixel individually.
/// @param state_map map of servo ids to state data to read into
/// @return True on comm success, false otherwise
bool DynamixelInterfaceDriver::getBulkState(std::unordered_map<int, DynamixelState> &state_map)
{
  DynamixelSeriesType type;
  bool success = false;
  std::unordered_map<int, SyncData*> read_map;
  uint i = 0;

  // base case, nothing to read
  if (state_map.size() == 0)
  {
    return false;
  }


  // check types match
  for (auto& it : state_map)
  {
    if (i == 0)
    {
      type = it.second.type;
    }
    else if ((i > 0) && (type != it.second.type))
    {
      return false;
    }
    read_map[it.first] = static_cast<SyncData*>(&it.second);
    i++;
  }

  // perform bulk read depending on type
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      // read data
      if(use_group_read_)
      {
        bulkRead(read_map, DXL_LEGACY_PRESENT_POSITION, 6); // bulk method
      }
      else
      {
        for (auto& it : state_map)
        {
          if(readRegisters(it.first, DXL_LEGACY_PRESENT_POSITION, 6, &it.second.data)) // individual reads
          {
            it.second.success = true;
          }
          else
          {
            it.second.success = false;
          }

        }
      }
      for (auto& it : state_map) // decode responses
      {
        if (it.second.success)
        {
          it.second.position = *((int16_t*) &it.second.data[0]);
          it.second.velocity = *((int16_t*) &it.second.data[2]);
          it.second.effort = *((int16_t*) &it.second.data[4]);
          success = true;
        }
      }
      return success;

    case DXL_MX:
    case DXL_X:
      // read data
      if(use_group_read_)
      {
        bulkRead(read_map, DXL_STANDARD_PRESENT_CURRENT, 10); // bulk method
      }
      else
      {
        for (auto& it : state_map)
        {
          if(readRegisters(it.first, DXL_STANDARD_PRESENT_CURRENT, 10, &it.second.data)) // individual reads
          {
            it.second.success = true;
          }
          else
          {
            it.second.success = false;
          }

        }
      }
      for (auto& it : state_map) // decode responses
      {
        if (it.second.success)
        {
          it.second.effort = *((int16_t*) &it.second.data[0]);
          it.second.velocity = *((int32_t*) &it.second.data[2]);
          it.second.position = *((int32_t*) &it.second.data[6]);
          success = true;
        }
      }
      return success;

    case DXL_P:
      // read data
      if(use_group_read_)
      {
        bulkRead(read_map, DXL_P_PRESENT_CURRENT, 10); // bulk method
      }
      else
      {
        for (auto& it : state_map)
        {
          if(readRegisters(it.first, DXL_P_PRESENT_CURRENT, 10, &it.second.data)) // individual reads
          {
            it.second.success = true;
          }
          else
          {
            it.second.success = false;
          }

        }
      }
      for (auto& it : state_map) // decode responses
      {
        if (it.second.success)
        {
          it.second.effort = *((int16_t*) &it.second.data[0]);
          it.second.velocity = *((int32_t*) &it.second.data[2]);
          it.second.position = *((int32_t*) &it.second.data[6]);
          success = true;
        }
      }
      return success;

    case DXL_LEGACY_PRO:
      // read data
      if(use_group_read_)
      {
        bulkRead(read_map, DXL_LEGACY_PRO_PRESENT_POSITION, 10); // bulk method
      }
      else
      {
        for (auto& it : state_map)
        {
          if(readRegisters(it.first, DXL_LEGACY_PRO_PRESENT_POSITION, 10, &it.second.data)) // individual reads
          {
            it.second.success = true;
          }
          else
          {
            it.second.success = false;
          }
        }
      }
      for (auto& it : state_map) // decode responses
      {
        if (it.second.success)
        {
          it.second.position = *((int32_t*) &it.second.data[0]);
          it.second.velocity = *((int32_t*) &it.second.data[4]);
          it.second.effort = *((int16_t*) &it.second.data[8]);
          success = true;
        }
      }
      return success;

    default:
      return false;
  }

  return false;
}


/// Bulk Reads the voltage and temperature in one instruction, behaves the same as getBulkState()
/// @param diag_map map of servo ids to diag_data to read into
/// @return True on comm success, false otherwise
bool DynamixelInterfaceDriver::getBulkDiagnosticInfo(std::unordered_map<int, DynamixelDiagnostic> &diag_map)
{
  DynamixelSeriesType type;
  bool success = false;
  std::unordered_map<int, SyncData*> read_map;
  uint i = 0;

  // base case, nothing to read
  if (diag_map.size() == 0)
  {
    return false;
  }

  // check types match
  for (auto& it : diag_map)
  {
    if (i == 0)
    {
      type = it.second.type;
    }
    else if ((i > 0) && (type != it.second.type))
    {
      return false;
    }
    read_map[it.first] = static_cast<SyncData*>(&it.second);
    i++;
  }

  // perform bulk read depending on type
  switch(type)
  {
    // OLDER, DISCONTINUED SERIES
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      // read data
      if(use_group_read_)
      {
        bulkRead(read_map, DXL_LEGACY_PRESENT_VOLTAGE, 2); // bulk method
      }
      else
      {
        for (auto& it : diag_map)
        {
          if(readRegisters(it.first, DXL_LEGACY_PRESENT_VOLTAGE, 2, &it.second.data)) // individual reads
          {
            it.second.success = true;
          }
          else
          {
            it.second.success = false;
          }

        }
      }
      for (auto& it : diag_map) // decode responses
      {
        if (it.second.success)
        {
          it.second.voltage = it.second.data[0];
          it.second.temperature = it.second.data[1];
        }
      }
      return success;

    // NEW MX AND X SERIES
    case DXL_MX:
    case DXL_X:
      // read data
      if(use_group_read_)
      {
        syncRead(read_map, DXL_STANDARD_PRESENT_INPUT_VOLTAGE, 3); // bulk method
      }
      else
      {
        for (auto& it : diag_map)
        {
          if(readRegisters(it.first, DXL_STANDARD_PRESENT_INPUT_VOLTAGE, 3, &it.second.data)) // individual reads
          {
            it.second.success = true;
          }
          else
          {
            it.second.success = false;
          }

        }
      }
      for (auto& it : diag_map) // decode responses
      {
        if (it.second.success)
        {
          it.second.voltage = *((uint16_t*) &it.second.data[0]);
          it.second.temperature = it.second.data[2];
        }
      }
      return success;

    // NEW PRO SERIES
    case DXL_P:
      // read data
      if(use_group_read_)
      {
        syncRead(read_map, DXL_P_PRESENT_INPUT_VOLTAGE, 3); // bulk method
      }
      else
      {
        for (auto& it : diag_map)
        {
          if(readRegisters(it.first, DXL_P_PRESENT_INPUT_VOLTAGE, 3, &it.second.data)) // individual reads
          {
            it.second.success = true;
          }
          else
          {
            it.second.success = false;
          }

        }
      }
      for (auto& it : diag_map) // decode responses
      {
        if (it.second.success)
        {
          it.second.voltage = *((uint16_t*) &it.second.data[0]);
          it.second.temperature = it.second.data[2];
        }
      }
      return success;

    // OLD PRO SERIES
    case DXL_LEGACY_PRO:
      // read data
      if(use_group_read_)
      {
        syncRead(read_map, DXL_LEGACY_PRO_PRESENT_VOLTAGE, 3); // bulk method
      }
      else
      {
        for (auto& it : diag_map)
        {
          if(readRegisters(it.first, DXL_LEGACY_PRO_PRESENT_VOLTAGE, 3, &it.second.data)) // individual reads
          {
            it.second.success = true;
          }
          else
          {
            it.second.success = false;
          }

        }
      }
      for (auto& it : diag_map) // decode responses
      {
        if (it.second.success)
        {
          it.second.voltage = *((uint16_t*) &it.second.data[0]);
          it.second.temperature = it.second.data[2];
        }
      }
      return success;

    default:
      return false;
  }

  return false;

}


// ********************** Protected Read Methods *********************** //

/// Performs the bulk read for each protocol. A bulk read is a broadcast instruction on a bus that commands a list
/// of dynamixels to respond in order with a read of a specified address and length (which can be different for each
/// dynamixel). This protocol can be used to read many parameters from many dynamixels on a bus in just one
/// instruction.
/// @param read_data Pointer to a map of SyncData objects, containing ids and vectors to read into
/// @param address The address value in the control table the dynamixels will start reading from
/// @param length The number of bytes to read consecutively from the control table
/// @returns true if at least one dynamixel was successfully read
bool DynamixelInterfaceDriver::bulkRead(std::unordered_map<int, SyncData*> &read_data, uint16_t address, uint16_t length)
{
  int dxl_comm_result;
  bool success = false;

  // Initialize GroupSyncRead instance
  dynamixel::GroupBulkRead GroupBulkRead(portHandler_, packetHandler_);

  // add all dynamixels to sync read
  for (auto& it : read_data)
  {
    GroupBulkRead.addParam(it.first, address, length);
    // clear data in response object
    it.second->data.clear();
  }

  //perform sync read
  dxl_comm_result = GroupBulkRead.txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS)
  {
    return false;
  }

  //get all responses back from read
  for (auto& it : read_data)
  {
    // if data available
    if(GroupBulkRead.isAvailable(it.first, address, length))
    {
      // have at least one response
      success = true;

      // Get values from read and place into vector
      for (int j = 0; j < length; j++)
      {
        it.second->data.push_back(GroupBulkRead.getData(it.first, address + j, 1));
      }
      it.second->success = true;
    }
    else
    {
      it.second->success = false;
    }
  }

  if (!success)
  {
    return false;
  }
  else
  {
    return true;
  }

}

/// Performs the sync read for each protocol. A sync read is a broadcast instruction on a bus that commands a list
/// of dynamixels to respond in order with a read of a specified address and length. This protocol can be used to read
/// many parameters from many dynamixels on a bus in just one instruction.
/// @param read_data Pointer to a map of SyncData objects, containing ids and vectors to read into
/// @param address The address value in the control table the dynamixels will start reading from
/// @param length The number of bytes to read consecutively from the control table
/// @returns true if at least one dynamixel was successfully read
bool DynamixelInterfaceDriver::syncRead(std::unordered_map<int, SyncData*> &read_data, uint16_t address, uint16_t length)
{
  int dxl_comm_result;
  bool success = false;

  if (use_legacy_protocol_)
  {
    return false;
  }

  // Initialize GroupSyncRead instance
  dynamixel::GroupSyncRead GroupSyncRead(portHandler_, packetHandler_, address, length);

  // add all dynamixels to sync read
  for (auto& it : read_data)
  {
    GroupSyncRead.addParam(it.first);
    // clear data in response object
    it.second->data.clear();
  }

  //perform sync read
  dxl_comm_result = GroupSyncRead.txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS)
  {
    return false;
  }

  //get all responses back from read
  for (auto& it : read_data)
  {
    // if data available
    if(GroupSyncRead.isAvailable(it.first, address, length))
    {
      // have at least one response
      success = true;
      // Get values from read and place into vector
      for (int j = 0; j < length; j++)
      {
        it.second->data.push_back(GroupSyncRead.getData(it.first, address + j, 1));
      }
      it.second->success = true;
    }
    else
    {
      it.second->success = false;
    }
  }

  if (!success)
  {
    return false;
  }
  else
  {
    return true;
  }
}


// **************************** SETTERS ******************************** //

/// Sets the Operating mode of the Dynamixel. The three possible control modes are: Position, Velocity or Torque.
/// The way these modes are enabled differs for each series of dynamixel. XM and Pro have an operating mode register
/// that can be used to changed the control type. For AX, RX and MX series, the method of control is defined by the
/// values in the angle limit registers and the torque_control_enable register. Note torque control is not available on
/// the MX-28 models.
/// @param servo_id The ID of the servo to write to
/// @param operating_mode The method of control
/// @return True on comm success and valid operating mode, false otherwise.
bool DynamixelInterfaceDriver::setOperatingMode(int servo_id, DynamixelSeriesType type, DynamixelControlMode operating_mode)
{
  uint8_t error;
  int dxl_comm_result = 0;
  uint16_t model_num;
  bool success = true;

  //Read address and size always depends on servo series
  switch(type)
  {

    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:

      //MX Series has no operating mode register, instead operating mode depends on
      //angle limits and torque_control_enable register
      switch (operating_mode)
      {

        case POSITION_CONTROL:

          //Position control, set normal angle limits
          success = setAngleLimits(servo_id, type, 0, 4095); //Master mode, normal roatation

          //Turn off Torque control mode if there is actually a register for this
          success = getModelNumber(servo_id, &model_num);

          if (success && ((model_num == 310) || (model_num == 320))) // only MX-64 and MX-106
          {
            dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_TORQUE_CONTROL_ENABLE,
                0, &error);
          }
          break;

        case VELOCITY_CONTROL:

          //Velocity control, turn off angle limits
          success = setAngleLimits(servo_id, type, 0, 0); //Master mode, normal roatation

          //Torque control mode, there is actually a register for this
          success = getModelNumber(servo_id, &model_num);

          if (success & ((model_num == 310) || (model_num == 320))) // only MX-64 and MX-106
          {
            dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_TORQUE_CONTROL_ENABLE,
                0, &error);
          }
          break;

        case TORQUE_CONTROL:

          //Torque control mode, only MX-64 and MX-106
          success = getModelNumber(servo_id, &model_num);

          if (success && ((model_num == 310) || (model_num == 320)))
          {
            //Turn off angle limits
            success = setAngleLimits(servo_id, type, 0, 0); //Master mode, normal roatation
            ROS_INFO("torque_control_enabled");
            dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_TORQUE_CONTROL_ENABLE,
                1, &error);
          }
          break;

      }

    case DXL_MX:
    case DXL_X:

      if ((operating_mode == TORQUE_CONTROL) || (operating_mode == CURRENT_BASED_POSITION_CONTROL))
      {
        success = getModelNumber(servo_id, &model_num);

        if (success && (model_num != 30))
        {
          dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_STANDARD_OPERATING_MODE,
            operating_mode, &error);
        }
      }
      else
      {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_STANDARD_OPERATING_MODE,
          operating_mode, &error);
      }
      break;

    case DXL_P:

      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_P_OPERATING_MODE,
          operating_mode, &error);
      break;

    case DXL_LEGACY_PRO:

      if (operating_mode != PWM_CONTROL)
      {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_OPERATING_MODE,
            operating_mode, &error);
      }
      break;

    default:
      return false;

  }

  // check return value
  if ((dxl_comm_result == COMM_SUCCESS) && (success))
  {
    return true;
  }
  else
  {
    return false;
  }
}


/// Sets the minimum and maximum angle limits for the dynamixel
/// @param servo_id The ID of the servo to write to
/// @param min_angle the minimum angle limit (in encoder values)
/// @param max_angle the maximum angle limit (in encoder values)
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setAngleLimits(int servo_id, DynamixelSeriesType type, int32_t min_angle, int32_t max_angle)
{

  if (setMaxAngleLimit(servo_id, type, max_angle) == true)
  {
    return setMinAngleLimit(servo_id, type, min_angle);
  }
  else
  {
    return false;
  }

}

/// Sets the minimum angle limit for the dynamixel
/// @param servo_id The ID of the servo to write to
/// @param angle The minimum angle limit (in encoder values)
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setMinAngleLimit(int servo_id, DynamixelSeriesType type, int32_t angle)
{

  uint8_t error;
  int dxl_comm_result;

  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_CW_ANGLE_LIMIT,
        (int16_t) angle, &error);
      break;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, servo_id, DXL_STANDARD_MIN_POSITION_LIMIT,
          angle, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, servo_id, DXL_P_MIN_POSITION_LIMIT,
        angle, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_MIN_ANGLE_LIMIT,
        angle, &error);
      break;

  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}

/// Sets the maximum angle limit for the dynamixel
/// @param servo_id The ID of the servo to write to
/// @param angle The maximum angle limit (in encoder values)
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setMaxAngleLimit(int servo_id, DynamixelSeriesType type, int32_t angle)
{

  uint8_t error;
  int dxl_comm_result;

  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_CCW_ANGLE_LIMIT,
        (int16_t) angle, &error);
      break;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, servo_id, DXL_STANDARD_MAX_POSITION_LIMIT,
          angle, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, servo_id, DXL_P_MAX_POSITION_LIMIT,
        angle, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_MAX_ANGLE_LIMIT,
        angle, &error);
      break;

  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}

/// Sets the maximum torque limit for the dynamixel
/// @param servo_id The ID of the servo to write to
/// @param max_torque the maximum torque limit
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setMaxTorque(int servo_id, DynamixelSeriesType type, uint16_t max_torque)
{

  uint8_t error;
  int dxl_comm_result;

  //Read address and size always depends on servo series
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_MAX_TORQUE,
          max_torque, &error);
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_TORQUE_LIMIT,
          max_torque, &error);

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_STANDARD_CURRENT_LIMIT,
          max_torque, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_P_CURRENT_LIMIT,
          max_torque, &error);
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_P_GOAL_CURRENT,
          max_torque, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_MAX_TORQUE,
          max_torque, &error);
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_GOAL_TORQUE,
          max_torque, &error);
      break;

  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }


}

/// Sets the torque enable register of the dynamixel. This value defines the on/off state of the servo.
/// @param servo_id The ID of the servo to write to
/// @param on The state of the servo (true = on, false = off).
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setTorqueEnabled(int servo_id, DynamixelSeriesType type, bool on)
{

  uint8_t error;
  int dxl_comm_result;

  //Read address and size always depends on servo series
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_TORQUE_ENABLE,
          (uint8_t) on, &error);
      break;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_STANDARD_TORQUE_ENABLE,
          (uint8_t) on, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_P_TORQUE_ENABLE,
          (uint8_t) on, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_TORQUE_ENABLE,
          (uint8_t) on, &error);
      break;

  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}

/// Sets the torque control enable register of the dynamixel mx series. can be used to dynamically
/// switch between position and torque control modes.
/// @param on The torque control state of the servo (true = on, false = off).
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setTorqueControlEnabled(int servo_id, DynamixelSeriesType type, bool on)
{

  uint8_t error;
  uint16_t model_num;
  int dxl_comm_result;

  //Torque control mode, there is actually a register for this
  getModelNumber(servo_id, &model_num);
  if ( (model_num == 310) || (model_num == 320) ) //No torque control on MX-28
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_TORQUE_CONTROL_ENABLE,
        (uint8_t) (on), &error);
  }
  else
  {
    return false;
  }



  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}


/// Sets the position PID values for the dynamixels
/// @param servo_id The ID of the servo to write to
/// @param operating_mode The operating mode to set gains for. @see setOperatingMode
/// @param p_gain The proportional gain value to write
/// @param i_gain The integral gain value to write
/// @param d_gain The derivative gain value to write
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setPositionPIDGains(int servo_id, DynamixelSeriesType type, double p_gain, double i_gain, double d_gain)
{

  uint16_t p_val, i_val, d_val;

  //Convert values based on servo series
  switch(type)
  {
    case DXL_LEGACY_MX:
      p_val = (uint16_t) (p_gain * 8.0);
      i_val = (uint16_t) (i_gain * (1000.0 / 2048.0));
      d_val = (uint16_t) (d_gain / (4.0 / 1000.0));
      break;

    case DXL_MX:
    case DXL_X:
      p_val = (uint16_t) (p_gain * 128.0);		//possible 0-127
      i_val = (uint16_t) (i_gain * 65536.0);	//by the same logic, 0-0.25
      d_val = (uint16_t) (d_gain * 16.0);			//0-1024
      break;

    case DXL_P:
      p_val = (uint16_t) p_gain;
      i_val = 0;
      d_val = 0;
      break;

    default:
      return false;
  }

  //set the register values for the given control mode,
  //This changes based on servo series, a value of gain < 0
  //indicates that the gain should be left to the default
  if (!(p_gain < 0))
  {
    if(!setPositionProportionalGain(servo_id, type, p_val))
    {
      return false;
    }
  }

  if (!(i_gain < 0) && (type != DXL_LEGACY_PRO))
  {
    if(!setPositionIntegralGain(servo_id, type, i_val))
    {
      return false;
    }
  }

  if (!(d_gain < 0) && (type != DXL_LEGACY_PRO))
  {
    if(!setPositionDerivativeGain(servo_id, type, d_val))
    {
      return false;
    }
  }

  return true;

}

/// Sets the proportional gain value for the position control mode if available. @see setPIDGains
/// @param servo_id The ID of the servo to write to
/// @param gain The proportional gain value to write
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setPositionProportionalGain(int servo_id, DynamixelSeriesType type, uint16_t gain)
{

  uint8_t error;
  int dxl_comm_result;

  //Read address and size always depends on servo series
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
      return false;

    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_P_GAIN,
          (uint8_t) (gain & 0x00FF), &error);
      break;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_STANDARD_POSITION_P_GAIN,
          gain, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_P_POSITION_P_GAIN,
          gain, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_POSITION_P_GAIN,
          gain, &error);
      break;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}

/// Sets the integral gain value for the position control mode if available. @see setPIDGains
/// @param servo_id The ID of the servo to write to
/// @param gain The integral gain value to write
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setPositionIntegralGain(int servo_id, DynamixelSeriesType type, uint16_t gain)
{

  uint8_t error;
  int dxl_comm_result;

  //Read address and size always depends on servo series
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
      return false;

    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_I_GAIN,
          (uint8_t) (gain & 0x00FF), &error);
      break;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_STANDARD_POSITION_I_GAIN,
          gain, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_P_POSITION_I_GAIN,
          gain, &error);
      break;

    case DXL_LEGACY_PRO:
      return false;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/// Sets the derivative gain value for the position control mode if available. @see setPIDGains
/// @param servo_id The ID of the servo to write to
/// @param gain The derivative gain value to write
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setPositionDerivativeGain(int servo_id, DynamixelSeriesType type, uint16_t gain)
{

  uint8_t error;
  int dxl_comm_result;

  //Read address and size always depends on servo series
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
      return false;

    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, servo_id, DXL_LEGACY_D_GAIN,
          (uint8_t) (gain & 0x00FF), &error);
      break;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_STANDARD_POSITION_D_GAIN,
          gain, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_P_POSITION_D_GAIN,
          gain, &error);
      break;

    case DXL_LEGACY_PRO:
      return false;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }
}


/// Sets the velocity PID values for the dynamixels
/// @param servo_id The ID of the servo to write to
/// @param operating_mode The operating mode to set gains for. @see setOperatingMode
/// @param p_gain The proportional gain value to write
/// @param i_gain The integral gain value to write
/// @param d_gain The derivative gain value to write
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setVelocityPIDGains(int servo_id, DynamixelSeriesType type, double p_gain, double i_gain)
{

  uint16_t p_val, i_val, d_val;

  //Convert values based on servo series
  switch(type)
  {
    case DXL_MX:
    case DXL_X:
      p_val = (uint16_t) (p_gain * 128.0);  //possible 0-127
      i_val = (uint16_t) (i_gain * 65536.0);  //by the same logic, 0-0.25
      break;

    case DXL_P:
      p_val = (uint16_t) p_gain;
      i_val = (uint16_t) i_gain;
      break;

    default:
      return false;
  }

  //set the register values for the given control mode,
  //This changes based on servo series, a value of gain < 0
  //indicates that the gain should be left to the default
  if (!(p_gain < 0))
  {
    if(!setVelocityProportionalGain(servo_id, type, p_val))
    {
      return false;
    }
  }

  if (!(i_gain < 0))
  {
    if(!setVelocityIntegralGain(servo_id, type, i_val))
    {
      return false;
    }
  }

  return true;

}

/// Sets the proportional gain value for the velocity control mode if available. @see setPIDGains
/// @param servo_id The ID of the servo to write to
/// @param gain The proportional gain value to write
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setVelocityProportionalGain(int servo_id, DynamixelSeriesType type, uint16_t gain)
{

  uint8_t error;
  int dxl_comm_result;

  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      return false;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_STANDARD_VELOCITY_P_GAIN,
          gain, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_P_VELOCITY_P_GAIN,
          gain, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_VELOCITY_P_GAIN,
          gain, &error);
      break;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/// Sets the integral gain value for the velocity control mode if available. @see setPIDGains
/// @param servo_id The ID of the servo to write to
/// @param gain The integral gain value to write
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setVelocityIntegralGain(int servo_id, DynamixelSeriesType type, uint16_t gain)
{
  uint8_t error;
  int dxl_comm_result;

  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      return false;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_STANDARD_VELOCITY_I_GAIN,
          gain, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_P_VELOCITY_I_GAIN,
          gain, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_VELOCITY_I_GAIN,
          gain, &error);
      break;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/// Sets the profile velocity of the dynamixel. Profile velocity is how fast the servo should move between positions.
/// @param servo_id The ID of the servo to write to
/// @param velocity The profile velocity value to write
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setProfileVelocity(int servo_id, DynamixelSeriesType type, int32_t velocity)
{
  //Read address and size always depends on servo series
  uint8_t error;
  int dxl_comm_result;

  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_MOVING_SPEED,
          (uint16_t) velocity, &error);
      return false;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, servo_id, DXL_STANDARD_PROFILE_VELOCITY,
          velocity, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, servo_id, DXL_P_PROFILE_VELOCITY,
          velocity, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_GOAL_VELOCITY,
          velocity, &error);
      break;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}

/// Sets the torque value of the dynamixel.
/// @param servo_id The ID of the servo to write to
/// @param torque The torque value to write
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setTorque(int servo_id, DynamixelSeriesType type, int16_t torque)
{

  uint8_t error;
  int dxl_comm_result;

  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_GOAL_TORQUE,
          torque, &error);
      return false;

    case DXL_MX:
    case DXL_X:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_STANDARD_GOAL_CURRENT,
          torque, &error);
      break;

    case DXL_P:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_P_GOAL_CURRENT,
          torque, &error);
      break;

    case DXL_LEGACY_PRO:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, servo_id, DXL_LEGACY_PRO_GOAL_TORQUE,
          torque, &error);
      break;
  }

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}

/// Writes arbitrary register values to the Dynamixel, can be used in cases where provided setters are insufficient
/// or to write multiple contiguous values at once.
/// @param servo_id The ID of the servo to write to
/// @param address The address value in the control table the dynamixel will start writing to
/// @param length The number of bytes to write consecutively to the control table
/// @param data Array containing the value to be written.
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::writeRegisters(int servo_id, uint16_t address, uint16_t length, uint8_t *data)
{

  uint8_t error;
  int dxl_comm_result;

  //Read address and size always depends on servo series
  dxl_comm_result = packetHandler_->writeTxRx(portHandler_, servo_id, address, length, data, &error);

  // check return value
  if (dxl_comm_result == COMM_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }

}

// *********************** SYNC_WRITE METHODS *************************** //

/// Set many dynamixels with new position values in one instruction. @see syncWrite.
/// @param position_data  map of ids to syncdata objects containing position data
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setMultiPosition(std::unordered_map<int, SyncData> &position_data)
{

  DynamixelSeriesType type;
  uint i = 0;

  // check types match
  for (auto& it : position_data)
  {
    if (i == 0)
    {
      type = it.second.type;
    }
    else if ((i > 0) && (type != it.second.type))
    {
      return false;
    }
    i++;
  }

  // switch on series type
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      return syncWrite(position_data, DXL_LEGACY_GOAL_POSITION, 2);
    case DXL_MX:
    case DXL_X:
      return syncWrite(position_data, DXL_STANDARD_GOAL_POSITION, 4);
    case DXL_P:
      return syncWrite(position_data, DXL_P_GOAL_POSITION, 4);
    case DXL_LEGACY_PRO:
      return syncWrite(position_data, DXL_LEGACY_PRO_GOAL_POSITION, 4);
    default:
      return false;
  }
  return false;
}

/// Set many dynamixels with new velocity values in one instruction. @see syncWrite.
/// @param velocity_data  map of ids to syncdata objects containing velocity data
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setMultiVelocity(std::unordered_map<int, SyncData> &velocity_data)
{

  DynamixelSeriesType type;
  uint i = 0;

  // check types match
  for (auto& it : velocity_data)
  {
    if (i == 0)
    {
      type = it.second.type;
    }
    else if ((i > 0) && (type != it.second.type))
    {
      return false;
    }
    i++;
  }

  // switch on series type
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      return syncWrite(velocity_data, DXL_LEGACY_MOVING_SPEED, 2);
    case DXL_MX:
    case DXL_X:
      return syncWrite(velocity_data, DXL_STANDARD_GOAL_VELOCITY, 4);
    case DXL_P:
      return syncWrite(velocity_data, DXL_P_GOAL_VELOCITY, 4);
    case DXL_LEGACY_PRO:
      return syncWrite(velocity_data, DXL_LEGACY_PRO_GOAL_VELOCITY, 4);
    default:
      return false;
  }
  return false;
}

/// Set many dynamixels with new profile velocity values in one instruction. @see syncWrite.
/// @param velocity_data  map of ids to syncdata objects containing velocity data
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setMultiProfileVelocity(std::unordered_map<int, SyncData> &velocity_data)
{

  DynamixelSeriesType type;
  uint i = 0;

  // check types match
  for (auto& it : velocity_data)
  {
    if (i == 0)
    {
      type = it.second.type;
    }
    else if ((i > 0) && (type != it.second.type))
    {
      return false;
    }
    i++;
  }

  // switch on series type
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
    case DXL_LEGACY_MX:
      return syncWrite(velocity_data, DXL_LEGACY_MOVING_SPEED, 2);
    case DXL_MX:
    case DXL_X:
      return syncWrite(velocity_data, DXL_STANDARD_PROFILE_VELOCITY, 4);
    case DXL_P:
      return syncWrite(velocity_data, DXL_P_PROFILE_VELOCITY, 4);
    case DXL_LEGACY_PRO:
      return syncWrite(velocity_data, DXL_LEGACY_PRO_GOAL_VELOCITY, 4);
    default:
      return false;
  }
  return false;
}

/// Set many dynamixels with new torque values in one instruction. @see syncWrite.
/// @param torque_data map of ids to syncdata objects containing torque data
/// @return True on comm success, false otherwise.
bool DynamixelInterfaceDriver::setMultiTorque(std::unordered_map<int, SyncData> &torque_data)
{

  DynamixelSeriesType type;
  uint i = 0;

  // check types match
  for (auto& it : torque_data)
  {
    if (i == 0)
    {
      type = it.second.type;
    }
    else if ((i > 0) && (type != it.second.type))
    {
      return false;
    }
    i++;
  }

  // switch on series type
  switch(type)
  {
    case DXL_AX:
    case DXL_RX:
      return false;
    case DXL_LEGACY_MX:
      return syncWrite(torque_data, DXL_LEGACY_GOAL_TORQUE, 2);
    case DXL_MX:
    case DXL_X:
      return syncWrite(torque_data, DXL_STANDARD_GOAL_CURRENT, 2);
    case DXL_P:
      return syncWrite(torque_data, DXL_P_GOAL_CURRENT, 2);
    case DXL_LEGACY_PRO:
      return syncWrite(torque_data, DXL_LEGACY_PRO_GOAL_TORQUE, 2);
    default:
      return false;
  }
  return false;
}

// ********************** Protected Write Methods *********************** //

/// Performs the sync write for each protocol. A sync write is a broadcast instruction on a bus that commands a list
/// of dynamixels to write a value into a specified address (the value written can be different for each dynamixel
/// but the address is universal). This can be used to update a parameter (say goal position) for many dynamixels,
/// each with a unique value, all in one instruction.
/// @param write_data Pointer to a map of SyncData objects, containing ids and vectors to write from.
/// @param address The address value in the control table the dynamixels will write to
/// @param length The number of bytes to write
/// @returns true on successful write, false otherwise
bool DynamixelInterfaceDriver::syncWrite(std::unordered_map<int, SyncData> &write_data, uint16_t address, uint16_t length)
{
  uint8_t dxl_comm_result;
  bool success = true;

  if (use_group_write_)
  {
    // Initialize GroupSyncWrite instance
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, address, length);

    //Add parameters
    for (auto& it : write_data)
    {
      // only write data that matches length
      if (it.second.data.size() == length)
      {
        groupSyncWrite.addParam(it.first, it.second.data.data());
      }
    }

    //Transmit packet and check success
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    //loop and write to dynamixels
    for (auto& it : write_data)
    {
      if(!writeRegisters(it.first, address, length, it.second.data.data()))
      {
        success = false;
      }
    }
    return success;
  }
}

}
