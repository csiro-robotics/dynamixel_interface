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
 * @file   dynamixel_interface_driver.h
 * @author Tom Molnar (Tom.Molnar@data61.csiro.au), Brian Axelrod
 * @date   January, 2017
 * @brief  Defines the hardware abstraction methods for communicating with dynamixels
 */


#ifndef DYNAMIXEL_INTERFACE_DRIVER_H_
#define DYNAMIXEL_INTERFACE_DRIVER_H_

#include <stdint.h>
#include <map>
#include <unordered_map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_interface_driver/dynamixel_new_const.h>


namespace dynamixel_interface_driver
{

/// Struct that describes the dynamixel motor's static and physical properties
typedef struct
{
  std::string name;         /// The Model Name
  uint16_t model_number;    /// Model number (e.g 29 = MX-28)
  DynamixelSeriesType type; /// Model type (e.g MX, AX, Pro)
  int cpr;                  /// Motor encoder counts per revolution
  double gear_conversion;   /// Gear reduction ratio
  double effort_ratio;      /// Torque ratio
  double current_ratio;     /// Current ratio
} DynamixelSpec;


/// Structs for performing reads and writes
typedef struct _SyncData
{
  int id;
  DynamixelSeriesType type;
  bool success;
  std::vector<uint8_t> data;
} SyncData;

typedef struct _DynamixelState : SyncData
{
  int32_t position;
  int32_t velocity;
  int32_t effort;
} DynamixelState;

typedef struct _DynamixelDiagnostic : SyncData
{
  int32_t voltage;
  int32_t temperature;
} DynamixelDiagnostic;

/// Provides the handling of the low level communications between the
/// dynamixels and the controller
class DynamixelInterfaceDriver
{
public:

  /// Constructor. Initialises port and packet handling objects and sets the
  /// baud rate.
  /// @param[in] device The serial port to connect to
  /// @param[in] baud The baud rate to use
  /// @param[optional] use_legacy_protocol Whether to use legacy 1.0 protocol (default false)
  /// @param[optional] use_group_read Whether to use bulk protocol methods with bulk getters (default true)
  /// @param[optional] use_group_read Whether to use bulk protocol methods with bulk setters (default true)
  DynamixelInterfaceDriver(std::string device, int baud, bool use_legacy_protocol, bool use_group_read,
          bool use_group_write);

  /// Destructor. Closes and releases serial port.
  ~DynamixelInterfaceDriver();

  /// Ping the specified id, used to check if a dynamixel with that ID is on the bus
  /// @param[in] servo_id The ID to ping on the bus.
  /// @return True if a dynamixel responds, false otherwise.
  bool ping(int servo_id);

  /// Get pointer to model spec data for given model number
  inline const DynamixelSpec* getModelSpec(uint model_number)
  {
    if (model_specs_.find(model_number) != model_specs_.end())
    {
      return &model_specs_.at(model_number);
    }
    else
    {
      return NULL;
    }
  }

  // ************************************ GETTERS ***************************************** //

  /// Retrieves the model number from the dynamixel's eeprom
  /// @param[in] servo_id The ID of the servo to retrieve from
  /// @param[out] model_number Stores the model_number returned
  /// @return True on comm success, false otherwise.
  bool getModelNumber(int servo_id, uint16_t* model_number);

  /// Retrieves the maximum torque limit from the dynamixel's eeprom.
  /// @param[in] servo_id The ID of the servo to retrieve from
  /// @param[in] type the type of the servo to read from
  /// @param[out] max_torque Stores the value returned
  /// @return True on comm success, false otherwise.
  bool getMaxTorque(int servo_id, DynamixelSeriesType type, uint16_t* max_torque);

  /// Retrieves the torque enabled value from the dynamixel's ram.
  /// @param servo_id The ID of the servo to retrieve from
  /// @param torque_enabled Stores the status of torque enable
  /// @return True on comm success, false otherwise.
  bool getTorqueEnabled(int servo_id, DynamixelSeriesType type, bool* torque_enabled);

  /// Retrieves the current target_velocity from the dynamixel's ram.
  /// @param servo_id The ID of the servo to retrieve from
  /// @param target_velocity Stores the value returned
  /// @return True on comm success, false otherwise.
  bool getTargetTorque(int servo_id, DynamixelSeriesType type, int16_t* target_torque);

  /// Retrieves arbitrary register readings from the Dynamixel, can be used in cases where provided getters are
  /// insufficient or to read multiple contiguous values at once.
  /// @param servo_id The ID of the servo to retrieve from
  /// @param address The address value in the control table the dynamixel will start reading from
  /// @param length The number of bytes to read consecutively from the control table
  /// @param response Array to store the raw dynamixel response.
  bool readRegisters(int servo_id, uint16_t address, uint16_t length, std::vector<uint8_t> *response);

  // *********************** BULK_READ METHODS *************************** //

  /// Bulk Reads the Present Position, Present Velocity and Present Current in one instruction. If the group read fails
  /// the function will fall back on reading each motor individually. Optionally, the group comms can be disabled on
  /// initialisation of the driver (by setting use_group_read to false) in which case the function will always read
  /// from each dynamixel individually.
  /// @param state_map map of servo ids to state data to read into
  /// @return True on comm success, false otherwise
  bool getBulkState(std::unordered_map<int, DynamixelState> &state_map);

  /// Bulk Reads the voltage and temperature in one instruction, behaves the same as getBulkState()
  /// @param diag_map map of servo ids to diag_data to read into
  /// @return True on comm success, false otherwise
  bool getBulkDiagnosticInfo(std::unordered_map<int, DynamixelDiagnostic> &diag_map);

  // **************************** SETTERS ******************************** //

  /// Sets the Operating mode of the Dynamixel. The three possible control modes are: Position, Velocity or Torque.
  /// The way these modes are enabled differs for each series of dynamixel. XM and Pro have an operating mode register
  /// that can be used to changed the control type. For AX, RX and MX series, the method of control is defined by the
  /// values in the angle limit registers and the torque_control_enable register. Note torque control is not available on
  /// the MX-28 models.
  /// @param servo_id The ID of the servo to write to
  /// @param operating_mode The method of control
  /// @return True on comm success and valid operating mode, false otherwise.
  bool setOperatingMode(int servo_id, DynamixelSeriesType type, DynamixelControlMode operating_mode);

  /// Sets the minimum and maximum angle limits for the dynamixel
  /// @param servo_id The ID of the servo to write to
  /// @param min_angle the minimum angle limit (in encoder values)
  /// @param max_angle the maximum angle limit (in encoder values)
  /// @return True on comm success, false otherwise.
  bool setAngleLimits(int servo_id, DynamixelSeriesType type, int32_t min_angle, int32_t max_angle);

  /// Sets the minimum angle limit for the dynamixel
  /// @param servo_id The ID of the servo to write to
  /// @param angle The minimum angle limit (in encoder values)
  /// @return True on comm success, false otherwise.
  bool setMinAngleLimit(int servo_id, DynamixelSeriesType type, int32_t angle);

  /// Sets the maximum angle limit for the dynamixel
  /// @param servo_id The ID of the servo to write to
  /// @param angle The maximum angle limit (in encoder values)
  /// @return True on comm success, false otherwise.
  bool setMaxAngleLimit(int servo_id, DynamixelSeriesType type, int32_t angle);

  /// Sets the maximum torque limit for the dynamixel
  /// @param servo_id The ID of the servo to write to
  /// @param max_torque the maximum torque limit
  /// @return True on comm success, false otherwise.
  bool setMaxTorque(int servo_id, DynamixelSeriesType type, uint16_t max_torque);

  /// Sets the torque enable register of the dynamixel. This value defines the on/off state of the servo.
  /// @param servo_id The ID of the servo to write to
  /// @param on The state of the servo (true = on, false = off).
  /// @return True on comm success, false otherwise.
  bool setTorqueEnabled(int servo_id, DynamixelSeriesType type, bool on);

  /// Sets the torque control enable register of the dynamixel mx series. can be used to dynamically
  /// switch between position and torque control modes.
  /// @param on The torque control state of the servo (true = on, false = off).
  /// @return True on comm success, false otherwise.
  bool setTorqueControlEnabled(int servo_id, DynamixelSeriesType type, bool on);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// The following functions set the PID gains for the dynamixel. note that the PID gains are available based on
  /// series and control type:
  ///  - AX Series:
  ///      - None
  ///  - RX Series:
  ///      - None
  ///  - MX (1.0) Series:
  ///      - Position: P,I,D
  ///      - Velocity: None
  ///      - Torque:   None
  ///  - XM Series:
  ///      - Position: P,I,D
  ///      - Velocity: P,I
  ///      - Torque:   None
  ///  - Pro Series:
  ///      - Position: P
  ///      - Velocity: P,I
  ///      - Torque:   None
  ///  -
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /// Sets the position PID values for the dynamixels
  /// @param servo_id The ID of the servo to write to
  /// @param operating_mode The operating mode to set gains for. @see setOperatingMode
  /// @param p_gain The proportional gain value to write
  /// @param i_gain The integral gain value to write
  /// @param d_gain The derivative gain value to write
  /// @return True on comm success, false otherwise.
  bool setPositionPIDGains(int servo_id, DynamixelSeriesType type, double p_gain, double i_gain, double d_gain);

  /// Sets the proportional gain value for the position control mode if available. @see setPIDGains
  /// @param servo_id The ID of the servo to write to
  /// @param gain The proportional gain value to write
  /// @return True on comm success, false otherwise.
  bool setPositionProportionalGain(int servo_id, DynamixelSeriesType type, uint16_t gain);

  /// Sets the integral gain value for the position control mode if available. @see setPIDGains
  /// @param servo_id The ID of the servo to write to
  /// @param gain The integral gain value to write
  /// @return True on comm success, false otherwise.
  bool setPositionIntegralGain(int servo_id, DynamixelSeriesType type, uint16_t gain);

  /// Sets the derivative gain value for the position control mode if available. @see setPIDGains
  /// @param servo_id The ID of the servo to write to
  /// @param gain The derivative gain value to write
  /// @return True on comm success, false otherwise.
  bool setPositionDerivativeGain(int servo_id, DynamixelSeriesType type, uint16_t gain);

  /// Sets the velocity PID values for the dynamixels
  /// @param servo_id The ID of the servo to write to
  /// @param operating_mode The operating mode to set gains for. @see setOperatingMode
  /// @param p_gain The proportional gain value to write
  /// @param i_gain The integral gain value to write
  /// @param d_gain The derivative gain value to write
  /// @return True on comm success, false otherwise.
  bool setVelocityPIDGains(int servo_id, DynamixelSeriesType type, double p_gain, double i_gain);

  /// Sets the proportional gain value for the velocity control mode if available. @see setPIDGains
  /// @param servo_id The ID of the servo to write to
  /// @param gain The proportional gain value to write
  /// @return True on comm success, false otherwise.
  bool setVelocityProportionalGain(int servo_id, DynamixelSeriesType type, uint16_t gain);

  /// Sets the integral gain value for the velocity control mode if available. @see setPIDGains
  /// @param servo_id The ID of the servo to write to
  /// @param gain The integral gain value to write
  /// @return True on comm success, false otherwise.
  bool setVelocityIntegralGain(int servo_id, DynamixelSeriesType type, uint16_t gain);

  /// Sets the profile velocity of the dynamixel. Profile velocity is how fast the servo should move between positions.
  /// @param servo_id The ID of the servo to write to
  /// @param velocity The profile velocity value to write
  /// @return True on comm success, false otherwise.
  bool setProfileVelocity(int servo_id, DynamixelSeriesType type, int32_t velocity);

  /// Sets the torque value of the dynamixel.
  /// @param servo_id The ID of the servo to write to
  /// @param torque The torque value to write
  /// @return True on comm success, false otherwise.
  bool setTorque(int servo_id, DynamixelSeriesType type, int16_t torque);

  /// Writes arbitrary register values to the Dynamixel, can be used in cases where provided setters are insufficient
  /// or to write multiple contiguous values at once.
  /// @param servo_id The ID of the servo to write to
  /// @param address The address value in the control table the dynamixel will start writing to
  /// @param length The number of bytes to write consecutively to the control table
  /// @param data Array containing the value to be written.
  /// @return True on comm success, false otherwise.
  bool writeRegisters(int servo_id, uint16_t address, uint16_t length, uint8_t *data);

  // *********************** SYNC_WRITE METHODS *************************** //

  /// Set many dynamixels with new position values in one instruction. @see syncWrite.
  /// @param position_data  map of ids to syncdata objects containing position data
  /// @return True on comm success, false otherwise.
  bool setMultiPosition(std::unordered_map<int, SyncData> &position_data);

  /// Set many dynamixels with new position values in one instruction. @see syncWrite.
  /// @param velocity_data  map of ids to syncdata objects containing velocity data
  /// @return True on comm success, false otherwise.
  bool setMultiVelocity(std::unordered_map<int, SyncData> &velocity_data);

  /// Set many dynamixels with new profile velocity values in one instruction. @see syncWrite.
  /// @param value_pairs  A vector of tuples, each tuple is a value pair containing a dynamixel ID and a profile
  /// velocity value.
  /// @return True on comm success, false otherwise.
  bool setMultiProfileVelocity(std::unordered_map<int, SyncData> &velocity_data);

  /// Set many dynamixels with new torque values in one instruction. @see syncWrite.
  /// @param torque_data map of ids to syncdata objects containing torque data
  /// @return True on comm success, false otherwise.
  bool setMultiTorque(std::unordered_map<int, SyncData> &torque_data);

private:

  /// Performs the bulk read for each protocol. A bulk read is a broadcast instruction on a bus that commands a list
  /// of dynamixels to respond in order with a read of a specified address and length (which can be different for each
  /// dynamixel). This protocol can be used to read many parameters from many dynamixels on a bus in just one
  /// instruction.
  /// @param read_data Pointer to a map of SyncData objects, containing ids and vectors to read into
  /// @param address The address value in the control table the dynamixels will start reading from
  /// @param length The number of bytes to read consecutively from the control table
  /// @returns true if at least one dynamixel was successfully read
  bool bulkRead(std::unordered_map<int, SyncData*> &read_data, uint16_t address, uint16_t length);

  /// Performs the sync read for each protocol. A sync read is a broadcast instruction on a bus that commands a list
  /// of dynamixels to respond in order with a read of a specified address and length. This protocol can be used to read
  /// many parameters from many dynamixels on a bus in just one instruction.
  /// @param read_data Pointer to a map of SyncData objects, containing ids and vectors to read into
  /// @param address The address value in the control table the dynamixels will start reading from
  /// @param length The number of bytes to read consecutively from the control table
  /// @returns true if at least one dynamixel was successfully read
  bool syncRead(std::unordered_map<int, SyncData*> &read_data, uint16_t address, uint16_t length);

  /// Performs the sync write for each protocol. A sync write is a broadcast instruction on a bus that commands a list
  /// of dynamixels to write a value into a specified address (the value written can be different for each dynamixel
  /// but the address is universal). This can be used to update a parameter (say goal position) for many dynamixels,
  /// each with a unique value, all in one instruction.
  /// @param write_data Pointer to a map of SyncData objects, containing ids and vectors to write from.
  /// @param address The address value in the control table the dynamixels will write to
  /// @param length The number of bytes to write
  /// @returns true on successful write, false otherwise
  bool syncWrite(std::unordered_map<int, SyncData> &write_data, uint16_t address, uint16_t length);

private:

  dynamixel::PortHandler *portHandler_; /// The port handler object. The dynamixel sdk serial object.
  dynamixel::PacketHandler *packetHandler_; /// packet handler. Provides raw response deconstruction.

  bool use_legacy_protocol_; /// if we are using legacy 1.0 protocol or newer 2.0 protocol (depends on model support)

  bool use_group_read_; /// using monolothic bulkRead or syncRead for bulk data exchange
  bool use_group_write_; /// using monolothic syncWrite for bulk data exchange

  uint8_t single_read_fallback_counter_; /// indicates group comm failure fallback interval

  std::unordered_map<uint16_t, const DynamixelSpec> model_specs_; /// map of model numbers to motor specifications

  std::unordered_map<int, std::vector<uint8_t>> raw_read_map_; /// map to store raw reads into
};

}

#endif //DYNAMIXEL_INTERFACE_DRIVER_H_
