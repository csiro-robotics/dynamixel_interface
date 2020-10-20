/* CSIRO Open Source Software License Agreement (variation of the BSD / MIT License)
 * Copyright (c) 2020, Commonwealth Scientific and Industrial Research Organisation (CSIRO) ABN 41 687 119 230.
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
 * dynamixel_interface is forked from projects authored by Brian
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
#include <string>
#include <unordered_map>
#include <vector>

#include <dynamixel_interface/dynamixel_const.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <ros/ros.h>

namespace dynamixel_interface
{
/// Struct that describes the dynamixel motor's static and physical properties
struct DynamixelSpec
{
  std::string name;              ///< The Model Name
  uint16_t model_number;         ///< Model number (e.g 29 = MX-28)
  DynamixelSeriesType type;      ///< Model type (e.g MX, AX, Pro)
  bool external_ports;           ///< If this model has data ports
  int encoder_cpr;               ///< Motor encoder counts per revolution
  double encoder_range_deg;      ///< Motor encoder range in degrees
  double velocity_radps_to_reg;  ///< Conversion factor from velocity in radians/sec to register counts
  double effort_reg_max;         ///< Max possible value for effort register
  double effort_reg_to_mA;       ///< Conversion factor from register values to current in mA
};


/// Basic struct for representing dynamixel data exchange
struct SyncData
{
  int id;                     ///< id of dynamixel
  DynamixelSeriesType type;   ///< type of dynamixel
  bool success;               ///< bool indicating comms success
  std::vector<uint8_t> data;  ///< IO data array
};

/// data struct used with getBulkState() to retrieve physical state
struct DynamixelState : SyncData
{
  int32_t position;  ///< position register value, units differ by series
  int32_t velocity;  ///< velocity register value, units differ by series
  int32_t effort;    ///< effort register value, units differ by series
};

/// data struct used with getBulkDiagnosticInfo() to retrieve diagnostics
struct DynamixelDiagnostic : SyncData
{
  int32_t voltage;      ///< voltage register value, usually units of 0.1V
  int32_t temperature;  ///< temperature register value, usually units of 0.1C
};

/// data struct used with getBulkDataportInfo() to retrieve dataport values
struct DynamixelDataport : SyncData
{
  std::array<uint16_t, 4> port_values;  ///< values from dataports, units differ by series and dataport setting
};

/// Provides the handling of the low level communications between the
/// dynamixels and the controller
class DynamixelInterfaceDriver
{
public:
  /// Constructor.
  /// @param[in] device The serial port to connect to
  /// @param[in] baud The baud rate to use
  /// @param[optional] use_legacy_protocol Whether to use legacy 1.0 protocol (default false)
  /// @param[optional] use_group_read Whether to use bulk protocol methods with bulk getters (default true)
  /// @param[optional] use_group_read Whether to use bulk protocol methods with bulk setters (default true)
  DynamixelInterfaceDriver(const std::string &device = "/dev/ttyUSB0", int baud = 1000000,
                           bool use_legacy_protocol = false, bool use_group_read = true, bool use_group_write = true);

  /// Destructor. Closes and releases serial port.
  ~DynamixelInterfaceDriver();

  /// Parses Motor Data, Opens port and sets up driver
  /// @returns true if initialisation successful, false otherwise
  bool initialise(void);

  /// Ping the specified id, used to check if a dynamixel with that ID is on the bus
  /// @param[in] servo_id The ID to ping on the bus.
  /// @return True if a dynamixel responds, false otherwise.
  bool ping(int servo_id) const;

  /// Get pointer to model spec data for given model number
  inline const DynamixelSpec *getModelSpec(uint model_number) const
  {
    auto it = model_specs_.find(model_number);
    if (it != model_specs_.end())
    {
      return &(it->second);
    }
    else
    {
      return nullptr;
    }
  };

  // ************************************ GETTERS ***************************************** //

  /// Retrieves the model number from the dynamixel's eeprom
  /// @param[in] servo_id The ID of the servo to retrieve from
  /// @param[out] model_number Stores the model_number returned
  /// @return True on comm success, false otherwise.
  bool getModelNumber(int servo_id, uint16_t *model_number) const;

  /// Retrieves the hardware status error value from the dynamixel's eeprom
  /// @param[in] servo_id The ID of the servo to retrieve from
  /// @param[in] type the type of the servo to read from
  /// @param[out] error Stores the returned error code
  /// @return True on comm success, false otherwise.
  bool getErrorStatus(int servo_id, DynamixelSeriesType type, uint8_t *error) const;

  /// Retrieves the maximum torque limit from the dynamixel's eeprom.
  /// @param[in] servo_id The ID of the servo to retrieve from
  /// @param[in] type the type of the servo to read from
  /// @param[out] max_torque Stores the value returned
  /// @return True on comm success, false otherwise.
  bool getMaxTorque(int servo_id, DynamixelSeriesType type, uint16_t *max_torque) const;

  /// Retrieves the torque enabled value from the dynamixel's ram.
  /// @param[in] servo_id The ID of the servo to retrieve from
  /// @param[in] type the type of the servo to read from
  /// @param[out] torque_enabled Stores the status of torque enable
  /// @return True on comm success, false otherwise.
  bool getTorqueEnabled(int servo_id, DynamixelSeriesType type, bool *torque_enabled) const;

  /// Retrieves the current target_velocity from the dynamixel's ram.
  /// @param[in] servo_id The ID of the servo to retrieve from
  /// @param[in] type the type of the servo to read from
  /// @param[out] target_velocity Stores the value returned
  /// @return True on comm success, false otherwise.
  bool getTargetTorque(int servo_id, DynamixelSeriesType type, int16_t *target_torque) const;

  /// Retrieves arbitrary register readings from the Dynamixel, can be used in cases where provided getters are
  /// insufficient or to read multiple contiguous values at once.
  /// @param[in] servo_id The ID of the servo to retrieve from
  /// @param[in] address The address value in the control table the dynamixel will start reading from
  /// @param[in] length The number of bytes to read consecutively from the control table
  /// @param[out] response Array to store the raw dynamixel response.
  bool readRegisters(int servo_id, uint16_t address, uint16_t length, std::vector<uint8_t> *response) const;

  // *********************** BULK_READ METHODS *************************** //

  /// Bulk Reads the Present Position, Present Velocity and Present Current in one instruction. If the group read fails
  /// the function will fall back on reading each motor individually. Optionally, the group comms can be disabled on
  /// initialisation of the driver (by setting use_group_read to false) in which case the function will always read
  /// from each dynamixel individually.
  /// @param[in] state_map map of servo ids to state data to read into
  /// @return True on comm success, false otherwise
  bool getBulkState(std::unordered_map<int, DynamixelState> &state_map) const;

  /// Bulk Reads the voltage and temperature in one instruction, behaves the same as getBulkState()
  /// @param[in] data_map map of servo ids to dataport objects to read into
  /// @return True on comm success, false otherwise
  bool getBulkDataportInfo(std::unordered_map<int, DynamixelDataport> &data_map) const;

  /// Bulk Reads the voltage and temperature in one instruction, behaves the same as getBulkState()
  /// @param[in] diag_map map of servo ids to diagnostics objects to read into
  /// @return True on comm success, false otherwise
  bool getBulkDiagnosticInfo(std::unordered_map<int, DynamixelDiagnostic> &diag_map) const;

  // **************************** SETTERS ******************************** //

  /// Sets the Operating mode of the Dynamixel. The three possible control modes are: Position, Velocity or Torque.
  /// The way these modes are enabled differs for each series of dynamixel. XM and Pro have an operating mode register
  /// that can be used to changed the control type. For AX, RX and MX series, the method of control is defined by the
  /// values in the angle limit registers and the torque_control_enable register. Note torque control is not available
  /// on the MX-28 models.
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] operating_mode The method of control
  /// @return True on comm success and valid operating mode, false otherwise.
  bool setOperatingMode(int servo_id, DynamixelSeriesType type, DynamixelControlMode operating_mode) const;

  /// Sets the minimum and maximum angle limits for the dynamixel
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] min_angle the minimum angle limit (in encoder values)
  /// @param[in] max_angle the maximum angle limit (in encoder values)
  /// @return True on comm success, false otherwise.
  bool setAngleLimits(int servo_id, DynamixelSeriesType type, int32_t min_angle, int32_t max_angle) const;

  /// Sets the minimum angle limit for the dynamixel
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] angle The minimum angle limit (in encoder values)
  /// @return True on comm success, false otherwise.
  bool setMinAngleLimit(int servo_id, DynamixelSeriesType type, int32_t angle) const;

  /// Sets the maximum angle limit for the dynamixel
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] angle The maximum angle limit (in encoder values)
  /// @return True on comm success, false otherwise.
  bool setMaxAngleLimit(int servo_id, DynamixelSeriesType type, int32_t angle) const;

  /// Sets the maximum torque limit for the dynamixel
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] max_torque the maximum torque limit
  /// @return True on comm success, false otherwise.
  bool setMaxTorque(int servo_id, DynamixelSeriesType type, uint16_t max_torque) const;

  /// Sets the maximum velocity limit for the dynamixel
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] max_vel the maximum velocity limit
  /// @return True on comm success, false otherwise.
  bool setMaxVelocity(int servo_id, DynamixelSeriesType type, uint32_t max_vel) const;

  /// Sets the torque enable register of the dynamixel. This value defines the on/off state of the servo.
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] on The state of the servo (true = on, false = off).
  /// @return True on comm success, false otherwise.
  bool setTorqueEnabled(int servo_id, DynamixelSeriesType type, bool on) const;

  /// Sets the torque control enable register of the dynamixel mx series. can be used to dynamically
  /// switch between position and torque control modes.
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] on The torque control state of the servo (true = on, false = off).
  /// @return True on comm success, false otherwise.
  bool setTorqueControlEnabled(int servo_id, DynamixelSeriesType type, bool on) const;

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
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] p_gain The proportional gain value to write
  /// @param[in] i_gain The integral gain value to write
  /// @param[in] d_gain The derivative gain value to write
  /// @return True on comm success, false otherwise.
  bool setPositionPIDGains(int servo_id, DynamixelSeriesType type, double p_gain, double i_gain, double d_gain) const;

  /// Sets the proportional gain value for the position control mode if available. @see setPIDGains
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] gain The proportional gain value to write
  /// @return True on comm success, false otherwise.
  bool setPositionProportionalGain(int servo_id, DynamixelSeriesType type, uint16_t gain) const;

  /// Sets the integral gain value for the position control mode if available. @see setPIDGains
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] gain The integral gain value to write
  /// @return True on comm success, false otherwise.
  bool setPositionIntegralGain(int servo_id, DynamixelSeriesType type, uint16_t gain) const;

  /// Sets the derivative gain value for the position control mode if available. @see setPIDGains
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] gain The derivative gain value to write
  /// @return True on comm success, false otherwise.
  bool setPositionDerivativeGain(int servo_id, DynamixelSeriesType type, uint16_t gain) const;

  /// Sets the velocity PID values for the dynamixels
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] p_gain The proportional gain value to write
  /// @param[in] i_gain The integral gain value to write
  /// @param[in] d_gain The derivative gain value to write
  /// @return True on comm success, false otherwise.
  bool setVelocityPIDGains(int servo_id, DynamixelSeriesType type, double p_gain, double i_gain) const;

  /// Sets the proportional gain value for the velocity control mode if available. @see setPIDGains
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] gain The proportional gain value to write
  /// @return True on comm success, false otherwise.
  bool setVelocityProportionalGain(int servo_id, DynamixelSeriesType type, uint16_t gain) const;

  /// Sets the integral gain value for the velocity control mode if available. @see setPIDGains
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] gain The integral gain value to write
  /// @return True on comm success, false otherwise.
  bool setVelocityIntegralGain(int servo_id, DynamixelSeriesType type, uint16_t gain) const;

  /// Sets the profile velocity of the dynamixel. Profile velocity is how fast the servo should move between positions.
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] velocity The profile velocity value to write
  /// @return True on comm success, false otherwise.
  bool setProfileVelocity(int servo_id, DynamixelSeriesType type, int32_t velocity) const;

  /// Sets the torque value of the dynamixel.
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] type the type of the servo to read from
  /// @param[in] torque The torque value to write
  /// @return True on comm success, false otherwise.
  bool setTorque(int servo_id, DynamixelSeriesType type, int16_t torque) const;

  /// Writes arbitrary register values to the Dynamixel, can be used in cases where provided setters are insufficient
  /// or to write multiple contiguous values at once.
  /// @param[in] servo_id The ID of the servo to write to
  /// @param[in] address The address value in the control table the dynamixel will start writing to
  /// @param[in] length The number of bytes to write consecutively to the control table
  /// @param[in] data Array containing the value to be written.
  /// @return True on comm success, false otherwise.
  bool writeRegisters(int servo_id, uint16_t address, uint16_t length, uint8_t *data) const;

  // *********************** SYNC_WRITE METHODS *************************** //

  /// Set many dynamixels with new position values in one instruction. @see syncWrite.
  /// @param[in] position_data map of ids to syncdata objects containing position data
  /// @return True on comm success, false otherwise.
  bool setMultiPosition(std::unordered_map<int, SyncData> &position_data) const;

  /// Set many dynamixels with new position values in one instruction. @see syncWrite.
  /// @param[in] velocity_data map of ids to syncdata objects containing velocity data
  /// @return True on comm success, false otherwise.
  bool setMultiVelocity(std::unordered_map<int, SyncData> &velocity_data) const;

  /// Set many dynamixels with new profile velocity values in one instruction. @see syncWrite.
  /// @param[in] velocity_data map of ids to syncdata objects containing profile velocity data
  /// @return True on comm success, false otherwise.
  bool setMultiProfileVelocity(std::unordered_map<int, SyncData> &velocity_data) const;

  /// Set many dynamixels with new torque values in one instruction. @see syncWrite.
  /// @param[in] torque_data map of ids to syncdata objects containing torque data
  /// @return True on comm success, false otherwise.
  bool setMultiTorque(std::unordered_map<int, SyncData> &torque_data) const;

  /// Returns a string version of the dynamixel series based on the input type
  /// @param[in] type the type of the servo to read from
  /// @returns The string name of the Dynamixel series
  inline std::string getSeriesName(DynamixelSeriesType type) const
  {
    std::string series_name = "undefined";
    switch (type)
    {
      case kSeriesAX:
        series_name = "AX";
        break;
      case kSeriesRX:
        series_name = "RX";
        break;
      case kSeriesDX:
        series_name = "DX";
        break;
      case kSeriesEX:
        series_name = "EX";
        break;
      case kSeriesLegacyMX:
        series_name = "Legacy MX";
        break;
      case kSeriesMX:
        series_name = "MX";
        break;
      case kSeriesX:
        series_name = "X";
        break;
      case kSeriesP:
        series_name = "P";
        break;
      case kSeriesLegacyPro:
        series_name = "Legacy Pro";
        break;
    }
    return series_name;
  };

private:
  /// Loads in the motor specifications from the motor_data.yaml file
  /// @returns true if load successful, false otherwise
  bool loadMotorData(void);

  /// Performs the bulk read for each protocol. A bulk read is a broadcast instruction on a bus that commands a list
  /// of dynamixels to respond in order with a read of a specified address and length (which can be different for each
  /// dynamixel). This protocol can be used to read many parameters from many dynamixels on a bus in just one
  /// instruction.
  /// @param[in] read_data Pointer to a map of SyncData objects, containing ids and vectors to read into
  /// @param[in] address The address value in the control table the dynamixels will start reading from
  /// @param[in] length The number of bytes to read consecutively from the control table
  /// @returns true if at least one dynamixel was successfully read
  bool bulkRead(std::unordered_map<int, SyncData*> &read_data, uint16_t address, uint16_t length) const;

  /// Performs the sync read for each protocol. A sync read is a broadcast instruction on a bus that commands a list
  /// of dynamixels to respond in order with a read of a specified address and length. This protocol can be used to read
  /// many parameters from many dynamixels on a bus in just one instruction.
  /// @param[in] read_data Pointer to a map of SyncData objects, containing ids and vectors to read into
  /// @param[in] address The address value in the control table the dynamixels will start reading from
  /// @param[in] length The number of bytes to read consecutively from the control table
  /// @returns true if at least one dynamixel was successfully read
  bool syncRead(std::unordered_map<int, SyncData*> &read_data, uint16_t address, uint16_t length) const;

  /// Performs the sync write for each protocol. A sync write is a broadcast instruction on a bus that commands a list
  /// of dynamixels to write a value into a specified address (the value written can be different for each dynamixel
  /// but the address is universal). This can be used to update a parameter (say goal position) for many dynamixels,
  /// each with a unique value, all in one instruction.
  /// @param[in] write_data Pointer to a map of SyncData objects, containing ids and vectors to write from.
  /// @param[in] address The address value in the control table the dynamixels will write to
  /// @param[in] length The number of bytes to write
  /// @returns true on successful write, false otherwise
  bool syncWrite(std::unordered_map<int, SyncData> &write_data, uint16_t address, uint16_t length) const;

private:
  std::unique_ptr<dynamixel::PortHandler> portHandler_;  ///< The port handler object. The dynamixel sdk serial object.
  std::unique_ptr<dynamixel::PacketHandler> packetHandler_;  ///< packet handler. Provides raw response deconstruction.

  std::string device_;  ///< name of device to open
  int baud_rate_;       ///< baud rate

  bool use_legacy_protocol_;  ///< if we are using legacy 1.0 protocol or newer 2.0 protocol (depends on model support)
  bool use_group_read_;       ///< using monolothic bulkRead or syncRead for bulk data exchange
  bool use_group_write_;      ///< using monolothic syncWrite for bulk data exchange

  bool initialised_ = false;              ///< Variable to indicate if we're ready for comms
  uint8_t single_read_fallback_counter_;  ///< indicates group comm failure fallback interval

  std::unordered_map<uint16_t, const DynamixelSpec> model_specs_;  ///< map of model numbers to motor specifications
  std::unordered_map<int, std::vector<uint8_t>> raw_read_map_;     ///< map to store raw reads into
};

}  // namespace dynamixel_interface

#endif  // DYNAMIXEL_INTERFACE_DRIVER_H_
