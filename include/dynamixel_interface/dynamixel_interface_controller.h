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
 * @file   dynamixel_interface_controller.h
 * @author Tom Molnar (Tom.Molnar@data61.csiro.au), Brian Axelrod
 * @date   January, 2017
 * @brief  Defines the dynamixel controller class and the types used therein
 */

#ifndef DYNAMIXEL_INTERFACE_CONTROLLER_H_
#define DYNAMIXEL_INTERFACE_CONTROLLER_H_

#include <algorithm>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <XmlRpcValue.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/JointState.h>

#include <dynamixel_interface/dynamixel_interface_driver.h>

#include <dynamixel_interface/DataPort.h>
#include <dynamixel_interface/DataPorts.h>
#include <dynamixel_interface/ServoDiag.h>
#include <dynamixel_interface/ServoDiags.h>

namespace dynamixel_interface
{
/// Struct that describes each servo's place in the system including which joint it corresponds to.
struct DynamixelInfo
{
  int id;                  ///< The unique (per port) ID of the motor
  std::string joint_name;  ///< The unique (globally) name of the joint

  double max_vel;       ///< Motor maximum joint velocity (rad/s)
  double torque_limit;  ///< Motor maximum torque limit (%rated max)

  int zero_pos;  ///< Motor initial position (in raw encoder values). This value defines the 0 radian position
  int min_pos;   ///< Motor minimum encoder value. Note that if min > max, the motor direction is reversed
  int max_pos;   ///< Motor maximum encoder value. Note that if min > max, the motor direction is reversed

  const DynamixelSpec *model_spec;  ///< Motor model specification including encoder counts and unit conversion factors

  bool torque_enabled;                ///< Motor enable flag
  DynamixelControlMode current_mode;  ///< control mode (position, velocity, torque)
  uint8_t hardware_status;            ///< current motor status, used for hardware error reporting
};

/// Struct which stores information about each port in use and which joints use that port
struct PortInfo
{
  std::string port_name;  ///< User defined port name

  std::string device;  ///< Serial device name
  int baudrate;        ///< Serial baud rate

  bool use_legacy_protocol;  ///< boolean indicating if legacy protocol (for older series dynamixels) is in use

  std::unique_ptr<DynamixelInterfaceDriver> driver;  ///< The driver object

  std::unordered_map<std::string, DynamixelInfo> joints;  ///< map of joint information by name
};

/// This class forms a ROS Node that provides an interface with the dynamixel series of servos.
/// The controller functions on a timer based callback for updating the motor states. Commands are
/// Continuously sent to the motor and updated via callback once new messages are published to the command
/// Topic. This class also provides a threaded interface for controlling multiple sets of dynamixels simultaneously
/// and synchronously through different serial ports. This allows robots with many motors to reduce the overall IO
/// time required for control.
class DynamixelInterfaceController
{
public:
  /// Constructor, loads the motor configuration information from the specified yaml file and intialises
  /// The motor states.
  DynamixelInterfaceController();

  /// Destructor, deletes the objects holding the serial ports and disables the motors if required
  ~DynamixelInterfaceController();

  /// Parses param information from the rosparam server
  /// @returns true if all params parsed successfully, false otherwise
  bool parseParameters(void);

  /// Initialises the controller, performing the following steps:
  /// - for each port:
  ///   - Initialise driver
  ///   - for each dynamixel
  //      - check response on bus
  ///     - load model information
  ///     - validate compatibility with port
  /// @returns true on successful initialisation, false otherwise
  bool initialise();

  /// Gets the target loop rate for the controller
  /// @returns the target loop rate for the controller (Hz)
  inline double getLoopRate(void) { return loop_rate_; };

  /// main loop for performing IO, handles the creation and joining of IO threads for each port, so that IO for multiple
  /// usb devices can be threaded.
  void loop(void);

private:
  /// Parses the information in the yaml file for each port
  /// @param[in] ports the xml structure to be parsed
  /// @returns true if all params parsed false otherwise
  bool parsePortInformation(XmlRpc::XmlRpcValue ports);

  /// Parses the information in the yaml file for each servo
  /// @param[out] port the port object to parse the servo info into
  /// @param[out] servos the xml structure to be parsed
  /// @returns true if all params parsed, false otherwise
  bool parseServoInformation(PortInfo &port, XmlRpc::XmlRpcValue servos);

  /// Initialises the port, opening the driver, and validating all dynamixels
  /// @param[in] port the port object to initialise
  /// @returns true on successful initialisation, false otherwise
  bool initialisePort(PortInfo &port);

  /// Initialises the dynamixel. Pings the given id to make sure it exists, then loads it's model information and sets
  /// up the relevant registers
  /// @param[in] port the port for this dynamixel
  /// @param[in] dynamixel the dynamixel to intialise
  /// @returns true on successful initialisation, false otherwise
  bool initialiseDynamixel(PortInfo &port, DynamixelInfo &dynamixel);

  /// Callback for recieving a command from the /desired_joint_state topic. The function atomically updates the class
  /// member variable containing the latest message and sets a flag indicating a new message has been received
  /// @param[in] joint_commands the command received from the topic
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_commands);

  /// Top level control function for each port's IO thread.
  /// @param[in] port the port for this thread to perform IO on
  /// @param[out] read_msg the JointState this threads joint data is read into
  /// @param[out] dataport_msg the DataPorts msg this thread reads dataport data into
  /// @param[out] status_msgs the ServoDiags msg this thread reads diagnostic data into
  /// @param[in] perform_write whether we are writing data this iteration
  void multiThreadedIO(PortInfo &port, sensor_msgs::JointState &read_msg, dynamixel_interface::DataPorts &dataport_msg,
                       dynamixel_interface::ServoDiags &status_msg, bool perform_write) const;

  /// Function called in each thread to perform a write on a port
  /// @param[in] port_num index used to retrieve port information from port list
  /// @param[in] joint_commands message cointaining the commands for each joint
  void multiThreadedWrite(PortInfo &port, sensor_msgs::JointState joint_commands) const;

  /// Function in thread to perform read on a port.
  /// @param[in] port the port for this thread to perform IO on
  /// @param[out] read_msg the JointState this threads joint data is read into
  /// @param[out] dataport_msg the DataPorts msg this thread reads dataport data into
  /// @param[out] status_msgs the ServoDiags msg this thread reads diagnostic data into
  void multiThreadedRead(PortInfo &port, sensor_msgs::JointState &read_msg,
                         dynamixel_interface::DataPorts &dataports_msg,
                         dynamixel_interface::ServoDiags &diags_msg) const;

  std::unique_ptr<ros::NodeHandle> nh_;  ///< Handler for the ROS Node

  std::vector<PortInfo> dynamixel_ports_;  ///< method of control (position/velocity/torque)

  sensor_msgs::JointState write_msg_;  ///< Stores the last message received from the write command topic

  std::mutex write_mutex_;    ///< Mutex for write_msg, as there are potentially multiple threads
  bool write_ready_ = false;  ///< Booleans indicating if we have received commands

  ros::Subscriber joint_state_subscriber_;  ///< Gets joint states for writes

  ros::Publisher joint_state_publisher_;  ///< Publishes joint states from reads
  ros::Publisher diagnostics_publisher_;  ///< Publishes joint states from reads
  ros::Publisher dataport_publisher_;     ///< Publishes the data from the external ports on dynamixel_pros
  ros::Publisher debug_publisher_;        ///< Debug message publisher

  ros::Timer broadcast_timer_;  ///< Timer that controls the rate of the IO callback

  // variables for tracking diagnostics reading rate
  bool read_diagnostics_;         ///< Bool for telling threads to read diagnostics data
  uint diagnostics_counter_ = 0;  ///< Counter for tracking diagnostics rate
  uint diagnostics_iters_ = 0;    ///< publish when diagnostic_counter_ == this

  // variables for tracking dataport reading rate
  bool read_dataport_;         ///< Bool for telling threads to read dataport data
  uint dataport_counter_ = 0;  ///< counter for tracking dataport rate
  uint dataport_iters_ = 0;    ///< publish when dataport_counter_ == this

  DynamixelControlMode control_type_;  /// method of control (position/velocity/torque)

  bool parameters_parsed_ = false;  ///< Bool indicating if we have parsed parameters
  bool initialised_ = false;        ///< Bool indicating if we are ready for operation

  bool stop_motors_on_shutdown_;  ///< Indicates if the motors should be turned off when the controller stops
  bool ignore_input_velocity_;    ///< can set driver to ignore profile velocity commands in position mode

  double global_max_vel_;       ///< global joint speed limit
  double global_torque_limit_;  ///< global joint torque limit

  double loop_rate_;         ///< Desired loop rate (joint states are published at this rate)
  double diagnostics_rate_;  ///< Desired rate at which servo diagnostic information is published
  double dataport_rate_;     ///< Rate at which the pro external dataport is read
};

}  // namespace dynamixel_interface

#endif  // DYNAMIXEL_INTERFACE_CONTROLLER_H_
