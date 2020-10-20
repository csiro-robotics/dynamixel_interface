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
 * @file   dynamixel_interface_controller.cpp
 * @author Tom Molnar (Tom.Molnar@data61.csiro.au), Brian Axelrod
 * @date   January, 2017
 * @brief  Implements the dynamixel controller class and defines the main loop of operation
 */

#include <stdlib.h>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#define _USE_MATH_DEFINES
#include <math.h>

#include <dynamixel_interface/dynamixel_interface_controller.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/spinner.h>
#include <ros/transport_hints.h>


namespace dynamixel_interface
{
/// Constructor, loads the motor configuration information from the specified yaml file and intialises
/// The motor states.
DynamixelInterfaceController::DynamixelInterfaceController()
{
  nh_ = std::make_unique<ros::NodeHandle>();
}


/// Destructor, deletes the objects holding the serial ports and disables the motors if required
DynamixelInterfaceController::~DynamixelInterfaceController()
{
  ROS_INFO("shutting_down dynamixel_interface_controller");

  if (stop_motors_on_shutdown_)
  {
    for (int i = 0; i < dynamixel_ports_.size(); i++)
    {
      // Turn off all the motors
      for (auto &it : dynamixel_ports_[i].joints)
      {
        dynamixel_ports_[i].driver->setTorqueEnabled(it.second.id, it.second.model_spec->type, 0);
        ROS_INFO("Torque disabled on %s joint\n", it.first.c_str());
      }
    }
  }
}

/// Parses param information from the rosparam server
/// @returns true if all params parsed successfully, false otherwise
bool DynamixelInterfaceController::parseParameters(void)
{
  // Stores config variables only used in init function
  std::string mode;
  std::string node_name = ros::this_node::getName();

  // load all the info from the param server, with defaults
  ros::param::param<double>("~loop_rate", loop_rate_, 50.0);
  ros::param::param<bool>("~disable_torque_on_shutdown", stop_motors_on_shutdown_, false);


  ros::param::param<bool>("~ignore_input_velocity", ignore_input_velocity_, false);

  ros::param::param<double>("~dataport_rate", dataport_rate_, 0.0);
  ros::param::param<double>("~diagnostics_rate", diagnostics_rate_, 0.0);

  ros::param::param<std::string>("~control_mode", mode, "Position");

  ros::param::param<double>("~global_max_vel", global_max_vel_, 5.0);
  ros::param::param<double>("~global_torque_limit", global_torque_limit_, 1.0);


  // clamp read rates to sensible values
  if (loop_rate_ <= 0)
  {
    ROS_ERROR("Loop rate must be > 0!");
    return false;
    ;
  }

  if (diagnostics_rate_ < 0)
  {
    diagnostics_rate_ = 0;
  }
  else if (diagnostics_rate_ > loop_rate_)
  {
    ROS_WARN("Clamping diagnostics rate to loop rate.");
    diagnostics_rate_ = loop_rate_;
  }
  else
  {
    diagnostics_iters_ = std::round(loop_rate_ / diagnostics_rate_);
    diagnostics_counter_ = 0;
  }

  // num iters to read from
  if (dataport_rate_ < 0)
  {
    dataport_rate_ = 0;
  }
  else if (dataport_rate_ > loop_rate_)
  {
    ROS_WARN("Clamping dataport rate to loop rate.");
    dataport_rate_ = loop_rate_;
  }
  else
  {
    dataport_iters_ = std::round(loop_rate_ / dataport_rate_);
    dataport_counter_ = 0;
  }


  // Set control mode for this run
  if (mode == "position")
  {
    control_type_ = kModePositionControl;
  }
  else if (mode == "velocity")
  {
    control_type_ = kModeVelocityControl;
  }
  else if (mode == "effort")
  {
    control_type_ = kModeTorqueControl;
  }
  else
  {
    ROS_ERROR("Control Mode Not Supported!");
    return false;
  }

  // Attempt to parse information for each device (port)
  if (ros::param::has("~ports"))
  {
    // PARSE ARRAY OF PORT INFORMATION
    XmlRpc::XmlRpcValue ports;
    ros::param::get("~ports", ports);
    parsePortInformation(ports);

    // shutdown if no valid ports
    if (dynamixel_ports_.size() == 0)
    {
      ROS_ERROR("No valid ports found, shutting_down...");
      return false;
    }
  }
  else
  {
    ROS_ERROR("No port details loaded to param server");
    return false;
  }

  if (diagnostics_rate_ > 0)
  {
    diagnostics_publisher_ = nh_->advertise<dynamixel_interface::ServoDiags>("servo_diagnostics", 1);
  }

  if (dataport_rate_ > 0)
  {
    dataport_publisher_ = nh_->advertise<dynamixel_interface::DataPorts>("external_dataports", 1);
  }

  // advertise the joint state input and output topics
  joint_state_publisher_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 1);

  joint_state_subscriber_ = nh_->subscribe<sensor_msgs::JointState>("desired_joint_states", 1,
                                                                    &DynamixelInterfaceController::jointStateCallback,
                                                                    this, ros::TransportHints().tcpNoDelay());
  parameters_parsed_ = true;
  return true;
}


/// Parses the information in the yaml file for each port
/// @param[in] ports the xml structure to be parsed
bool DynamixelInterfaceController::parsePortInformation(XmlRpc::XmlRpcValue ports)
{
  // If there is no servos array in the param server, return
  if (ports.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Invalid/missing device information on the param server");
    return false;
  }

  // number of ports defined
  int num_ports = ports.size();

  // For every port, load and verify its information
  for (int i = 0; i < ports.size(); i++)
  {
    PortInfo port;

    bool use_group_read = false;
    bool use_group_write = false;

    /************************* PORT ARGUMENTS *********************/


    // check if info exists for specified port
    if (ports[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Invalid/Missing info-struct for servo index %d", i);
      return false;
    }


    // get port name
    if (ports[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Invalid/Missing port name for port %d", i);
      return false;
    }
    else
    {
      port.port_name = static_cast<std::string>(ports[i]["name"]);
    }


    // get device name
    if (ports[i]["device"].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Invalid/Missing device name for port %d", i);
      return false;
    }
    else
    {
      port.device = static_cast<std::string>(ports[i]["device"]);
    }

    // get baud rate
    if (ports[i]["baudrate"].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("Invalid/Missing baudrate for port %d", i);
      return false;
    }
    else
    {
      port.baudrate = static_cast<int>(ports[i]["baudrate"]);
    }

    // get protocol
    if (ports[i]["use_legacy_protocol"].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    {
      ROS_ERROR("Invalid/Missing use_legacy_protocol option for port %d", i);
      return false;
    }
    else
    {
      port.use_legacy_protocol = static_cast<bool>(ports[i]["use_legacy_protocol"]);
    }

    // get group read enabled
    if (ports[i]["group_read_enabled"].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    {
      ROS_ERROR("Invalid/Missing group_read_enabled option for port %d", i);
      return false;
    }
    else
    {
      use_group_read = static_cast<bool>(ports[i]["group_read_enabled"]);
    }

    // get group write enabled
    if (ports[i]["group_write_enabled"].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    {
      ROS_ERROR("Invalid/Missing group_write_enabled option for port %d", i);
      return false;
    }
    else
    {
      use_group_write = static_cast<bool>(ports[i]["group_write_enabled"]);
    }


    /************************* Driver initialisation *********************/

    // Create driver
    port.driver = std::unique_ptr<DynamixelInterfaceDriver>(new DynamixelInterfaceDriver(
      port.device, port.baudrate, port.use_legacy_protocol, use_group_read, use_group_write));

    /************************* Dynamixel initialisation *********************/

    XmlRpc::XmlRpcValue servos;

    // If there is no servos array in the param server, return
    if (ports[i]["servos"].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Invalid/missing servo information on the param server");
      return false;
    }
    else
    {
      servos = ports[i]["servos"];
    }

    parseServoInformation(port, servos);

    // add port only if dynamixels were found
    if (port.joints.size() > 0)
    {
      ROS_INFO("Adding port %s to loop", port.port_name.c_str());
      // add port information to server
      dynamixel_ports_.emplace_back(std::move(port));
    }
    else
    {
      ROS_ERROR("No dynamixels found on %s (%s)!", port.device.c_str(), port.port_name.c_str());
      return false;
    }
  }

  return true;
}

/// Parses the information in the yaml file for each servo
/// @param[in] port the port object to parse the servo info into
/// @param[in] servos the xml structure to be parsed
bool DynamixelInterfaceController::parseServoInformation(PortInfo &port, XmlRpc::XmlRpcValue servos)
{
  // number of servos defined
  int num_servos = servos.size();

  // For every servo, load and verify its information
  for (int i = 0; i < servos.size(); i++)
  {
    // store the loaded information in this struct
    DynamixelInfo info;

    // check if info exists for specified joint
    if (servos[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Invalid/Missing info-struct for servo index %d", i);
      return false;
    }

    // get joint id
    if (servos[i]["id"].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("Invalid/Missing id for servo index %d", i);
      return false;
    }
    else
    {
      // store the servo's ID
      info.id = static_cast<int>(servos[i]["id"]);
    }

    // get joint name
    if (servos[i]["joint_name"].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Invalid/Missing joint name for servo index %d, id: %d", i, info.id);
      return false;
    }
    else
    {
      // store the servo's corresponding joint name
      info.joint_name = static_cast<std::string>(servos[i]["joint_name"]);

      // check this port and all previous ports for duplicate joint names (not allowed as joints are
      // referred to by name)
      if (port.joints.find(info.joint_name) != port.joints.end())
      {
        ROS_ERROR("Cannot have multiple joints with the same name [%s]", info.joint_name.c_str());
        return false;
      }
      else
      {
        for (int j = 0; j < dynamixel_ports_.size(); j++)
        {
          if (dynamixel_ports_[j].joints.find(info.joint_name) != dynamixel_ports_[j].joints.end())
          {
            ROS_ERROR("Cannot have multiple joints with the same name [%s]", info.joint_name.c_str());
            return false;
          }
        }
      }
    }

    // get joint zero position
    if (servos[i]["zero_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("Invalid/Missing zero position for servo index %d, id: %d", i, info.id);
      return false;
    }
    else
    {
      // store the servo's corresponding joint
      info.zero_pos = static_cast<int>(servos[i]["zero_pos"]);
    }

    // get joint default min position
    if (servos[i]["min_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("Invalid/Missing min position for servo index %d, id: %d", i, info.id);
      return false;
    }
    else
    {
      info.min_pos = static_cast<int>(servos[i]["min_pos"]);
    }

    // get joint default max position
    if (servos[i]["max_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("Invalid/Missing max position for servo index %d, id: %d", i, info.id);
      return false;
    }
    else
    {
      info.max_pos = static_cast<int>(servos[i]["max_pos"]);
    }

    // get joint default joint speed (or set to global if none specified)
    if (servos[i]["max_vel"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      info.max_vel = global_max_vel_;
    }
    else
    {
      info.max_vel = static_cast<double>(servos[i]["max_vel"]);
      if (info.max_vel < 0.0)
      {
        info.max_vel = global_max_vel_;
      }
    }


    // get joint torque limit (or set to global if none specified)
    if (servos[i]["torque_limit"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      info.torque_limit = global_torque_limit_;
    }
    else
    {
      info.torque_limit = static_cast<double>(servos[i]["torque_limit"]);
      if ((info.torque_limit > 1.0) || (info.torque_limit < 0.0))
      {
        info.torque_limit = global_torque_limit_;
      }
    }

    // store current control mode
    info.current_mode = control_type_;

    // add joint to port
    port.joints[info.joint_name] = info;
  }

  return true;
}


/// Initialises the controller, performing the following steps:
/// - for each port:
///   - Initialise driver
///   - for each dynamixel
//      - check response on bus
///     - load model information
///     - validate compatibility with port
/// @returns true on successful initialisation, false otherwise
bool DynamixelInterfaceController::initialise()
{
  if (!parameters_parsed_)
  {
    ROS_ERROR("Parameters not parsed yet, must be parsed before init");
    return false;
  }

  for (auto &port : dynamixel_ports_)
  {
    if (!initialisePort(port))
    {
      ROS_ERROR("Unable to initialised port %s", port.port_name.c_str());
      return false;
    }
  }

  return true;
}


/// Initialises the port, opening the driver, and validating all dynamixels
/// @returns true on successful initialisation, false otherwise
bool DynamixelInterfaceController::initialisePort(PortInfo &port)
{
  DynamixelSeriesType type_check;
  bool first_dynamixel = true;

  // Attempt driver initialisation
  if (!port.driver->initialise())
  {
    ROS_ERROR("Unable to initialise driver");
    return false;
  }

  // first, initialise dynamixels
  for (auto &it : port.joints)
  {
    if (!initialiseDynamixel(port, it.second))
    {
      ROS_ERROR("Unable to initialised dynamixel %s", it.second.joint_name.c_str());
      return false;
    }
  }

  // then, check type safety of bus
  for (auto &it : port.joints)
  {
    if (first_dynamixel)
    {
      type_check = it.second.model_spec->type;
      first_dynamixel = false;
    }
    else
    {
      switch (type_check)
      {
        case kSeriesAX:
        case kSeriesRX:
        case kSeriesDX:
        case kSeriesEX:
        case kSeriesLegacyMX:
          if (it.second.model_spec->type > kSeriesLegacyMX)
          {
            ROS_ERROR("Type mismatch on bus, only dynamixel who share a common register table may share a bus! "
                      "Have both %s and %s.",
                      port.driver->getSeriesName(type_check).c_str(),
                      port.driver->getSeriesName(it.second.model_spec->type).c_str());
            return false;
          }
          break;

        case kSeriesX:
        case kSeriesMX:
          if ((it.second.model_spec->type != kSeriesX) && (it.second.model_spec->type != kSeriesMX))
          {
            ROS_ERROR("Type mismatch on bus, only dynamixel who share a common register table may share a bus! "
                      "Have both %s and %s.",
                      port.driver->getSeriesName(type_check).c_str(),
                      port.driver->getSeriesName(it.second.model_spec->type).c_str());
            return false;
          }
          break;

        case kSeriesP:
          if (it.second.model_spec->type != kSeriesP)
          {
            ROS_ERROR("Type mismatch on bus, only dynamixel who share a common register table may share a bus! "
                      "Have both %s and %s.",
                      port.driver->getSeriesName(type_check).c_str(),
                      port.driver->getSeriesName(it.second.model_spec->type).c_str());
            return false;
          }
          break;

        case kSeriesLegacyPro:
          if (it.second.model_spec->type != kSeriesLegacyPro)
          {
            ROS_ERROR("Type mismatch on bus, only dynamixel who share a common register table may share a bus! "
                      "Have both %s and %s.",
                      port.driver->getSeriesName(type_check).c_str(),
                      port.driver->getSeriesName(it.second.model_spec->type).c_str());
            return false;
          }
          break;
      }
    }
  }

  return true;
}

/// Initialises the dynamixel. Pings the given id to make sure it exists, then loads it's model information and sets
/// up the relevant registers
/// @returns true on successful initialisation, false otherwise
bool DynamixelInterfaceController::initialiseDynamixel(PortInfo &port, DynamixelInfo &dynamixel)
{
  // sleep to make sure the bus is clear of comms
  ros::Duration(0.2).sleep();

  // Ping the servo to make sure that we can actually connect to it
  // and that it is alive and well on our bus, if not, sleep and try again
  // if all retry's fail, throw an error
  bool ping_success = true;
  int ping_count = 0;
  while (!port.driver->ping(dynamixel.id))
  {
    // increment ping count
    ping_count++;

    ROS_WARN("Failed to ping id: %d, attempt %d, retrying...", dynamixel.id, ping_count);
    // max number of retry's
    if (ping_count > 5)
    {
      // unable to detect motor
      ROS_ERROR("Cannot ping dynamixel id: %d", dynamixel.id);
      return false;
    }

    // sleep and try again
    ros::Duration(0.5).sleep();
  }

  // only add if ping was successful
  bool success = true;
  uint16_t model_number = 0;
  bool t_e;

  if (!port.driver->getModelNumber(dynamixel.id, &model_number))
  {
    ROS_ERROR("Unable to retrieve model number for dynamixel id %d", dynamixel.id);
    return false;
  }
  else if (model_number == 0)
  {
    ROS_ERROR("Invalid model number retrieved for dynamixel id %d", dynamixel.id);
    return false;
  }

  // If valid motor, setup in operating mode
  dynamixel.model_spec = port.driver->getModelSpec(model_number);
  if (dynamixel.model_spec == nullptr)
  {
    ROS_ERROR("Failed to load model information for dynamixel id %d", dynamixel.id);
    ROS_ERROR("Model Number: %d", model_number);
    ROS_ERROR("Info is not in database");
    return false;
    ;
  }

  // check error status of dynamixel
  uint8_t error_status = 0;
  if (!port.driver->getErrorStatus(dynamixel.id, dynamixel.model_spec->type, &error_status))
  {
    ROS_WARN("Failed to check error status of dynamixel id %d", dynamixel.id);
  }
  else if (error_status)
  {
    ROS_WARN("Dynamixel %d returned error code %d", dynamixel.id, error_status);
  }

  dynamixel.torque_enabled = false;

  // Display joint info
  ROS_INFO("Joint Name: %s, ID: %d, Series: %s, Model: %s", dynamixel.joint_name.c_str(), dynamixel.id,
           port.driver->getSeriesName(dynamixel.model_spec->type).c_str(), dynamixel.model_spec->name.c_str());

  // Only support effort control on newer spec dynamixels
  if (control_type_ == kModeTorqueControl)
  {
    switch (dynamixel.model_spec->type)
    {
      case kSeriesAX:
      case kSeriesRX:
      case kSeriesDX:
      case kSeriesEX:
      case kSeriesLegacyMX:
      case kSeriesLegacyPro:
        ROS_ERROR("Effort Control not supported for legacy series dynamixels!");
        return false;
    }
  }

  // maintain torque state in motor
  if (!port.driver->getTorqueEnabled(dynamixel.id, dynamixel.model_spec->type, &t_e))
  {
    ROS_ERROR("Unable to get torque_enable status for dynamixel id %d", dynamixel.id);
    return false;
  }
  if (!port.driver->setTorqueEnabled(dynamixel.id, dynamixel.model_spec->type, 0))
  {
    ROS_ERROR("Unable to set torque_enable status for dynamixel id %d", dynamixel.id);
    return false;
  }

  // set operating mode for the motor
  if (!port.driver->setOperatingMode(dynamixel.id, dynamixel.model_spec->type, control_type_))
  {
    ROS_ERROR("Failed to set operating mode for dynamixel id %d", dynamixel.id);
    return false;
  }

  // set torque limit for the motor
  if (!port.driver->setMaxTorque(dynamixel.id, dynamixel.model_spec->type,
                                 (int)(dynamixel.torque_limit * dynamixel.model_spec->effort_reg_max)))
  {
    ROS_ERROR("Failed to set torque limit for dynamixel id %d", dynamixel.id);
    return false;
  }

  // set velocity limits for the motor
  if (dynamixel.model_spec->type > kSeriesLegacyMX)
  {
    if (!port.driver->setMaxVelocity(dynamixel.id, dynamixel.model_spec->type,
                                     (int)(dynamixel.max_vel * dynamixel.model_spec->velocity_radps_to_reg)))
    {
      ROS_ERROR("Failed to set velocity limit for dynamixel id %d", dynamixel.id);
      return false;
    }
  }

  // angle limits are only relevant in position control mode
  if (control_type_ == kModePositionControl)
  {
    // set angle limits & direction
    if (dynamixel.min_pos > dynamixel.max_pos)
    {
      if (!port.driver->setAngleLimits(dynamixel.id, dynamixel.model_spec->type, dynamixel.max_pos, dynamixel.min_pos))
      {
        ROS_ERROR("Failed to set angle limits for dynamixel id %d", dynamixel.id);
        return false;
      }
    }
    else
    {
      if (!port.driver->setAngleLimits(dynamixel.id, dynamixel.model_spec->type, dynamixel.min_pos, dynamixel.max_pos))
      {
        ROS_ERROR("Failed to set angle limits for dynamixel id %d", dynamixel.id);
        return false;
      }
    }
  }

  // preserve torque enable state
  if (!port.driver->setTorqueEnabled(dynamixel.id, dynamixel.model_spec->type, t_e))
  {
    ROS_ERROR("Unable to reset torque_enable status for dynamixel id %d", dynamixel.id);
    return false;
  }

  return true;
}

/// Callback for recieving a command from the /desired_joint_state topic. The function atomically updates the class
/// member variable containing the latest message and sets a flag indicating a new message has been received
/// @param[in] joint_commands the command received from the topic
void DynamixelInterfaceController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_commands)
{
  // thread safe modification
  std::unique_lock<std::mutex> lock(write_mutex_);

  // In order to prevent information being lost if multiple nodes are publishing commands, the joint command is checked
  // against the existing message, either adding new dynamixel to the message or updating the commands for the joints
  // already inside. This way joint command messages with subsets of dynamixels are combined rather than overwritten.
  if (write_msg_.name.empty())
  {
    write_msg_ = *joint_commands;
  }
  // avoid array length issues
  else if ((write_msg_.position.empty() == joint_commands->position.empty()) &&
           (write_msg_.velocity.empty() == joint_commands->velocity.empty()) &&
           (write_msg_.effort.empty() == joint_commands->effort.empty()))
  {
    // only add joint if not already in list (THIS TAKES N*M TIME, NEED TO OPTIMISE)
    for (int i = 0; i < joint_commands->name.size(); i++)
    {
      bool in_msg = false;

      // if joint already in message, update values
      for (int j = 0; j < write_msg_.name.size(); j++)
      {
        if (joint_commands->name[i] == write_msg_.name[j])
        {
          if (!write_msg_.position.empty() && !joint_commands->position.empty())
          {
            write_msg_.position[j] = joint_commands->position[i];
          }

          if (!write_msg_.velocity.empty() && !joint_commands->velocity.empty())
          {
            write_msg_.velocity[j] = joint_commands->velocity[i];
          }

          if (!write_msg_.effort.empty() && !joint_commands->effort.empty())
          {
            write_msg_.effort[j] = joint_commands->effort[i];
          }

          in_msg = true;
          break;
        }
      }

      // not already in message, push back
      if (!in_msg)
      {
        write_msg_.name.push_back(joint_commands->name[i]);

        if (!write_msg_.position.empty() && !joint_commands->position.empty())
        {
          write_msg_.position.push_back(joint_commands->position[i]);
        }

        if (!write_msg_.velocity.empty() && !joint_commands->velocity.empty())
        {
          write_msg_.velocity.push_back(joint_commands->velocity[i]);
        }

        if (!write_msg_.effort.empty() && !joint_commands->effort.empty())
        {
          write_msg_.effort.push_back(joint_commands->effort[i]);
        }
      }
    }
  }

  write_ready_ = true;
}

/// main loop for performing IO, handles the creation and joining of IO threads for each port, so that IO for multiple
/// usb devices can be threaded.
void DynamixelInterfaceController::loop(void)
{
  // wait until init
  if (!initialised_)
  {
    return;
  }

  // don't access the driver after its been cleaned up
  int num_servos = 0;
  std::vector<std::thread> threads;

  sensor_msgs::JointState read_msg;
  sensor_msgs::JointState reads[dynamixel_ports_.size()];

  dynamixel_interface::DataPorts dataports_msg;
  dynamixel_interface::DataPorts dataports_reads[dynamixel_ports_.size()];

  dynamixel_interface::ServoDiags diags_msg;
  dynamixel_interface::ServoDiags diags_reads[dynamixel_ports_.size()];

  std::unique_lock<std::mutex> lock(write_mutex_);

  // enable torque only once we start receiving commands
  if (write_ready_ && first_write_)
  {
    // loop over every port
    for (int i = 0; i < dynamixel_ports_.size(); i++)
    {
      // get every joint on that port
      for (auto &it : dynamixel_ports_[i].joints)
      {
        // enable motor torque
        if (!dynamixel_ports_[i].driver->setTorqueEnabled(it.second.id, it.second.model_spec->type, 1))
        {
          ROS_ERROR("failed to enable torque on motor %d", it.second.id);
        }

        // if in position control mode we enable the default join movement speed (profile velocity)
        if (control_type_ == kModePositionControl)
        {
          int regVal =
            static_cast<int>((static_cast<double>(it.second.max_vel) * it.second.model_spec->velocity_radps_to_reg));
          dynamixel_ports_[i].driver->setProfileVelocity(it.second.id, it.second.model_spec->type, regVal);
        }

        ROS_INFO("Torque enabled on %s joint", it.first.c_str());
        it.second.torque_enabled = true;
      }
    }

    first_write_ = false;
  }

  // Control loop rate of dataport reads
  if (dataport_rate_ > 0)
  {
    dataport_counter_++;
    if (dataport_counter_ >= dataport_iters_)
    {
      read_dataport_ = true;
      dataport_counter_ = 0;
    }
  }

  // Control loop rate of diagnostic reads
  if (diagnostics_rate_ > 0)
  {
    diagnostics_counter_++;
    if (diagnostics_counter_ >= diagnostics_iters_)
    {
      read_diagnostics_ = true;
      diagnostics_counter_ = 0;
    }
  }

  // spawn an IO thread for each additional port
  for (int i = 1; i < dynamixel_ports_.size(); i++)
  {
    num_servos = num_servos + dynamixel_ports_[i].joints.size();
    reads[i] = sensor_msgs::JointState();

    threads.emplace_back(&DynamixelInterfaceController::multiThreadedIO, this, std::ref(dynamixel_ports_[i]),
                         std::ref(reads[i]), std::ref(dataports_reads[i]), std::ref(diags_reads[i]), write_ready_);
  }

  // get messages for the first port
  reads[0] = sensor_msgs::JointState();

  num_servos = num_servos + dynamixel_ports_[0].joints.size();

  // keep the write message thread safe
  sensor_msgs::JointState temp_msg = write_msg_;

  // Perform the IO for the first port here (don't bother spinning out another thread)
  multiThreadedIO(dynamixel_ports_[0], reads[0], dataports_reads[0], diags_reads[0], write_ready_);

  // loop and get port information (wait for threads in order if any were created)
  for (int i = 0; i < dynamixel_ports_.size(); i++)
  {
    if (i > 0)
    {
      threads[i - 1].join();
    }

    if (reads[i].name.size() == 0)
    {
      // ROS_ERROR("No values passed back from %s thread", dynamixel_ports_[i].device.c_str());
      continue;
    }

    // append read values to published message
    read_msg.name.insert(read_msg.name.end(), reads[i].name.begin(), reads[i].name.end());
    read_msg.position.insert(read_msg.position.end(), reads[i].position.begin(), reads[i].position.end());
    read_msg.velocity.insert(read_msg.velocity.end(), reads[i].velocity.begin(), reads[i].velocity.end());
    read_msg.effort.insert(read_msg.effort.end(), reads[i].effort.begin(), reads[i].effort.end());

    // get dataport read info if available
    if (read_dataport_)
    {
      if (dataports_reads[i].states.size() > 0)
      {
        // append read values to published message
        dataports_msg.states.insert(dataports_msg.states.end(), dataports_reads[i].states.begin(),
                                    dataports_reads[i].states.end());
      }
    }

    // get diagnostics info if available
    if (read_diagnostics_)
    {
      if (diags_reads[i].diagnostics.size() > 0)
      {
        diags_msg.diagnostics.insert(diags_msg.diagnostics.end(), diags_reads[i].diagnostics.begin(),
                                     diags_reads[i].diagnostics.end());
      }
    }
  }

  // reset write flag
  if (write_ready_)
  {
    write_ready_ = false;
    write_msg_.name.clear();
    write_msg_.position.clear();
    write_msg_.velocity.clear();
    write_msg_.effort.clear();
  }

  lock.unlock();

  // publish joint states
  if (read_msg.name.size() > 0)
  {
    joint_state_publisher_.publish(read_msg);
  }

  // publish external dataport message
  if ((read_dataport_) && (dataports_msg.states.size() > 0))
  {
    dataport_publisher_.publish(dataports_msg);
    read_dataport_ = false;
  }

  if (read_diagnostics_ && (diags_msg.diagnostics.size() > 0))
  {
    diagnostics_publisher_.publish(diags_msg);
    read_diagnostics_ = false;
  }
}

/// Top level control function for each port's IO thread.
/// @param[in] port the port for this thread to perform IO on
/// @param[out] read_msg the JointState this threads joint data is read into
/// @param[out] dataport_msg the DataPorts msg this thread reads dataport data into
/// @param[out] status_msgs the ServoDiags msg this thread reads diagnostic data into
/// @param[in] perform_write whether we are writing data this iteration
void DynamixelInterfaceController::multiThreadedIO(PortInfo &port, sensor_msgs::JointState &read_msg,
                                                   dynamixel_interface::DataPorts &dataport_msg,
                                                   dynamixel_interface::ServoDiags &diags_msg, bool perform_write) const
{
  // perform write
  if (perform_write)
  {
    multiThreadedWrite(port, write_msg_);
  }

  // perform read
  multiThreadedRead(port, read_msg, dataport_msg, diags_msg);
}


/// Function called in each thread to perform a write on a port
/// @param[in] port_num index used to retrieve port information from port list
/// @param[in] joint_commands message cointaining the commands for each joint
void DynamixelInterfaceController::multiThreadedWrite(PortInfo &port, sensor_msgs::JointState joint_commands) const
{
  // ignore empty messages
  if (joint_commands.name.size() < 1)
  {
    return;
  }

  // booleans used to setup which parameters to write
  bool has_pos = false;
  bool has_vel = false;
  bool has_eff = false;

  // figure out which values have been specified
  if ((joint_commands.position.size() == joint_commands.name.size()) && (control_type_ == kModePositionControl))
  {
    has_pos = true;
  }
  if ((joint_commands.velocity.size() == joint_commands.name.size()) &&
      ((control_type_ == kModeVelocityControl) || (control_type_ == kModePositionControl && !ignore_input_velocity_)))
  {
    has_vel = true;
  }
  if ((joint_commands.effort.size() == joint_commands.name.size()) && (control_type_ == kModeTorqueControl))
  {
    has_eff = true;
  }

  if ((!has_pos) && (!has_vel) && (!has_eff))
  {
    // no valid array sizes for current control mode, ignore
    return;
  }

  // write objects
  std::unordered_map<int, SyncData> velocities, positions, efforts;

  // push motor encoder value onto list
  SyncData write_data;

  // loop and calculate the values for each specified joint
  for (int i = 0; i < joint_commands.name.size(); i++)
  {
    // lookup the information for that particular joint to be able to control it
    if (port.joints.find(joint_commands.name[i]) == port.joints.end())
    {
      // Joint not on this port, ignore
      continue;
    }

    // Retrieve dynamixel information
    DynamixelInfo *info = &port.joints[joint_commands.name[i]];

    // calculate the position register value for the motor
    if ((has_pos) && (control_type_ == kModePositionControl))
    {
      // used to bound positions to limits
      int up_lim, dn_lim;

      // get radian position value
      double rad_pos = joint_commands.position[i];

      // define our clamping limits to avoid exceeding safe joint angles
      if (info->min_pos > info->max_pos)
      {
        rad_pos = -rad_pos;
        up_lim = info->min_pos;
        dn_lim = info->max_pos;
      }
      else
      {
        up_lim = info->max_pos;
        dn_lim = info->min_pos;
      }

      // convert from radians to motor encoder value
      double pos =
        rad_pos * ((info->model_spec->encoder_cpr * 360.0) / (2 * M_PI * info->model_spec->encoder_range_deg)) +
        info->zero_pos;

      // clamp joint angle to be within safe limit
      if ((pos <= up_lim) && (pos >= dn_lim))
      {
        // push motor encoder value onto list
        write_data.id = info->id;
        write_data.type = info->model_spec->type;
        if (write_data.type <= kSeriesLegacyMX)
        {
          write_data.data.resize(2);
          *(reinterpret_cast<uint16_t*>(write_data.data.data())) = static_cast<uint16_t>(pos);
        }
        else
        {
          write_data.data.resize(4);
          *(reinterpret_cast<uint32_t*>(write_data.data.data())) = static_cast<uint32_t>(pos);
        }

        positions[info->id] = write_data;
      }
    }

    // calculate the velocity register value for the motor
    if (has_vel && (control_type_ != kModeTorqueControl))
    {
      // get rad/s value from message
      double rad_s_vel = joint_commands.velocity[i];

      // clamp to joint speed limit
      rad_s_vel = std::clamp(rad_s_vel, -info->max_vel, info->max_vel);

      // convert to motor encoder value
      int vel = (int)((rad_s_vel * info->model_spec->velocity_radps_to_reg));

      // Velocity values serve 2 different functions, in velocity control mode their sign
      // defines direction, however in position control mode their absolute value is used
      // to set the profile velocity (how fast the motor moves to the specified position)
      // we also need to take an absolute value as each motor series handles negative inputs
      // differently
      if ((control_type_ == kModeVelocityControl) && (info->min_pos > info->max_pos))
      {
        // The old dynamixel series use a different method of representing velocity values. They define the value in the
        // register as a signed 10-bit value, with the first 9 bits representing magnitude and the 10th bit representing
        // the sign.
        if (info->model_spec->type <= kSeriesLegacyMX)
        {
          vel = std::abs(vel) + 1024;
        }
        else
        {
          vel = -vel;
        }
      }
      else if (control_type_ == kModePositionControl)
      {
        vel = std::abs(vel);
      }

      // push motor encoder value onto list
      write_data.id = info->id;
      write_data.type = info->model_spec->type;
      if (write_data.type <= kSeriesLegacyMX)
      {
        write_data.data.resize(2);
        *(reinterpret_cast<uint16_t*>(write_data.data.data())) = static_cast<uint16_t>(vel);
      }
      else
      {
        write_data.data.resize(4);
        *(reinterpret_cast<uint32_t*>(write_data.data.data())) = static_cast<uint32_t>(vel);
      }

      velocities[info->id] = write_data;
    }

    if (has_eff && (control_type_ == kModeTorqueControl))
    {
      double input_effort = joint_commands.effort[i];

      int16_t effort = 0;

      // input and outputs are current values (in A)
      if (info->model_spec->effort_reg_to_mA != 0)
      {
        effort = (input_effort * 1000 / info->model_spec->effort_reg_to_mA);
        effort = abs(effort);

        if ((input_effort < 0) != (info->min_pos > info->max_pos))
        {
          effort = -effort;
        }
      }

      // push motor encoder value onto list
      write_data.id = info->id;
      write_data.type = info->model_spec->type;
      write_data.data.resize(2);
      *(reinterpret_cast<int16_t*>(write_data.data.data())) = effort;

      efforts[info->id] = write_data;
    }
  }

  // use the multi-motor write functions to reduce the bandwidth required to command
  // all the motors
  if (control_type_ == kModePositionControl)
  {
    // set the profile velocities if they have been defined
    if ((has_vel) && (!ignore_input_velocity_))
    {
      if (!port.driver->setMultiProfileVelocity(velocities))
      {
        ROS_ERROR("Failed to set profile velocities on port %s!", port.port_name.c_str());
      }
    }
    // send the positions to the motors
    if (has_pos)
    {
      if (!port.driver->setMultiPosition(positions))
      {
        ROS_ERROR("Failed to set positions on port %s!", port.port_name.c_str());
      }
    }
  }
  else if ((control_type_ == kModeVelocityControl) && has_vel)
  {
    // set the velocity values for each motor
    if (!port.driver->setMultiVelocity(velocities))
    {
      ROS_ERROR("Failed to set velocities on port %s!", port.port_name.c_str());
    }
  }
  else if ((control_type_ == kModeTorqueControl) && has_eff)
  {
    // set the effort values for each motor
    if (!port.driver->setMultiTorque(efforts))
    {
      ROS_ERROR("Failed to set efforts on port %s!", port.port_name.c_str());
    }
  }
}


/// Function in thread to perform read on a port.
/// @param[in] port the port for this thread to perform IO on
/// @param[out] read_msg the JointState this threads joint data is read into
/// @param[out] dataport_msg the DataPorts msg this thread reads dataport data into
/// @param[out] status_msgs the ServoDiags msg this thread reads diagnostic data into
void DynamixelInterfaceController::multiThreadedRead(PortInfo &port, sensor_msgs::JointState &read_msg,
                                                     dynamixel_interface::DataPorts &dataport_msg,
                                                     dynamixel_interface::ServoDiags &diags_msg) const
{
  bool comm_success;
  std::unordered_map<int, DynamixelState> state_map;
  std::unordered_map<int, DynamixelDiagnostic> diag_map;
  std::unordered_map<int, DynamixelDataport> data_map;

  // Iterate over all connected servos and add to list
  for (auto &it : port.joints)  //(map<string, DynamixelInfo>::iterator iter = port.joints.begin(); iter !=
                                // port.joints.end(); iter++)
  {
    state_map[it.second.id] = DynamixelState();
    state_map[it.second.id].id = it.second.id;
    state_map[it.second.id].type = it.second.model_spec->type;
    diag_map[it.second.id] = DynamixelDiagnostic();
    diag_map[it.second.id].id = it.second.id;
    diag_map[it.second.id].type = it.second.model_spec->type;
    if (it.second.model_spec->external_ports)
    {
      data_map[it.second.id] = DynamixelDataport();
      data_map[it.second.id].id = it.second.id;
      data_map[it.second.id].type = it.second.model_spec->type;
    }
  }

  // get state info back from all dynamixels
  if (port.driver->getBulkState(state_map))
  {
    // Iterate over all connected servos and add to list
    for (auto &it : port.joints)
    {
      // //joint name
      std::string joint_name = it.first;

      // ignore joints that failed to read
      if (!state_map[it.second.id].success)
      {
        ROS_WARN("FAILED TO READ DYNAMIXEL %s (id %d)!", joint_name.c_str(), it.second.id);
        continue;
      }

      // put joint name in message
      read_msg.name.push_back(joint_name);

      // POSITION VALUE
      // get position and convert to radians
      // due to the fact that some series of dynamixel do not have a full 360 deg output range, we must scale the,
      // encoder counts by this fractional value to get the output in radians, thus:
      // rad_pos = (count-count_initial_position) * (range/360) * (2*PI/encoder_cpr)
      double rad_pos = (state_map[it.second.id].position - it.second.zero_pos) *
                       (2.0 * M_PI * it.second.model_spec->encoder_range_deg) /
                       (360.0 * it.second.model_spec->encoder_cpr);
      if (it.second.min_pos > it.second.max_pos)
      {
        rad_pos = -rad_pos;
      }

      // Put position in message
      read_msg.position.push_back(rad_pos);

      // VELOCITY VALUE
      int raw_vel = state_map[it.second.id].velocity;

      // handle the sign of the value based on the motor series
      if (it.second.model_spec->type <= kSeriesLegacyMX)
      {
        // The old dynamixel series use a different method of representing effort values. They define the value in the
        // register as a signed 10-bit value, with the first 9 bits representing magnitude and the 10th bit representing
        // the sign.
        raw_vel = (raw_vel & 0x3FF);

        if ((raw_vel > 1023) && (it.second.max_pos > it.second.min_pos))
        {
          raw_vel = -raw_vel;
        }
        else if ((raw_vel < 1023) && (it.second.max_pos < it.second.min_pos))
        {
          raw_vel = -raw_vel;
        }
      }
      else if (it.second.min_pos > it.second.max_pos)
      {
        raw_vel = -raw_vel;
      }

      // convert to rad/s
      double rad_s_vel = (static_cast<double>(raw_vel)) / it.second.model_spec->velocity_radps_to_reg;

      // put velocity in message
      read_msg.velocity.push_back(rad_s_vel);

      // TORQUE EFFORT VALUE
      double effort = 0;

      if (it.second.model_spec->type <= kSeriesLegacyMX)
      {
        // The old dynamixel series use a different method of representing effort values. They define the value in the
        // register as a signed 10-bit value, with the first 9 bits representing magnitude and the 10th bit representing
        // the sign.
        effort = static_cast<double>(state_map[it.second.id].effort & 0x3FF) * it.second.model_spec->effort_reg_to_mA;
        // check sign
        if (state_map[it.second.id].effort < 1023)
        {
          effort = 0.0 - effort;
        }
      }
      else
      {
        effort = static_cast<double>(state_map[it.second.id].effort) * it.second.model_spec->effort_reg_to_mA;
      }

      if (it.second.min_pos > it.second.max_pos)
      {
        effort = -effort;
      }

      // put effort in message
      read_msg.effort.push_back(effort);
    }
    read_msg.header.stamp = ros::Time::now();
  }
  else
  {
    ROS_WARN("Failed to get bulk state on port %s", port.port_name.c_str());
  }


  if (read_dataport_ && (data_map.size() > 0))
  {
    if (port.driver->getBulkDataportInfo(data_map))
    {
      // dynamixel_interface::ServoState diag_msg;
      dataport_msg.header.frame_id = port.port_name.c_str();

      // Iterate over all connected servos and add to list
      for (auto &it : port.joints)
      {
        dynamixel_interface::DataPort msg;

        // ignore joints that failed to read
        if (!data_map[it.second.id].success)
        {
          continue;
        }

        // get data
        msg.name = it.first;
        msg.port_values.push_back(data_map[it.second.id].port_values[0]);
        msg.port_values.push_back(data_map[it.second.id].port_values[1]);
        msg.port_values.push_back(data_map[it.second.id].port_values[2]);
        msg.port_values.push_back(data_map[it.second.id].port_values[3]);

        // push msg into array
        dataport_msg.states.push_back(msg);
      }
    }
    else
    {
      ROS_WARN("Failed to get dataports on port %s", port.port_name.c_str());
    }
  }

  if (read_diagnostics_)
  {
    if (port.driver->getBulkDiagnosticInfo(diag_map))
    {
      // dynamixel_interface::ServoState diag_msg;
      diags_msg.header.frame_id = port.port_name.c_str();

      // Iterate over all connected servos and add to list
      for (auto &it : port.joints)
      {
        dynamixel_interface::ServoDiag msg;

        // ignore joints that failed to read
        if (!diag_map[it.second.id].success)
        {
          continue;
        }

        // get name
        msg.name = it.first;
        msg.id = it.second.id;
        msg.model_name = it.second.model_spec->name;

        // get voltage (div 10 as units are 0.1V)
        msg.voltage = diag_map[it.second.id].voltage / 10.0;

        // get temperatures
        msg.temperature = static_cast<double>(diag_map[it.second.id].temperature);

        // push msg into array
        diags_msg.diagnostics.push_back(msg);
      }
    }
    else
    {
      ROS_WARN("Failed to get diagnostics on port %s", port.port_name.c_str());
    }
  }
}

}  // namespace dynamixel_interface