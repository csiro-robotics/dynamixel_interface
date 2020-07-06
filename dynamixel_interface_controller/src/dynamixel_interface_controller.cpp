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
 * @file   dynamixel_interface_controller.cpp
 * @author Tom Molnar (Tom.Molnar@data61.csiro.au), Brian Axelrod
 * @date   January, 2017
 * @brief  Implements the dynamixel controller class and defines the main loop of operation
 */

#include <string>
#include <stdlib.h>
#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>


#define _USE_MATH_DEFINES
#include <math.h>


#include "yaml-cpp/yaml.h"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

#include <ros/package.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/transport_hints.h>
#include <dynamixel_interface_controller/dynamixel_interface_controller.h>
#include <dynamixel_interface_driver/dynamixel_interface_driver.h>

using namespace dynamixel_interface_controller;
using namespace std;


/**
 * Constructor, loads the motor configuration information from the specified yaml file and intialises
 * The motor states.
 */
DynamixelInterfaceController::DynamixelInterfaceController()
{
  shutting_down_ = false;
  nh_ = new ros::NodeHandle("~");

  //Stores config variables only used in init function
  std::string mode;

  //load all the info from the param server, with defaults
  nh_->param<double>("publish_rate", publish_rate_, 50.0);
  nh_->param<bool>("disable_torque_on_shutdown", stop_motors_on_shutdown_, false);


  nh_->param<bool>("ignore_input_velocity", ignore_input_velocity_, false);

  nh_->param<double>("dataport_rate", dataport_read_rate_, 0.0);
  nh_->param<double>("diagnostics_rate", diagnostics_rate_, 0.0);

  nh_->param<std::string>("control_mode", mode, "Position");

  nh_->param<double>("global_joint_speed", global_joint_speed_, 5.0);
  nh_->param<double>("global_torque_limit", global_torque_limit_, 1.0);


  //clamp read rates to sensible values
  if (publish_rate_ < 0)
  {
    publish_rate_ = 0;
  }

  if (diagnostics_rate_ < 0)
  {
    diagnostics_rate_ = 0;
  }

  if (dataport_read_rate_ < 0)
  {
    dataport_read_rate_ = 0;
  }
  else if (dataport_read_rate_ > 50)
  {
    dataport_read_rate_ = 50;
  }

  // Set control mode for this run
  if (!strncmp(mode.c_str(), "Position", 8))
  {
    control_type_ = dynamixel_interface_driver::DXL_POSITION_CONTROL;
  }
  else if (!strncmp(mode.c_str(), "Velocity", 8))
  {
    control_type_ = dynamixel_interface_driver::DXL_VELOCITY_CONTROL;
  }
  else
  {
    ROS_ERROR("Control Mode Not Supported!");
    ROS_BREAK();
  }

  // Initialise write state variables
  write_ready_ = false;
  first_write_ = true;

  //Attempt to parse information for each device (port)
  if (nh_->hasParam("ports"))
  {

    //PARSE ARRAY OF PORT INFORMATION
    XmlRpc::XmlRpcValue ports;
    nh_->getParam("ports", ports);
    parsePortInformation(ports);

    //shutdown if no valid ports
    if (dynamixel_ports_.size() == 0)
    {
      ROS_ERROR("No valid ports found, shutting_down_...");
      ROS_BREAK();
    }

  }
  else
  {
    ROS_ERROR("No port details loaded to param server");
    ROS_BREAK();
  }

  if (diagnostics_rate_ > 0)
  {
    diagnostics_publisher_ = nh_->advertise<dynamixel_interface_controller::ServoState>("/servo_diagnostics", 1);
  }

  if (dataport_read_rate_ > 0)
  {
    dataport_publisher_  = nh_->advertise<dynamixel_interface_controller::DataPort>("/external_dataport", 1);
  }

  //advertise the joint state input and output topics
  joint_state_publisher_  = nh_->advertise<sensor_msgs::JointState>("/joint_states", 1);

  joint_state_subscriber_ = nh_->subscribe<sensor_msgs::JointState>("/desired_joint_states",
        1, &DynamixelInterfaceController::jointStateCallback, this, ros::TransportHints().tcpNoDelay());

  // Set custom callback queue for either input or output callback
  io_handle_.setCallbackQueue(&io_queue_);
  io_spinner_ = new ros::AsyncSpinner(1, &io_queue_);
  io_spinner_->start();
  last_dataport_time_ = last_diagnostics_time_ = std::chrono::steady_clock::now();

}


/**
 * Destructor, deletes the objects holding the serial ports and disables the motors if required
 */
DynamixelInterfaceController::~DynamixelInterfaceController()
{

  ROS_INFO("shutting_down_ dynamixel_interface_controller");

  shutting_down_ = false;
  io_spinner_->stop();
  delete nh_;

  if (stop_motors_on_shutdown_)
  {
    for (int i = 0; i < dynamixel_ports_.size(); i++)
    {
      //Turn off all the motors
      for (auto& it : dynamixel_ports_[i].joints)
      {
        dynamixel_ports_[i].driver->setTorqueEnabled(it.second.id, it.second.model_spec->type, 0);
        printf("Torque disabled on %s joint\n", it.first.c_str());
      }
    }
  }

  for (int i = 0; i < dynamixel_ports_.size(); i++)
  {
    //Delete driver objects
    delete dynamixel_ports_[i].driver;
  }
}



/**
 * Parses the information in the yaml file for each port
 * @param ports: the xml structure to be parsed
 */
void DynamixelInterfaceController::parsePortInformation(XmlRpc::XmlRpcValue ports)
{

  //If there is no servos array in the param server, return
  if (!ports.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Invalid/missing device information on the param server");
    ROS_BREAK();
  }

  //number of ports defined
  int num_ports = ports.size();

  //For every port, load and verify its information
  for (int i = 0; i < ports.size(); i++)
  {

    PortInfo port;

    bool use_group_read;
    bool use_group_write;

    /************************* PORT ARGUMENTS *********************/


    //check if info exists for specified port
    if (!ports[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Invalid/Missing info-struct for servo index %d", i);
      ROS_BREAK();
    }


    //get port name
    if (!ports[i]["port_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Invalid/Missing port name for port %d", i);
      ROS_BREAK();
    }
    else
    {
      port.port_name = static_cast<std::string>(ports[i]["port_name"]);
    }


    //get device name
    if (!ports[i]["device"].getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Invalid/Missing device name for port %d", i);
      ROS_BREAK();
    }
    else
    {
      port.device = static_cast<std::string>(ports[i]["device"]);
    }

    //get baud rate
    if (!ports[i]["baudrate"].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("Invalid/Missing baudrate for port %d", i);
      ROS_BREAK();
    }
    else
    {
      port.baudrate = static_cast<int>(ports[i]["baudrate"]);
    }

    //get protocol
    if (!ports[i]["use_legacy_protocol"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      ROS_ERROR("Invalid/Missing use_legacy_protocol option for port %d", i);
      ROS_BREAK();
    }
    else
    {
      port.use_legacy_protocol = static_cast<bool>(ports[i]["use_legacy_protocol"]);
    }

    //get group read enabled
    if (!ports[i]["group_read_enabled"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      ROS_ERROR("Invalid/Missing group_read_enabled option for port %d", i);
      ROS_BREAK();
    }
    else
    {
      use_group_read = static_cast<bool>(ports[i]["group_read_enabled"]);
    }

    //get group write enabled
    if (!ports[i]["group_write_enabled"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      ROS_ERROR("Invalid/Missing group_write_enabled option for port %d", i);
      ROS_BREAK();
    }
    else
    {
      use_group_write = static_cast<bool>(ports[i]["group_write_enabled"]);
    }


    /************************* Driver initialisation *********************/

    //Attempt to start driver
    try
    {
      port.driver = new dynamixel_interface_driver::DynamixelInterfaceDriver(port.device, port.baudrate,
          port.use_legacy_protocol, use_group_read, use_group_write);
    }
    catch (int n)
    {
      ROS_ERROR("Unable to start driver!, code %d", n);
      ROS_BREAK();
    }

    /************************* Dynamixel initialisation *********************/

    XmlRpc::XmlRpcValue servos;

    //If there is no servos array in the param server, return
    if (!servos.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Invalid/missing servo information on the param server");
      ROS_BREAK();
    }
    else
    {
      servos = ports[i]["servos"];
    }

    parseServoInformation(port, servos);

    //add port only if dynamixels were found
    if (port.joints.size() > 0)
    {
      //add port information to server
      dynamixel_ports_.push_back(port);
    }
    else
    {
      ROS_ERROR("No dynamixels found on %s, %s will not be used", port.device.c_str(),
          port.port_name.c_str());
    }
  }
}




/**
 * Parses the information in the yaml file for each servo
 * @param servos: the xml structure to be parsed
 */
void DynamixelInterfaceController::parseServoInformation(PortInfo &port, XmlRpc::XmlRpcValue servos)
{

  //number of servos defined
  int num_servos = servos.size();

  //For every servo, load and verify its information
  for (int i = 0; i < servos.size(); i++)
  {
    //store the loaded information in this struct
    DynamixelInfo info;

    //check if info exists for specified joint
    if (!servos[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Invalid/Missing info-struct for servo index %d", i);
      ROS_BREAK();
    }

    //get joint id
    if ((!servos[i].hasMember("id")) || (!servos[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt))
    {
      ROS_ERROR("Invalid/Missing id for servo index %d", i);
      ROS_BREAK();
    }
    else
    {
      //store the servo's ID
      info.id = static_cast<int>(servos[i]["id"]);
    }

    //get joint name
    if ((!servos[i].hasMember("joint_name")) || (!servos[i]["joint_name"].getType() == XmlRpc::XmlRpcValue::TypeString))
    {
      ROS_ERROR("Invalid/Missing joint name for servo index %d, id: %d", i, info.id);
      ROS_BREAK();
    }
    else
    {
      //store the servo's corresponding joint name
      info.joint_name = static_cast<std::string>(servos[i]["joint_name"]);

      //check this port and all previous ports for duplicate joint names (not allowed as joints are
      //referred to by name)
      if (port.joints.find(info.joint_name) !=  port.joints.end() )
      {
        ROS_ERROR("Cannot have multiple joints with the same name [%s]", info.joint_name.c_str());
        ROS_BREAK();
      }
      else
      {
        for (int j = 0; j < dynamixel_ports_.size(); j++)
        {
          if (dynamixel_ports_[j].joints.find(info.joint_name) !=  dynamixel_ports_[j].joints.end())
          {
            ROS_ERROR("Cannot have multiple joints with the same name [%s]", info.joint_name.c_str());
            ROS_BREAK();
          }
        }
      }
    }

    //get joint initial position
    if ((!servos[i].hasMember("init")) || (!servos[i]["init"].getType() == XmlRpc::XmlRpcValue::TypeInt))
    {
      ROS_WARN("Invalid/Missing initial position for servo index %d, id: %d", i, info.id);
      ROS_BREAK();
    }
    else
    {
      //store the servo's corresponding joint
      info.init = static_cast<int>(servos[i]["init"]);
    }

    //get joint default min position
    if ((!servos[i].hasMember("min")) || (!servos[i]["min"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
    {
      ROS_WARN("Invalid/Missing min position for servo index %d, id: %d", i, info.id);
      ROS_BREAK();
    }
    else
    {
      info.min = static_cast<int>(servos[i]["min"]);
    }

    //get joint default max position
    if ((!servos[i].hasMember("max")) || (servos[i]["max"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
    {
      ROS_WARN("Invalid/Missing max position for servo index %d, id: %d", i, info.id);
      ROS_BREAK();
    }
    else
    {
      info.max = static_cast<int>(servos[i]["max"]);
    }

    //get joint default joint speed (or set to global if none specified)
    if ((!servos[i].hasMember("joint_speed")) || (!servos[i]["joint_speed"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
    {
      info.joint_speed = global_joint_speed_;
    }
    else
    {
      info.joint_speed = static_cast<double>(servos[i]["joint_speed"]);
      if (info.joint_speed < 0.0)
      {
          info.joint_speed = global_joint_speed_;
      }
    }


    //get joint torque limit (or set to global if none specified)
    if ((!servos[i].hasMember("torque_limit")) || (!servos[i]["torque_limit"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
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

    //sleep to make sure the bus is clear of comms
    ros::Duration(0.2).sleep();

    //Ping the servo to make sure that we can actually connect to it
    //and that it is alive and well on our bus, if not, sleep and try again
    //if all retry's fail, throw an error
    bool ping_success = true;
    int ping_count = 0;
    while (!port.driver->ping(info.id))
    {
      //increment ping count
      ping_count++;

      ROS_WARN("Failed to ping id: %d, attempt %d, retrying...", info.id, ping_count);
      //max number of retry's
      if (ping_count > 5)
      {
        //unable to ping
        ping_success = false;
        break;
      }

      //sleep and try again
      ros::Duration(0.5).sleep();
    }

    //only add if ping was successful
    if (ping_success)
    {
      bool success = true;
      uint16_t model_number = 0;
      bool t_e;
      success = port.driver->getModelNumber(info.id, &model_number);

      //If valid motor, setup in operating mode
      if ((success) && (model_number))
      {
        info.model_spec = port.driver->getModelSpec(model_number);
        if (info.model_spec == NULL)
        {
          ROS_ERROR("Failed to load model information for dynamixel id %d", info.id);
          ROS_ERROR("Model Number: %d", model_number);
          ROS_ERROR("Info is not in database");
          ROS_BREAK();
        }

        info.torque_enabled = false;

        //Display joint info
        ROS_INFO("Joint Name: %s, ID: %d, Model: %s", info.joint_name.c_str(), info.id, info.model_spec->name.c_str());

        //maintain torque state in motor
        port.driver->getTorqueEnabled(info.id, info.model_spec->type, &t_e);
        port.driver->setTorqueEnabled(info.id, info.model_spec->type, 0);

        //set operating mode for the motor
        if (!port.driver->setOperatingMode(info.id, info.model_spec->type, control_type_))
        {
          ROS_WARN("Failed to set operating mode for %s motor (id %d)", info.joint_name.c_str(), info.id);
          ROS_BREAK();
        }

        //set torque limit for the motor
        if (!port.driver->setMaxTorque(info.id, info.model_spec->type, (int) (info.torque_limit * info.model_spec->effort_ratio)))
        {
          ROS_WARN("Failed to set torque limit for %s motor (id %d)", info.joint_name.c_str(), info.id);
        }

        //angle limits are only relevant in position control mode
        if (control_type_ == dynamixel_interface_driver::DXL_POSITION_CONTROL)
        {
          //set angle limits & direction
          if (info.min > info. max)
          {
            if (!port.driver->setAngleLimits(info.id, info.model_spec->type, info.max, info.min))
            {
              ROS_WARN("Failed to set angle limits for %s motor (id %d)", info.joint_name.c_str(), info.id);
            }
          }
          else
          {
            if (!port.driver->setAngleLimits(info.id, info.model_spec->type, info.min, info.max))
            {
              ROS_WARN("Failed to set angle limits for %s motor (id %d)", info.joint_name.c_str(), info.id);
            }
          }
        }

        //preserve torque enable state
        port.driver->setTorqueEnabled(info.id, info.model_spec->type, t_e);

        //store current control mode
        info.current_mode = control_type_;

        //add joint to port
        port.joints[info.joint_name] = info;

      }
      else
      {
        //can detect but cannot communicate, possibly serial error
        ROS_ERROR("Failed to retrieve model number for id %d", info.id);
        ROS_BREAK();
      }
    }
    else
    {
        //unable to detect motor
        ROS_ERROR("Cannot ping dynamixel id: %d", info.id);
    }
  }
}


/**
 * Start broadcasting JointState messages corresponding to the connected dynamixels
 */
void DynamixelInterfaceController::startBroadcastingJointStates()
{

  broadcast_timer_ = io_handle_.createTimer(ros::Duration(1.0 / publish_rate_),
      &DynamixelInterfaceController::publishJointStates, this);

}


/**
 * Callback for recieving a command from the /desired_joint_state topic.
 * The function atomically updates the class member variable containing the latest message and sets
 * a flag indicating a new message has been received
 * @param joint_commands the command received from the topic
 */
void DynamixelInterfaceController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_commands)
{

  //thread safe modification
  std::unique_lock<std::mutex> lock(write_mutex_);

  if (write_msg_.name.empty())
  {
    write_msg_ = *joint_commands;
  }
  //avoid array length issues
  else if ((write_msg_.position.empty() == joint_commands->position.empty()) &&
      (write_msg_.velocity.empty() == joint_commands->velocity.empty()) &&
      (write_msg_.effort.empty() == joint_commands->effort.empty()))
  {
    //only add joint if not already in list (THIS TAKES N*M TIME, NEED TO OPTIMISE)
    for (int i = 0; i < joint_commands->name.size(); i++)
    {
      bool in_msg = false;

      //if joint already in message, update values
      for (int j = 0; j < write_msg_.name.size(); j++)
      {
        if (joint_commands->name[i] == write_msg_.name[j])
        {

          if(!write_msg_.position.empty() && !joint_commands->position.empty())
          {
            write_msg_.position[j] = joint_commands->position[i];
          }

          if(!write_msg_.velocity.empty() && !joint_commands->velocity.empty())
          {
            write_msg_.velocity[j] = joint_commands->velocity[i];
          }

          if(!write_msg_.effort.empty() && !joint_commands->effort.empty())
          {
            write_msg_.effort[j] = joint_commands->effort[i];
          }

          in_msg = true;
          break;
        }
      }

      //not already in message, push back
      if (!in_msg)
      {
        write_msg_.name.push_back(joint_commands->name[i]);

        if(!write_msg_.position.empty() && !joint_commands->position.empty())
        {
          write_msg_.position.push_back(joint_commands->position[i]);
        }

        if(!write_msg_.velocity.empty() && !joint_commands->velocity.empty())
        {
          write_msg_.velocity.push_back(joint_commands->velocity[i]);
        }

        if(!write_msg_.effort.empty() && !joint_commands->effort.empty())
        {
          write_msg_.effort.push_back(joint_commands->effort[i]);
        }
      }
    }
  }
  write_ready_ = true;
  lock.unlock();

}

/**
 * TimeEvent callback for handling top level control of IO (for multiple ports).
 * Function spawns and waits on a thread for each
 */
void DynamixelInterfaceController::publishJointStates(const ros::TimerEvent& event)
{
  //don't access the driver after its been cleaned up
  if (shutting_down_)
      return;

  int num_servos = 0;
  std::vector<std::thread> threads;

  sensor_msgs::JointState read_msg;
  sensor_msgs::JointState reads[dynamixel_ports_.size()];

  dynamixel_interface_controller::DataPort dataport_msg;
  dynamixel_interface_controller::DataPort dataport_reads[dynamixel_ports_.size()];

  dynamixel_interface_controller::ServoState status_msg;
  dynamixel_interface_controller::ServoState status_reads[dynamixel_ports_.size()];

  std::unique_lock<std::mutex> lock(write_mutex_);

  //enable torque only once we start receiving commands
  if (write_ready_ && first_write_)
  {
    //loop over every port
    for (int i = 0; i < dynamixel_ports_.size(); i++)
    {
      //get every joint on that port
      for (auto& it : dynamixel_ports_[i].joints)
      {
        int32_t priorPos = it.second.init;

        //enable motor torque
        if (!dynamixel_ports_[i].driver->setTorqueEnabled(it.second.id, it.second.model_spec->type, 1))
        {
          ROS_ERROR("failed to enable torque on motor %d", it.second.id);
        }

        //if in position control mode we enable the default join movement speed (profile velocity)
        if (control_type_ == dynamixel_interface_driver::DXL_POSITION_CONTROL)
        {
          int regVal = (int) ((double) (it.second.joint_speed) * (60/(2.0 * M_PI)) * it.second.model_spec->gear_conversion);
          dynamixel_ports_[i].driver->setProfileVelocity(it.second.id, it.second.model_spec->type, regVal);
        }

        ROS_INFO("Torque enabled on %s joint", it.first.c_str());
        it.second.torque_enabled = true;
      }
    }

    first_write_ = false;
  }

  dataport_msg.header.stamp = read_msg.header.stamp = ros::Time::now();

  //spawn an IO thread for each additional port
  for (int i = 1; i < dynamixel_ports_.size(); i++)
  {
    num_servos = num_servos + dynamixel_ports_[i].joints.size();
    reads[i] = sensor_msgs::JointState();

    std::thread readThread(&DynamixelInterfaceController::multiThreadedIO, this,
        std::ref(dynamixel_ports_[i]), std::ref(reads[i]),
        std::ref(dataport_reads[i]), std::ref(status_reads[i]), write_ready_);

    threads.push_back(move(readThread));
  }

  //get messages for the first port
  reads[0] = sensor_msgs::JointState();

  num_servos = num_servos + dynamixel_ports_[0].joints.size();

  //keep the write message thread safe
  sensor_msgs::JointState temp_msg = write_msg_;

  //perform the IO on the first port

  //perform write
  if (write_ready_)
  {
    multiThreadedWrite(dynamixel_ports_[0], temp_msg);
  }

  //perform read
  multiThreadedRead(dynamixel_ports_[0], reads[0], dataport_reads[0], status_reads[0]);


  //loop and get port information (wait for threads in order if any were created)
  for (int i = 0; i < dynamixel_ports_.size(); i++)
  {

    if (i > 0)
    {
      threads[i-1].join();
    }

    if (reads[i].name.size() == 0)
    {
      //ROS_ERROR("No values passed back from %s thread", dynamixel_ports_[i].device.c_str());
      continue;
    }

    //append read values to published message
    read_msg.name.insert(read_msg.name.end(), reads[i].name.begin(), reads[i].name.end());
    read_msg.position.insert(read_msg.position.end(), reads[i].position.begin(), reads[i].position.end());
    read_msg.velocity.insert(read_msg.velocity.end(), reads[i].velocity.begin(), reads[i].velocity.end());
    read_msg.effort.insert(read_msg.effort.end(), reads[i].effort.begin(), reads[i].effort.end());

    // get dataport read info if available
    if (read_dataport_)
    {
      if (dataport_reads[i].name.size() == 0)
      {
        continue;
      }

      //append read values to published message
      dataport_msg.name.insert(dataport_msg.name.end(), dataport_reads[i].name.begin(), dataport_reads[i].name.end());
      dataport_msg.value.insert(dataport_msg.value.end(), dataport_reads[i].value.begin(), dataport_reads[i].value.end());
    }

    //get diagnostics info if available
    if (publish_diagnostics_)
    {
      if(status_reads[i].joint_names.size() == 0)
      {
        continue;
      }

      status_msg.joint_names.insert(status_msg.joint_names.end(), status_reads[i].joint_names.begin(), status_reads[i].joint_names.end());
      status_msg.voltages.insert(status_msg.voltages.end(), status_reads[i].voltages.begin(), status_reads[i].voltages.end());
      status_msg.temperatures.insert(status_msg.temperatures.end(), status_reads[i].temperatures.begin(), status_reads[i].temperatures.end());
      status_msg.error_states.insert(status_msg.error_states.end(), status_reads[i].error_states.begin(), status_reads[i].error_states.end());
      status_msg.modes.insert(status_msg.modes.end(), status_reads[i].modes.begin(), status_reads[i].modes.end());

      //check error status of each dynamixel and report new errors
      for (int j = 0; j < status_reads[i].joint_names.size(); j++)
      {
        DynamixelInfo info = dynamixel_ports_[i].joints.at(status_reads[i].joint_names[j]);
        if (status_reads[i].error_states[j] != 0)
        {
          if (info.hardware_status != status_reads[i].error_states[j])
          {
            ROS_WARN("Dynamixel Error! Joint %s (id %d) has returned with code %d!",
                info.joint_name.c_str(), info.id, status_reads[i].error_states[j]);
          }
          info.hardware_status = status_reads[i].error_states[j];
          dynamixel_ports_[i].joints.at(status_reads[i].joint_names[j]) = info;
        }
      }
    }
  }

  //reset write flag
  if (write_ready_)
  {
    write_ready_ = false;
    write_msg_.name.clear();
    write_msg_.position.clear();
    write_msg_.velocity.clear();
    write_msg_.effort.clear();
  }

  lock.unlock();

  //publish joint states
  if (read_msg.name.size() > 0)
  {
    joint_state_publisher_.publish(read_msg);
  }

  //publish external dataport message
  if ((read_dataport_) && (dataport_msg.name.size() > 0))
  {
    dataport_publisher_.publish(dataport_msg);
    read_dataport_ = false;
  }

  if (publish_diagnostics_)
  {
    diagnostics_publisher_.publish(status_msg);
    publish_diagnostics_ = false;
  }

  //timestamp
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  //Control loop rate of dataport reads
  std::chrono::duration<double> delta = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_dataport_time_);
  if (delta.count() >= (1-(0.5*dataport_read_rate_/publish_rate_)) / (dataport_read_rate_))
  {
    read_dataport_ = true;
    last_dataport_time_ = now;
  }

  //Control loop rate of diagnostic reads
  delta = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_diagnostics_time_);
  if (delta.count() >= (1-(0.5*diagnostics_rate_/publish_rate_)) / (diagnostics_rate_))
  {
    publish_diagnostics_ = true;
    last_diagnostics_time_ = now;
  }

}

/**
 * Top level control function for each port's IO thread.
 * @param port_num index used to retrieve port information from port list
 * @param read_msg the msg this threads join data is read into, this is then combined by the top level function.
 * @param perform_write boolean indicating whether or not to write latest joint_state to servos
 */
void DynamixelInterfaceController::multiThreadedIO(PortInfo &port, sensor_msgs::JointState &read_msg,
                                                    dynamixel_interface_controller::DataPort &dataport_msg,
                                                    dynamixel_interface_controller::ServoState &status_msg,
                                                    bool perform_write)
{

  sensor_msgs::JointState thread_write_msg = write_msg_;

  //perform write
  if (write_ready_)
  {
    multiThreadedWrite(port, thread_write_msg);
  }

  //perform read
  multiThreadedRead(port, read_msg, dataport_msg, status_msg);

}


/**
 * Function called in each thread to perform a write on a port
 * @param port_num index used to retrieve port information from port list
 * @param joint_commands message cointaining the commands for each joint
 */
void DynamixelInterfaceController::multiThreadedWrite(PortInfo &port, sensor_msgs::JointState joint_commands)
{
  //ignore empty messages
  if (joint_commands.name.size() < 1)
  {
    return;
  }

  //booleans used to setup which parameters to write
  bool has_pos = false;
  bool has_vel = false;

  //figure out which values have been specified
  if ((joint_commands.position.size() == joint_commands.name.size()) && (control_type_ == dynamixel_interface_driver::DXL_POSITION_CONTROL))
  {
    has_pos = true;
  }
  if ((joint_commands.velocity.size() == joint_commands.name.size()) && ((control_type_ == dynamixel_interface_driver::DXL_VELOCITY_CONTROL) ||
      (control_type_ == dynamixel_interface_driver::DXL_POSITION_CONTROL && !ignore_input_velocity_)))
  {
    has_vel = true;
  }

  if ((!has_pos) && (!has_vel))
  {
    //no valid array sizes for current control mode, ignore
    return;
  }

  //write objects
  std::unordered_map<int, dynamixel_interface_driver::SyncData> velocities, positions;

  //push motor encoder value onto list
  dynamixel_interface_driver::SyncData write_data;

  //loop and calculate the values for each specified joint
  for (int i = 0; i < joint_commands.name.size(); i++)
  {

    //lookup the information for that particular joint to be able to control it
    if (port.joints.find(joint_commands.name[i]) == port.joints.end())
    {
      //Joint not on this port, ignore
      continue;
    }

    //Retrieve dynamixel information
    DynamixelInfo *info = &port.joints[joint_commands.name[i]];

    //calculate the position register value for the motor
    if ((has_pos) && (control_type_ == dynamixel_interface_driver::DXL_POSITION_CONTROL))
    {

      //used to bound positions to limits
      int up_lim, dn_lim;

      //get radian position value
      double rad_pos = joint_commands.position[i];

      //define our clamping limits to avoid exceeding safe joint angles
      if (info->min > info->max)
      {
        rad_pos = -rad_pos;
        up_lim = info->min;
        dn_lim = info->max;
      }
      else
      {
        up_lim = info->max;
        dn_lim = info->min;
      }

      //convert from radians to motor encoder value
      double pos = (rad_pos / 2.0 / M_PI * info->model_spec->cpr + 0.5) + info->init;

      //clamp joint angle to be within safe limit
      if ((pos <= up_lim) && (pos >= dn_lim))
      {
        //push motor encoder value onto list
        write_data.id = info->id;
        write_data.type = info->model_spec->type;
        if (write_data.type <= dynamixel_interface_driver::DXL_LEGACY_MX)
        {
          write_data.data.resize(2);
          *((uint16_t*) write_data.data.data()) = (uint16_t) pos;
        }
        else
        {
          write_data.data.resize(4);
          *((uint32_t*) write_data.data.data()) = (uint32_t) pos;
        }

        positions[info->id] = write_data;
      }
    }

    //calculate the velocity register value for the motor
    if (has_vel)
    {

      //get rad/s value from message
      double rad_s_vel = joint_commands.velocity[i];

      //clamp to joint speed limit
      if (abs(rad_s_vel) > info->joint_speed)
      {
        rad_s_vel = (rad_s_vel < 0) ? (-info->joint_speed) : (info->joint_speed);
      }

      //convert to motor encoder value
      int vel = (int) ((rad_s_vel * (60/(2.0 * M_PI)) * info->model_spec->gear_conversion));

      //Velocity values serve 2 different functions, in velocity control mode their sign
      //defines direction, however in position control mode their absolute value is used
      //to set the profile velocity (how fast the motor moves to the specified position)
      vel = abs(vel);

      //we also need to take an absolute value as each motor series handles negative inputs
      //differently
      if ((control_type_ == dynamixel_interface_driver::DXL_VELOCITY_CONTROL) && ((rad_s_vel < 0) != (info->min > info->max)))
      {
        if (info->model_spec->type <= dynamixel_interface_driver::DXL_LEGACY_MX)
        {
          vel = vel + 1024;
        }
        else
        {
          vel = -vel;
        }
      }

      //push motor encoder value onto list
      write_data.id = info->id;
      write_data.type = info->model_spec->type;
      if (write_data.type <= dynamixel_interface_driver::DXL_LEGACY_MX)
      {
        write_data.data.resize(2);
        *((uint16_t*) write_data.data.data()) = (uint16_t) vel;
      }
      else
      {
        write_data.data.resize(4);
        *((int32_t*) write_data.data.data()) = (int32_t) vel;
      }

      velocities[info->id] = write_data;

    }
  }

  //use the multi-motor write functions to reduce the bandwidth required to command
  //all the motors
  if (control_type_ == dynamixel_interface_driver::DXL_POSITION_CONTROL)
  {
    //set the profile velocities if they have been defined
    if ((has_vel) && (!ignore_input_velocity_))
    {
      port.driver->setMultiProfileVelocity(velocities);
    }
    //send the positions to the motors
    if (has_pos)
    {
      port.driver->setMultiPosition(positions);
    }
  }
  else if ((control_type_ == dynamixel_interface_driver::DXL_VELOCITY_CONTROL) && has_vel)
  {
    //set the velocity values for each motor
    port.driver->setMultiVelocity(velocities);
  }

}


/**
 * Function in thread to perform read on a port.
 * @param port_num index used to retrieve port information from port list
 * @param read_msg the msg this ports join data is read into.
 */
void DynamixelInterfaceController::multiThreadedRead(PortInfo &port, sensor_msgs::JointState &read_msg,
                                                     dynamixel_interface_controller::DataPort &dataport_msg,
                                                     dynamixel_interface_controller::ServoState &status_msg)
{

  bool comm_success;
  std::unordered_map<int, dynamixel_interface_driver::DynamixelState> state_map;
  std::unordered_map<int, dynamixel_interface_driver::DynamixelDiagnostic> diag_map;

  //Iterate over all connected servos and add to list
  for (auto& it : port.joints) //(map<string, DynamixelInfo>::iterator iter = port.joints.begin(); iter != port.joints.end(); iter++)
  {
    state_map[it.second.id] = dynamixel_interface_driver::DynamixelState();
    state_map[it.second.id].id = it.second.id;
    state_map[it.second.id].type = it.second.model_spec->type;
    diag_map[it.second.id] = dynamixel_interface_driver::DynamixelDiagnostic();
  }

  //get state info back from all dynamixels
  if(port.driver->getBulkState(state_map))
  {

    //Iterate over all connected servos and add to list
    for (auto& it : port.joints)
    {
      // //joint name
      std::string joint_name = it.first;

      //ignore joints that failed to read
      if(!state_map[it.second.id].success)
      {
        ROS_INFO("FAILED TO READ DYNAMIXEL %s (id %d)!", joint_name.c_str(), it.second.id);
        continue;
      }

      //put joint name in message
      read_msg.name.push_back(joint_name);

      //POSITION VALUE
      //get position and convert to radians
      double rad_pos = (state_map[it.second.id].position - it.second.init) / ((double) (it.second.model_spec->cpr)) * 2 * M_PI;
      if (it.second.min > it.second.max)
      {
        rad_pos = -rad_pos;
      }

      //Put position in message
      read_msg.position.push_back(rad_pos);

      //VELOCITY VALUE
      int raw_vel = state_map[it.second.id].velocity;

      //handle the sign of the value based on the motor series
      if (it.second.model_spec->type <= dynamixel_interface_driver::DXL_LEGACY_MX)
      {
        raw_vel = (raw_vel & 0x3FF);

        if ((raw_vel > 1023) && (it.second.max > it.second.min))
        {
          raw_vel = -raw_vel;
        }
        else if ((raw_vel < 1023) && (it.second.max < it.second.min))
        {
          raw_vel = -raw_vel;
        }
      }
      else if (it.second.min > it.second.max)
      {
        raw_vel = -raw_vel;
      }

      //convert to rad/s
      double rad_s_vel = ((double) raw_vel) * ((2.0 * M_PI) / 60.0) / it.second.model_spec->gear_conversion;

      //put velocity in message
      read_msg.velocity.push_back(rad_s_vel);

      //TORQUE EFFORT VALUE
      double effort = 0;

      if (it.second.model_spec->type <= dynamixel_interface_driver::DXL_LEGACY_MX)
      {
        effort = ((double) (state_map[it.second.id].effort & 0x3FF)) * it.second.model_spec->current_ratio;
        //check sign
        if (state_map[it.second.id].effort < 1023)
        {
          effort = 0.0 - effort;
        }
      }
      else
      {
        effort = ((double) (state_map[it.second.id].effort) * it.second.model_spec->current_ratio);
      }

      if (it.second.min > it.second.max)
      {
        effort = -effort;
      }

      //put effort in message
      read_msg.effort.push_back(effort);
    }
    read_msg.header.stamp = ros::Time::now();
  }

  if (read_dataport_)
  {
    // //put joint name in message
    // dataport_msg.name.push_back(joint_name);

    // //publish dataport value
    // dataport_msg.value.push_back(response[3]);
  }

  if (publish_diagnostics_)
  {
    if( port.driver->getBulkDiagnosticInfo(diag_map) )
    {
      //dynamixel_interface_controller::ServoState diag_msg;
      status_msg.header.frame_id = port.port_name.c_str();

      //Iterate over all connected servos and add to list
      for (auto& it : port.joints) //(map<string, DynamixelInfo>::iterator iter = port.joints.begin(); iter != port.joints.end(); iter++)
      {
        //joint name
        std::string joint_name = it.first;

        //ignore joints that failed to read
        if (!diag_map[it.second.id].success)
        {
          continue;
        }

        //put effort in message
        status_msg.joint_names.push_back(joint_name.c_str());

        //get voltage
        status_msg.voltages.push_back((double) (diag_map[it.second.id].voltage) / 10.0);

        //get temperatures
        status_msg.temperatures.push_back((double) (diag_map[it.second.id].temperature));

        // status_msg.error_states.push_back(response[2]);
        // if (response[2] != 0)
        // {
        //     ROS_WARN("Dynamixel Error! Joint %s (id %d) has returned with code %d", joint_name, info.id, response[2]);
        // }
        status_msg.modes.push_back(it.second.current_mode);
      }
    }
  }
}



/** Main loop. intialises controller and starts timer and ROS callback handling */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "dynamixel_interface_controller");

  DynamixelInterfaceController controller;

  //Initialize Timer callback and the async spinner for write commands, this starts the IO.
  controller.startBroadcastingJointStates();

//   ros::MultiThreadedSpinner spinner(2); // Use 2 threads
//   spinner.spin(); // spin() will not return until the node has been shutdown

  //Use a single threaded spinner for the global queue to service the joint states callback, (this callback will be
  //primarily rate limited by the motors)
  ros::spin();

}


