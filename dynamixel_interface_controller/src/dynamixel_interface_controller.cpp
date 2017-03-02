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
#include <XmlRpcValue.h>

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

    //load the file containing model info, we're not using the param server here
    string path = ros::package::getPath("dynamixel_interface_controller");
    path += "/config/motor_data.yaml";

    YAML::Node doc;

    #ifdef HAVE_NEW_YAMLCPP
        doc = YAML::LoadFile(path);
    #else
        ifstream fin(path.c_str());
        YAML::Parser parser(fin);
        parser.GetNextDocument(doc);
    #endif

    //read in the dynamixel model information from the motor data file
    for (int i = 0; i < doc.size(); i++)
    {
        dynamixelSpec spec;

        // Load the basic specs of this motor type 
        doc[i]["name"] >> spec.name;
        doc[i]["model_number"] >> spec.model_number;
        doc[i]["cpr"]  >> spec.cpr;
        doc[i]["gear_reduction"]  >> spec.gear_reduction;
        doc[i]["torque_ratio"] >> spec.torque_ratio;
        doc[i]["current_ratio"] >> spec.current_ratio;

        model_number2specs_[spec.model_number] = spec;
    }

    //Stores config variables only used in init function
    std::string mode;
    double global_joint_speed;
    double global_torque_limit;
    double global_p_gain;
    double global_i_gain;
    double global_d_gain;

    //load all the info from the param server, with defaults
    nh_->param<double>("publish_rate", publish_rate_, 50.0);
    nh_->param<bool>("disable_torque_on_shutdown", stop_motors_on_shutdown_, false);
    nh_->param<bool>("echo_joint_commands", echo_joint_commands_, false);
    nh_->param<bool>("use_torque_as_effort", use_torque_as_effort_, false);
    nh_->param<bool>("mx_effort_use_current", mx_effort_use_current_, false);
    nh_->param<double>("diagnostics_rate", diagnostics_rate_, 0.0);

    nh_->param<std::string>("control_mode", mode, "Position");
    nh_->param<bool>("dynamic_mode_switching", dynamic_mode_switching_, false);
    nh_->param<double>("global_joint_speed", global_joint_speed, 5.0);
    nh_->param<double>("global_torque_limit", global_torque_limit, 1.0);
    nh_->param<double>("global_p_gain", global_p_gain, -1.0);
    nh_->param<double>("global_i_gain", global_i_gain, -1.0);
    nh_->param<double>("global_d_gain", global_d_gain, -1.0);

    // Set control mode for this run
    if (!strncmp(mode.c_str(), "Position", 8))
    {
        control_type_ = POSITION_CONTROL;
    }
    else if (!strncmp(mode.c_str(), "Velocity", 8))
    {
        control_type_ = VELOCITY_CONTROL;
    }
    else if (!strncmp(mode.c_str(), "Torque", 8))
    {
        control_type_ = TORQUE_CONTROL;
    }

    // Initialise write state variables
    write_ready_ = false;
    first_write_ = true;

    //Attempt to parse information for each device (port)
    int num_ports = 0;
    if (nh_->hasParam("ports"))
    {

        //PARSE ARRAY OF PORT INFORMATION

        XmlRpc::XmlRpcValue ports;
        nh_->getParam("ports", ports);
        //If there is no servos array in the param server, return
        if (!ports.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Invalid/missing device information on the param server");
            ROS_BREAK();
        }

        //number of ports defined
        num_ports = ports.size();

        //For every port, load and verify its information
        for (int i = 0; i < ports.size(); i++)
        {

            portInfo port;

            bool use_group_comms;

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

            //get series
            if (!ports[i]["series"].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("Invalid/Missing device name for port %d", i);
                ROS_BREAK();
            }
            else
            {
                port.series = static_cast<std::string>(ports[i]["series"]);
            }

            //get group comms enabled
            if (!ports[i]["group_comms_enabled"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
            {
                ROS_ERROR("Invalid/Missing group_comms_enabled option for port %d", i);
                ROS_BREAK();
            }
            else
            {
                use_group_comms = static_cast<bool>(ports[i]["group_comms_enabled"]);
            }

            /************************* Driver initialisation *********************/

            //Attempt to start driver
            try
            {
                port.driver = new dynamixel_interface_driver::DynamixelInterfaceDriver(port.device, port.baudrate, port.series, use_group_comms);    
            }
            catch (int n)
            {
                ROS_ERROR("Unable to start driver!, code %d", n);
                ROS_BREAK();
            }         

            /************************* Dynamixel initialisation *********************/

            int num_servos = 0;

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

            //number of servos defined
            num_servos = servos.size();

            //For every servo, load and verify its information
            for (int j = 0; j < servos.size(); j++)
            {
                //store the loaded information in this struct
                dynamixelInfo info;

                //check if info exists for specified joint
                if (!servos[j].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    ROS_ERROR("Invalid/Missing info-struct for servo index %d", i);
                    ROS_BREAK();
                }

                //get joint id
                if ( (!servos[j].hasMember("id")) || (!servos[j]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt) ) 
                {
                    ROS_ERROR("Invalid/Missing id for servo index %d", i);
                    ROS_BREAK();
                }
                else
                {
                    //store the servo's ID
                    info.id = static_cast<int>(servos[j]["id"]);
                }

                //get joint name
                if ( (!servos[j].hasMember("joint_name")) || 
                        (!servos[j]["joint_name"].getType() == XmlRpc::XmlRpcValue::TypeString) )
                {
                    ROS_ERROR("Invalid/Missing joint name for servo index %d, id: %d", i, info.id);
                    ROS_BREAK();
                }
                else
                {
                    //store the servo's corresponding joint name
                    info.joint_name = static_cast<std::string>(servos[j]["joint_name"]);

                    //check this port and all previous ports for duplicate joint names (not allowed as joints are
                    //referred to by name)
                    if ( port.joints.find(info.joint_name) !=  port.joints.end() )
                    {
                        ROS_ERROR("Cannot have multiple joints with the same name [%s]", info.joint_name.c_str());
                        ROS_BREAK();
                    }
                    else
                    {
                        for (int k = 0; k < dynamixel_ports_.size(); k++)
                        {
                            if ( dynamixel_ports_[k].joints.find(info.joint_name) !=  dynamixel_ports_[k].joints.end() )
                            {
                                ROS_ERROR("Cannot have multiple joints with the same name [%s]", 
                                        info.joint_name.c_str());
                                ROS_BREAK();
                            }
                        }
                    }
                }

                //get joint initial position
                if ( (!servos[j].hasMember("init")) || (!servos[j]["init"].getType() == XmlRpc::XmlRpcValue::TypeInt) )
                {
                    ROS_WARN("Invalid/Missing initial position for servo index %d, id: %d", i, info.id);
                    ROS_BREAK();
                }
                else
                {
                    //store the servo's corresponding joint 
                    info.init = static_cast<int>(servos[j]["init"]);
                }

                //get joint default min position
                if ( (!servos[j].hasMember("min")) || (!servos[j]["min"].getType() == XmlRpc::XmlRpcValue::TypeDouble) )
                {
                    ROS_WARN("Invalid/Missing min position for servo index %d, id: %d", i, info.id);
                    ROS_BREAK();
                }
                else
                {
                    info.min = static_cast<int>(servos[j]["min"]);
                }

                //get joint default max position
                if ( (!servos[j].hasMember("max")) || (servos[j]["max"].getType() == XmlRpc::XmlRpcValue::TypeDouble) )
                {
                    ROS_WARN("Invalid/Missing max position for servo index %d, id: %d", i, info.id);
                    ROS_BREAK();
                }
                else
                {
                    info.max = static_cast<int>(servos[j]["max"]);
                }

                //get joint default joint speed (or set to global if none specified)
                if ( (!servos[j].hasMember("joint_speed")) || 
                        (!servos[j]["joint_speed"].getType() == XmlRpc::XmlRpcValue::TypeDouble) )
                {
                    info.joint_speed = global_joint_speed;
                }
                else
                {
                    info.joint_speed = static_cast<double>(servos[j]["joint_speed"]);
                    if (info.joint_speed < 0.0)
                    {
                        info.joint_speed = global_joint_speed;
                    }
                }
               

                //get joint torque limit (or set to global if none specified)
                if ( (!servos[j].hasMember("torque_limit")) || 
                        (!servos[j]["torque_limit"].getType() == XmlRpc::XmlRpcValue::TypeDouble) )
                {
                    info.torque_limit = global_torque_limit;
                }
                else
                {
                    info.torque_limit = static_cast<double>(servos[j]["torque_limit"]);
                    if ((info.torque_limit > 1.0) || (info.torque_limit < 0.0))
                    {
                        info.torque_limit = global_torque_limit;
                    }
                }
               
                //get joint p_gain (or set to global if none specified)
                if ( (!servos[j].hasMember("p_gain")) || 
                        (!servos[j]["p_gain"].getType() == XmlRpc::XmlRpcValue::TypeDouble) )
                {
                    info.p_gain = global_p_gain;
                }
                else
                {
                    info.p_gain = static_cast<double>(servos[j]["p_gain"]);
                    if (info.p_gain < 0.0)
                    {
                        info.p_gain = global_p_gain;
                    }
                }

                //get joint i_gain (or set to global if none specified)
                if ( (!servos[j].hasMember("i_gain")) || 
                        (!servos[j]["i_gain"].getType() == XmlRpc::XmlRpcValue::TypeDouble) )
                {
                    info.i_gain = global_i_gain;
                }
                else
                {
                    info.i_gain = static_cast<double>(servos[j]["i_gain"]);
                    if (info.i_gain < 0.0)
                    {
                        info.i_gain = global_i_gain;
                    }
                }

                //get joint d_gain (or set to global if none specified)
                if ( (!servos[j].hasMember("d_gain")) || 
                        (!servos[j]["d_gain"].getType() == XmlRpc::XmlRpcValue::TypeDouble) )
                {
                    info.d_gain = global_d_gain;
                }
                else
                {
                    info.d_gain = static_cast<double>(servos[j]["d_gain"]);
                    if (info.d_gain < 0.0)
                    {
                        info.d_gain = global_d_gain;
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
                    info.model_number = 0;
                    bool t_e;
                    success &= port.driver->getModelNumber(info.id, &info.model_number);
                   
                    //If valid motor, setup in operating mode
                    if ((success) || (info.model_number))
                    {

                        //set up the lookup tables that we'll use later in the code
                        //to look up how to operate each joint
                        info.cpr = model_number2specs_[info.model_number].cpr;
                        info.gear_reduction = model_number2specs_[info.model_number].gear_reduction;
                        info.model_name = model_number2specs_[info.model_number].name;
                        info.torque_ratio = model_number2specs_[info.model_number].torque_ratio;
                        info.current_ratio = model_number2specs_[info.model_number].current_ratio;
                        info.torque_enabled = false;

                        //Display joint info
                        ROS_INFO("Joint Name: %s, ID: %d, Model: %s", info.joint_name.c_str(), info.id, 
                                info.model_name.c_str());
                        
                        //maintain torque state in motor
                        port.driver->getTorqueEnabled(info.id, &t_e);
                        port.driver->setTorqueEnabled(info.id, 0);
                     
                        //check support for operating mode
                        if ((control_type_ == TORQUE_CONTROL) && (info.model_number == 29))
                        {
                            ROS_ERROR("Torque Control mode not available with MX-28, skipping");
                            continue;
                        }

                        //check for valid motor series
                        if ( ((port.series == "MX") && (info.model_number > 320)) 
                                || ((port.series == "XM") && ((info.model_number < 350) || (info.model_number > 1020)))
                                || ((port.series == "PRO") && (info.model_number < 35072)) )
                        {
                            ROS_ERROR("Wrong series of dynamixel found, skipping");
                            continue;
                        }

                        //set operating mode for the motor
                        if ( !port.driver->setOperatingMode(info.id, control_type_) )
                        {
                            ROS_WARN("Failed to set operating mode for %s motor (id %d)", info.joint_name.c_str(), 
                                info.id);
                        }

                        //set torque limit for the motor
                        //ROS_INFO("%f %f %d", info.torque_limit, info.torque_ratio, 
                        //        (int) (info.torque_limit * info.torque_ratio));
                        if ( !port.driver->setMaxTorque(info.id, (int) (info.torque_limit * info.torque_ratio)) )
                        {
                            ROS_WARN("Failed to set torque limit for %s motor (id %d)", info.joint_name.c_str(), 
                                    info.id);
                        }

                        //angle limits are only relevant in position control mode
                        if (control_type_ == POSITION_CONTROL)
                        {
                            //set angle limits & direction
                            if (info.min > info. max) 
                            {
                                if ( !port.driver->setAngleLimits(info.id, info.max, info.min) )
                                {
                                    ROS_WARN("Failed to set angle limits for %s motor (id %d)", info.joint_name.c_str(), 
                                            info.id);
                                }
                            }
                            else
                            {
                                if ( !port.driver->setAngleLimits(info.id, info.min, info.max) )
                                {
                                    ROS_WARN("Failed to set angle limits for %s motor (id %d)", info.joint_name.c_str(), 
                                            info.id);
                                }                
                            }
                        }

                        //preserve torque enable state
                        port.driver->setTorqueEnabled(info.id, t_e);


                        //set PID tuning
                        if (!port.driver->setPIDGains(info.id, control_type_, info.p_gain, info.i_gain, info.d_gain))
                        {
                            ROS_WARN("Failed to set PID tuning for %s motor (id %d)", info.joint_name.c_str(), info.id);
                        }


                        info.current_mode == control_type_;

                        //add joint to port
                        port.joints[info.joint_name] = info;

                    }
                    else
                    {
                        ROS_ERROR("Failed to load model information for dynamixel id %d", info.id);
                        ROS_ERROR("Model Number: %d, Model Info: %d ", info.model_number, info.model_info);
                        if (model_number2specs_.find(info.model_number) != model_number2specs_.end())
                            ROS_ERROR("Info is in database");
                        else
                            ROS_ERROR("Info is not in database");
                        ROS_BREAK();
                    }
                }
                else
                {
                    ROS_ERROR("Cannot ping dynamixel id: %d", info.id);
                }

            }

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



    //advertise the sensor feedback topic 
    joint_state_publisher_  = nh_->advertise<sensor_msgs::JointState>("/joint_states", 1);

    //advertise the debug topic
    if (echo_joint_commands_)
    {
        debug_publisher_ = nh_->advertise<sensor_msgs::JointState>("/writeDebug", 1);
    }

    if (diagnostics_rate_ > 0)
    {
        diagnostics_publisher_ = nh_->advertise<dynamixel_interface_controller::ServoState>("/servo_diagnostics", 1);
    }

    //Start listening to command messages
    joint_state_subscriber_ = nh_->subscribe<sensor_msgs::JointState>("/desired_joint_state", 
        1, &DynamixelInterfaceController::jointStateCallback, this);
}


/** 
 * Destructor, deletes the objects holding the serial ports and disables the motors if required
 */
DynamixelInterfaceController::~DynamixelInterfaceController()
{
    
    ROS_INFO("shutting_down_ dynamixel_interface_controller");

    shutting_down_ = false;
    delete nh_;

    if (stop_motors_on_shutdown_)
    {
        for (int i = 0; i < dynamixel_ports_.size(); i++)
        {   
            //Turn off all the motors
            for (map<string, dynamixelInfo>::iterator iter = dynamixel_ports_[i].joints.begin(); 
                    iter != dynamixel_ports_[i].joints.end(); iter++)
            {  
                dynamixel_ports_[i].driver->setTorqueEnabled(iter->second.id, 0);
                printf("Torque disabled on %s joint\n", iter->first.c_str());
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
 * Start broadcasting JointState messages corresponding to the connected dynamixels 
 */
void DynamixelInterfaceController::startBroadcastingJointStates()
{
    
    broadcast_timer_ = nh_->createTimer(ros::Duration(1.0 / publish_rate_), 
            &DynamixelInterfaceController::publishJointStates, this);

    if (diagnostics_rate_ > 0)
    {
        diagnostics_timer_ = nh_->createTimer(ros::Duration(1.0 / diagnostics_rate_), 
                &DynamixelInterfaceController::diagnosticsRateCallback, this);       
    }
}




/** 
 * Callback for recieving a command from the /desired_joint_state topic.
 * The function atomically updates the class member variable containing the latest message and sets
 * a flag indicating a new message has been received
 * @param joint_commands the command received from the topic
 */
void DynamixelInterfaceController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_commands)
{
    
    std::unique_lock<std::mutex> lock(write_mutex_);
    write_msg_ = *joint_commands;
    write_ready_ = true;
    lock.unlock();
}


/**
 * TimeEvent callback for handling top level control of IO (for multiple ports).
 * Function spawns and waits on a thread for each 
 */
void DynamixelInterfaceController::diagnosticsRateCallback(const ros::TimerEvent& event)
{

    publish_diagnostics_ = true;

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

    std::unique_lock<std::mutex> lock(write_mutex_);

    //enable torque only once we start receiving commands
    if (write_ready_ && first_write_)
    {
        //loop over every port
        for (int i = 0; i < dynamixel_ports_.size(); i++)
        {
            //get every joint on that port
            std::map<std::string, dynamixelInfo>::iterator it;
            for(it = dynamixel_ports_[i].joints.begin(); it != dynamixel_ports_[i].joints.end(); it++)
            {
                
                dynamixelInfo info = it->second;
                int32_t priorPos = info.init;

                //this is to ensure that the motor is enabled at it's current position
                dynamixel_ports_[i].driver->getPosition(info.id, &priorPos);
                dynamixel_ports_[i].driver->setPosition(info.id, priorPos);

                //enable motor torque
                if (!dynamixel_ports_[i].driver->setTorqueEnabled(info.id, 1))
                {
                    ROS_ERROR("failed to enable torque on motor %d", info.id);
                }

                //if in position control mode we enable the default join movement speed (profile velocity)
                if (control_type_ == POSITION_CONTROL)
                {
                    int regVal = (int) ((double) (info.joint_speed) * (60/(2.0 * M_PI)) * info.gear_reduction);
                    dynamixel_ports_[i].driver->setProfileVelocity(info.id, regVal);
                } 
                else if (control_type_ == TORQUE_CONTROL)
                {
                    dynamixel_ports_[i].driver->setOperatingMode(info.id, control_type_);
                }
                
                ROS_INFO("Torque enabled on %s joint", it->first.c_str());
                info.torque_enabled = true;

            }
        }

        first_write_ = false;
    }

    //spawn an IO thread for each additional port
    for (int i = 1; i < dynamixel_ports_.size(); i++)
    {
        num_servos = num_servos + dynamixel_ports_[i].joints.size();
        reads[i] = sensor_msgs::JointState();
        std::thread readThread(&DynamixelInterfaceController::multiThreadedIO, this, i, std::ref(reads[i]), write_ready_);
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
        if (echo_joint_commands_)
        {
            debug_publisher_.publish(temp_msg);
        }
        multiThreadedWrite(0, temp_msg);
    }

    //perform read
    multiThreadedRead(0, reads[0]);


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

    }

    //timestamp
    read_msg.header.stamp = ros::Time::now();

    //reset write flag
    if (write_ready_)
    {
        write_ready_ = false;

    } 
    lock.unlock();

    //publish joint states
    joint_state_publisher_.publish(read_msg);


    publish_diagnostics_ = false;

}

/**
 * Top level control function for each port's IO thread.
 * @param port_num index used to retrieve port information from port list
 * @param read_msg the msg this threads join data is read into, this is then combined by the top level function.
 * @param perform_write boolean indicating whether or not to write latest joint_state to servos
 */
void DynamixelInterfaceController::multiThreadedIO(int port_num, sensor_msgs::JointState &read_msg, bool perform_write)
{

    sensor_msgs::JointState thread_write_msg = write_msg_;
    
    //perform write
    if (write_ready_)
    {
        multiThreadedWrite(port_num, thread_write_msg);
    }

    //perform read
    multiThreadedRead(port_num, read_msg);

}


/**
 * Function called in each thread to perform a write on a port
 * @param port_num index used to retrieve port information from port list
 * @param joint_commands message cointaining the commands for each joint
 */
void DynamixelInterfaceController::multiThreadedWrite(int port_num, sensor_msgs::JointState joint_commands)
{

    //get this threads port information
    portInfo port = dynamixel_ports_[port_num];

    //ignore empty messages
    if (joint_commands.name.size() < 1)
    {
        return;
    }

    //booleans used to setup which parameters to write
    bool has_pos = false, has_vel = false, has_torque = false;

    //figure out which values have been specified
    if ((joint_commands.position.size() == joint_commands.name.size()) && (control_type_ == POSITION_CONTROL))
    {
        has_pos = true;
    }
    if ((joint_commands.velocity.size() == joint_commands.name.size()) && (control_type_ != TORQUE_CONTROL))
    {
        has_vel = true;
    }
    if ((joint_commands.effort.size() == joint_commands.name.size()) && ((control_type_ == TORQUE_CONTROL) ||
            ((control_type_ == POSITION_CONTROL) && dynamic_mode_switching_)) )
    {
        has_torque = true; 
    }

    //vectors to store the calculated values
    vector<int> ids, velocities, positions, torques, modes;

    //loop and calculate the values for each specified joint
    for (int i = 0; i < joint_commands.name.size(); i++)
    {

        //lookup the information for that particular joint to be able to control it
        std::map<std::string, dynamixelInfo>::iterator joint_it;
        if ((joint_it = port.joints.find(joint_commands.name[i])) == port.joints.end()) {
            //Joint not on this port, ignore
            continue;
        }

        //Retrieve dynamixel information
        dynamixelInfo info = joint_it->second;

        //prepare data to be sent to the motor
        ids.push_back(info.id);

        //calculate the position register value for the motor
        if (has_pos)
        {

            //used to bound positions to limits
            int up_lim, dn_lim;

            //get radian position value
            double rad_pos = joint_commands.position[i];
            
            //define our clamping limits to avoid exceeding safe joint angles
            if (info.min > info.max)
            {
                rad_pos = -rad_pos;
                up_lim = info.min;
                dn_lim = info.max;
            }
            else
            {
                up_lim = info.max;
                dn_lim = info.min;
            }

            //convert from radians to motor encoder value
            int pos = (int)(rad_pos / 2.0 / M_PI * info.cpr + 0.5) + info.init;


            //clamp joint angle to be within safe limit
            if (pos > up_lim)  
            {
                pos = up_lim;
            }
            else if (pos < dn_lim)
            {
                pos = dn_lim;
            }

            //push motor encoder value onto list
            positions.push_back(pos);

        }

        //calculate the velocity register value for the motor
        if (has_vel)
        {

            //get rad/s value from message
            double rad_s_vel = joint_commands.velocity[i];

            //clamp to joint speed limit
            if (abs(rad_s_vel) > info.joint_speed) {

                rad_s_vel = (rad_s_vel < 0) ? (-info.joint_speed) : (info.joint_speed);

            }

            //convert to motor encoder value
            int vel = (int) ((rad_s_vel * (60/(2.0 * M_PI)) * info.gear_reduction));

            //Velocity values serve 2 different functions, in velocity control mode their sign
            //defines direction, however in position control mode their absolute value is used
            //to set the profile velocity (how fast the motor moves to the specified position)
            vel = abs(vel);

            //we also need to take an absolute value as each motor series handles negative inputs
            //differently
            if ((control_type_ == VELOCITY_CONTROL) && ((rad_s_vel < 0) != (info.min > info.max)))
            {

                if (port.series == "MX")
                {
                    vel = vel + 1024;
                }

                else
                {
                    vel = -vel;
                }

            }

            //push motor velocity value onto list
            velocities.push_back(vel);
        }


        //calculate torque register values for the motor
        if (has_torque)
        {
            //replace the below with proper code when you can figure out the units
            double input_torque = joint_commands.effort[i];

            int torque = 0;

            if (use_torque_as_effort_)
            {
                if (info.current_ratio != 0)
                {
                    torque = (int) (input_torque * info.current_ratio);
                    torque = abs(torque);

                    if ((input_torque < 0) != (info.min > info.max))
                    {
                        if (port.series == "MX")
                        {
                            torque = 1024 + torque;
                        }
                        else
                        {
                            torque = -torque;
                        }
                    }
                }
            }
            else
            {
                if (info.torque_ratio != 0)
                {
                    torque = (int) (input_torque * info.torque_ratio);
                    torque = abs(torque);

                    if ((input_torque < 0) != (info.min > info.max))
                    {
                        if (port.series == "MX")
                        {
                            torque = 1024 + torque;
                        }
                        else
                        {
                            torque = -torque;
                        }
                    }
                }
            }

            torques.push_back(torque);

            if (dynamic_mode_switching_)
            {
                //update control mode for motor if relevant
                if ((control_type_ == POSITION_CONTROL) && (torque != 0))
                {
                    modes.push_back(1);
                }
                else if ((control_type_ == POSITION_CONTROL) && (torque == 0))
                {
                    modes.push_back(0);
                }
            }

        }

        
    }

    //use the multi-motor write functions to reduce the bandwidth required to command
    //all the motors

    if ( control_type_ == POSITION_CONTROL )
    {

        if (has_torque && dynamic_mode_switching_)
        {


            //set the torque values for each motor
            vector< vector<int> > data;
            for (int i = 0; i < ids.size(); i++)
            {
                vector<int> temp;
                temp.push_back(ids[i]);
                temp.push_back(torques[i]);
                data.push_back(temp);
            }
            port.driver->setMultiTorque(data);


            //set torque control mode enable for each motor
            data.clear();
            
            for (int i = 0; i < ids.size(); i++)
            {
                vector<int> temp;
                temp.push_back(ids[i]);
                temp.push_back(modes[i]);
                data.push_back(temp);
            }
            port.driver->setMultiTorqueControl(data);

        }

        //set the profile velocities if they have been defined
        if (has_vel)
        {
            vector< vector<int> > data;
            for (int i = 0; i < ids.size(); i++)
            {
                vector<int> temp;
                temp.push_back(ids[i]);
                temp.push_back(velocities[i]);
                data.push_back(temp);
            }
            port.driver->setMultiProfileVelocity(data);
        }

        //send the positions to the motors
        if (has_pos)
        {
            vector< vector<int> > data;
            for (int i = 0; i < ids.size(); i++)
            {
                vector<int> temp;
                temp.push_back(ids[i]);
                temp.push_back(positions[i]);
                data.push_back(temp);
            }
            port.driver->setMultiPosition(data);
        }

    }
    else if ( control_type_ == VELOCITY_CONTROL && has_vel)
    {

        //set the velocity values for each motor
        vector< vector<int> > data;
        for (int i = 0; i < ids.size(); i++)
        {
            vector<int> temp;
            temp.push_back(ids[i]);
            temp.push_back(velocities[i]);
            data.push_back(temp);
        }
        port.driver->setMultiVelocity(data);
    }
    else if ( (control_type_ == TORQUE_CONTROL) && has_torque)
    {

        //set the torque values for each motor
        vector< vector<int> > data;
        for (int i = 0; i < ids.size(); i++)
        {
            vector<int> temp;
            temp.push_back(ids[i]);
            temp.push_back(torques[i]);
            data.push_back(temp);
        }
        port.driver->setMultiTorque(data);
    }

}


/**
 * Function in thread to perform read on a port.
 * @param port_num index used to retrieve port information from port list
 * @param read_msg the msg this ports join data is read into.
 */
void DynamixelInterfaceController::multiThreadedRead(int port_num, sensor_msgs::JointState &read_msg)
{

    bool comm_success;
    portInfo port = dynamixel_ports_[port_num];
    std::vector<int> *servo_ids = new std::vector<int>;
    std::map<int, std::vector<int32_t> >  *responses = new std::map<int, std::vector<int32_t> >;


    //Iterate over all connected servos and add to list
    for (map<string, dynamixelInfo>::iterator iter = port.joints.begin(); iter != port.joints.end(); iter++)
    {
        servo_ids->push_back(iter->second.id);
    }

    //get state info back from all dynamixels
    if( port.driver->getBulkStateInfo(servo_ids, responses, mx_effort_use_current_) ) {
        
        //Iterate over all connected servos and add to list
        for (map<string, dynamixelInfo>::iterator iter = port.joints.begin(); iter != port.joints.end(); iter++)
        {
            //joint name
            string joint_name = iter->first;
            
            //info struct
            dynamixelInfo info = iter->second;

            //ignore joints that failed to read
            if(std::find(servo_ids->begin(), servo_ids->end(), info.id) == servo_ids->end())
            {
                continue;
            } 

            //response from dynamixel
            std::vector<int32_t> response = responses->at(info.id);
            
            //put joint name in message
            read_msg.name.push_back(joint_name);
            
            //POSITION VALUE
            //get position and convert to radians
            double rad_pos = (response[0] - info.init) / ((double) (info.cpr)) * 2 * M_PI;
            if (info.min > info.max)
            {
                rad_pos = -rad_pos;
            }

            //Put position in message
            read_msg.position.push_back(rad_pos);

            
            //VELOCITY VALUE
            int raw_vel = response[1];

            //handle the sign of the value based on the motor series
            if (port.series == "MX")
            {
                raw_vel = (response[1] & 0x3FF);

                if ((response[1] > 1023) && (info.max > info.min))
                {
                    raw_vel = -raw_vel;
                }
                else if ((response[1] < 1023) && (info.max < info.min))
                {
                    raw_vel = -raw_vel;
                }
            }
            else if (info.min > info.max)
            {
                raw_vel = -raw_vel;
            }
        
            //convert to rad/s
            double rad_s_vel = ((double) raw_vel) * ((2.0 * M_PI) / 60.0) / info.gear_reduction;

            //put velocity in message
            read_msg.velocity.push_back(rad_s_vel);


            //TORQUE EFFORT VALUE
            //convert raw value to fraction of max torque
            int raw_torque = response[2];
            double torque = 0;

            
            if (use_torque_as_effort_)
            {
                if (port.series == "MX")
                {
                    if (mx_effort_use_current_)
                    {
                        torque = (double) (raw_torque - 2048) / info.current_ratio;
                    }
                    else
                    {
                        torque = ((double) (response[2] & 0x3FF)) / info.current_ratio;
                        //check sign 
                        if (response[2] < 1023)
                        {
                            torque = 0.0 - torque;
                        }
                    }
                }
                else
                {
                    torque = ((double) (response[2]) / info.current_ratio);
                }  
            }
            else
            {
                if (port.series == "MX")
                {
                    if (mx_effort_use_current_)
                    {
                        torque = (double) (raw_torque - 2048) / info.torque_ratio;
                    }
                    else
                    {
                        torque = ((double) (response[2] & 0x3FF)) / info.torque_ratio;
                        //check sign 
                        if (response[2] < 1023)
                        {
                            torque = 0.0 - torque;
                        }
                    }
                }
                else
                {
                    torque = ((double) (response[2]) / info.torque_ratio);
                }             
            }

            if (info.min > info.max)
            {
                torque = -torque;
            }

            //put effort in message
            read_msg.effort.push_back(torque);
        }

        responses->clear();

        if (publish_diagnostics_)
        {
            if( port.driver->getBulkDiagnosticInfo(servo_ids, responses) )
            {

                dynamixel_interface_controller::ServoState diag_msg;

                //Iterate over all connected servos and add to list
                for (map<string, dynamixelInfo>::iterator iter = port.joints.begin(); iter != port.joints.end(); iter++)
                {
                    //joint name
                    string joint_name = iter->first;
                    
                    //info struct
                    dynamixelInfo info = iter->second;

                    //ignore joints that failed to read
                    if(std::find(servo_ids->begin(), servo_ids->end(), info.id) == servo_ids->end())
                    {
                        continue;
                    } 

                    //response from dynamixel
                    std::vector<int32_t> response = responses->at(info.id);

                    //put effort in message
                    diag_msg.joint_names.push_back(joint_name.c_str());

                    //get voltage
                    diag_msg.voltages.push_back( (double)(response[0]) / 10);

                    //get temperatures
                    diag_msg.temperatures.push_back( (double)(response[1]));

                    diag_msg.error_states.push_back(response[2]);

                }

                //publish diagnostic info
                diagnostics_publisher_.publish(diag_msg);

            }
        }

    }
    else
    {
        ROS_ERROR("READ FAILURE, UNABLE TO GET JOINT STATES ON PORT %s", port.device.c_str());
    }

    read_msg.header.stamp = ros::Time::now();

}



/** Main loop. intialises controller and starts timer and ROS callback handling */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "dynamixel_interface_controller");
 
  DynamixelInterfaceController controller;
  
  //Initialize Timer callback, this starts the IO
  controller.startBroadcastingJointStates();

  //Use a multi threaded spinner to handle the timer based IO callback and
  //write topic subscriber callbacks simultaneously
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

}


