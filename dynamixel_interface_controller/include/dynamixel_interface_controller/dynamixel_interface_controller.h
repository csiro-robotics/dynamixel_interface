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
 * dynamixel_driver and dynamixel_interface_controller packages are adapted from software provided by Brian Axelrod (on behalf of 
 * Willow Garage):
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

#include <string>
#include <map>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <dynamixel_interface_driver/dynamixel_interface_driver.h>


namespace dynamixel_interface_controller
{


/**
 * Struct that describes each servo's place in the system including 
 * which joint it corresponds to. 
 */
struct dynamixelInfo 
{

    int id; /**< The unique (per port) ID of the motor */ 

    /** The unique (globally) name of the joint */ 
    std::string joint_name; 

    /** Motor default joint speed (rad/s) */ 
    double joint_speed; 

    /** Motor maximum torque limit (%rated max) */ 
    double torque_limit; 

    /** Proportional gain value */ 
    double p_gain; 

    /** Integral gain value */ 
    double i_gain; 

    /** Differential gain value */ 
    double d_gain; 

    /**
     * Motor initial position (in raw encoder values).
     * This value defines the 0 radian position for the motor
     */ 
    int init; 

    /**
     * Motor minimum encoder value.
     * Note that if min > max, the motor direction is reversed 
     */ 
    int min;

    /**
     * Motor maximum encoder value.
     * Note that if min > max, the motor direction is reversed 
     */ 
    int max; 

    /** Motor model name */ 
    std::string model_name;

    /** Motor model number */  
    uint16_t model_number;

    /** Motor model info */ 
    uint32_t model_info; 
    
    /** Motor encoder Counts Per Revolution */ 
    int cpr;

    /** Motor rad/s to register value ratio */ 
    double gear_reduction; 

    /** Motor torque reading to register value ratio */ 
    double torque_ratio; 

    /** Motor enable flag */
    bool torque_enabled; 


};

/** 
 * Struct which stores information about each port in use and which
 * joints use that port
 */
struct portInfo
{

    /** User defined port name */
    std::string port_name;

    /** Serial Configuration */
    std::string device;
    int baudrate;

    /** Which series of motor is on the port */
    std::string series;

    /** Pointer to the serial driver */
    dynamixel_interface_driver::DynamixelInterfaceDriver *driver;

    /** map of joint names to information */
    std::map<std::string, dynamixelInfo> joints;
};

/**
 * Struct that describes the dynamixel motor's static and physical
 * properties 
 */
struct dynamixelSpec
{

    /** The Model Name */
    std::string name;

    /** Model number (e.g 29 = MX-28) */
    uint16_t model_number;

    /** Motor encoder counter per revolution */
    int cpr;

    /** Gear reduction ratio */
    double gear_reduction;

    /** Torque ratio */
    double torque_ratio;
    
};

/**
 * The different control modes available on the dynamixel servos. 
 * The values chosen for each type reflect those used on the motors 
 * themselves.
 */
enum controlMode
{
    POSITION_CONTROL = 3,
    VELOCITY_CONTROL = 1,
    TORQUE_CONTROL = 0,
    UNKOWN  = -1
};



/**
 * This class forms a ROS Node that provides an interface with the dynamixel series of servos.
 * The controller functions on a timer based callback for updating the motor states. Commands are
 * Continuously sent to the motor and updated via callback once new messages are published to the command
 * Topic. This class also provides a threaded interface for controlling multiple sets of dynamixels simultaneously
 * and synchronously through different serial ports. This allows robots with many motors to reduce the overall IO
 * time required for control.
 */
class DynamixelInterfaceController
{
public:

    /** 
     * Constructor, loads the motor configuration information from the specified yaml file and intialises 
     * The motor states.
     */
    DynamixelInterfaceController();

    /** 
     * Destructor, deletes the objects holding the serial ports and disables the motors if required
     */
    ~DynamixelInterfaceController();


    /**
     * Start broadcasting JointState messages corresponding to the connected dynamixels 
     */
    void startBroadcastingJointStates();

private:

    /** 
     * Callback for recieving a command from the /desired_joint_state topic.
     * The function atomically updates the class member variable containing the latest message and sets
     * a flag indicating a new message has been received
     * @param joint_commands the command received from the topic
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_commands);

    /**
     * TimeEvent callback for handling top level control of IO (for multiple ports).
     * Function spawns and waits on a thread for each additional port defined. In cases where no additional ports are
     * 
     */
    void publishJointStatesThreaded(const ros::TimerEvent& event);

    /**
     * Top level control function for each port's IO thread.
     * @param port_num index used to retrieve port information from port list
     * @param read_msg the msg this threads join data is read into, this is then combined by the top level function.
     */
    void multiThreadedIO(int port_num, sensor_msgs::JointState &read_msg);

    /**
     * Function called in each thread to perform a write on a port
     * @param port_num index used to retrieve port information from port list
     * @param joint_commands message cointaining the commands for each joint
     */
    void multiThreadedWrite(int port_num, sensor_msgs::JointState joint_commands);
    

    /**
     * Function in thread to perform read on a port.
     * @param port_num index used to retrieve port information from port list
     * @param read_msg the msg this ports join data is read into.
     */
    void multiThreadedRead(int port_num, sensor_msgs::JointState &read_msg);


    /** Handler for the ROS Node */
    ros::NodeHandle *nh_;

    /** Rate at which joint state information is published */
    double publish_rate_;     

    /** Indicates to callbacks that the controller is shutting down */
    volatile bool shutting_down_;

    /** Indicates if the motors should be turned off when the controller stops */
    bool stop_motors_on_shutdown_;

    /** Can echo commands sent to the motors (useful for monitoring write values/rates) */
    bool echo_joint_commands_;

    /** Stores the last message received from the write command topic */
    sensor_msgs::JointState write_msg_;

    /** Mutex for write_msg, as there are potentially multiple threads */
    std::mutex write_mutex_;

    /** Booleans indicating if we have received commands */
    bool write_ready_;
    bool first_write_;

    /** method of control (position/velocity/torque) */
    controlMode control_type_;   

    /** map of model numbers to motor specifications */
    std::map<uint16_t, dynamixelSpec> model_number2specs_;

    /** the vector of all ports in use */
    std::vector<portInfo> dynamixel_ports_;

    /** Publishes joint states from reads */
    ros::Publisher joint_state_publisher_;

    /** Gets joint states for writes */
    ros::Subscriber joint_state_subscriber_;

    /** Timer that controls the rate of the IO callback */
    ros::Timer broadcast_timer_;

    /** Debug message publisher */
    ros::Publisher debug_publisher_;

};

}

#endif //DYNAMIXEL_INTERFACE_CONTROLLER_H_
