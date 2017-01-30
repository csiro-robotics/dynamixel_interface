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
 * dynamixel_interface_driver and dynamixel_interface_controller packages are adapted from software provided by Brian Axelrod (on behalf of 
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
 * @file   dynamixel_const.h
 * @author Tom Molnar (Tom.Molnar@data61.csiro.au), Brian Axelrod
 * @date   December, 2016
 * @brief  Defines the register address tables for each series of dynamixel,
 * as well as the control and status codes for communication.
 */


#ifndef DYNAMIXEL_CONST_H__
#define DYNAMIXEL_CONST_H__

#include <stdint.h>
#include <string>


namespace dynamixel_interface_driver
{

/**
 * Control table/register addresses for each series of dynamixel
 */
typedef enum DynamixelControlEnum
{

    /**
     * Pro Series values
     */ 
    DXL_PRO_MODEL_NUMBER = 0,
    DXL_PRO_MODEL_INFO=2,
    DXL_PRO_FIRMWARE_VERSION = 6,
    DXL_PRO_ID = 7,
    DXL_PRO_BAUD_RATE = 8,
    DXL_PRO_RETURN_DELAY_TIME = 9,
    DXL_PRO_OPERATING_MODE = 11,
    DXL_PRO_LIMIT_TEMPERATURE = 21,
    DXL_PRO_DOWN_LIMIT_VOLTAGE = 24,
    DXL_PRO_UP_LIMIT_VOLTAGE = 22,
    DXL_PRO_ACCEL_LIMIT = 26,
    DXL_PRO_VELOCITY_LIMIT = 32,
    DXL_PRO_MAX_TORQUE = 30,
    DXL_PRO_MAX_ANGLE_LIMIT = 36,
    DXL_PRO_MIN_ANGLE_LIMIT = 40,
    DXL_PRO_VELOCITY_I_GAIN = 586,
    DXL_PRO_VELOCITY_P_GAIN = 588,
    DXL_PRO_POSITION_P_GAIN = 594,
    DXL_PRO_TORQUE_ENABLE = 562,
    DXL_PRO_LED = 25,
    DXL_PRO_CW_COMPLIANCE_MARGIN = 26,
    DXL_PRO_CCW_COMPLIANCE_MARGIN = 27,
    DXL_PRO_CW_COMPLIANCE_SLOPE = 28,
    DXL_PRO_CCW_COMPLIANCE_SLOPE = 29,
    DXL_PRO_GOAL_POSITION = 596,
    DXL_PRO_GOAL_VELOCITY = 600,
    DXL_PRO_GOAL_TORQUE = 604,
    DXL_PRO_PRESENT_POSITION = 611,
    DXL_PRO_PRESENT_VELOCTY = 615,
    DXL_PRO_PRESENT_CURRENT = 621,
    DXL_PRO_PRESENT_VOLTAGE = 623,
    DXL_PRO_PRESENT_TEMPERATURE = 625,
    DXL_PRO_REGISTERED_INSTRUCTION = 890,
    DXL_PRO_MOVING = 610,
    DXL_PRO_RETURN_LEVEL = 891,

    /** 
     * XM Series values
     */
    DXL_X_MODEL_NUMBER = 0,
    DXL_X_MODEL_INFO = 2,
    DXL_X_FIRMWARE_VERSION = 6,
    DXL_X_ID = 7,
    DXL_X_BAUD_RATE = 8,
    DXL_X_RETURN_DELAY_TIME = 9,
    DXL_X_DRIVE_MODE = 10,
    DXL_X_OPERATING_MODE = 11,
    DXL_X_PROTOCOL_VERSION = 13,
    DXL_X_HOMING_OFFSET = 20,
    DXL_X_MOVING_THRESHOLD = 24,
    DXL_X_TEMPERATURE_LIMIT = 31,
    DXL_X_MAX_VOLTAGE_LIMIT = 32,
    DXL_X_MIN_VOLTAGE_LIMIT = 34,
    DXL_X_PWM_LIMIT = 36,
    DXL_X_CURRENT_LIMIT = 38,
    DXL_X_ACCELERATION_LIMIT = 40,
    DXL_X_VELOCITY_LIMIT = 44,
    DXL_X_MAX_POSITION_LIMIT = 48,
    DXL_X_MIN_POSITION_LIMIT = 52,
    DXL_X_SHUTDOWN = 63,
    DXL_X_TORQUE_ENABLE = 64,
    DXL_X_LED = 65,
    DXL_X_STATUS_RETURN_LEVEL = 68,
    DXL_X_REGISTERED_INSTRUCTION = 69,
    DXL_X_HARDWARE_ERROR_STATUS = 70,
    DXL_X_VELOCITY_I_GAIN = 76,
    DXL_X_VELOCITY_P_GAIN = 78,
    DXL_X_POSITION_D_GAIN = 80,
    DXL_X_POSITION_I_GAIN = 82,
    DXL_X_POSITION_P_GAIN = 84,
    DXL_X_FEEDFORWARD_2ND_GAIN = 88,
    DXL_X_FEEDFORWARD_1ST_GAIN = 90,
    DXL_X_BUS_WATCHDOG = 98,
    DXL_X_GOAL_PWM = 100,
    DXL_X_GOAL_CURRENT = 102,
    DXL_X_GOAL_VELOCITY = 104,
    DXL_X_PROFILE_ACCELERATION = 108,
    DXL_X_PROFILE_VELOCITY = 112,
    DXL_X_GOAL_POSITION = 116,
    DXL_X_REALTIME_TICK = 120,
    DXL_X_MOVING = 122,
    DXL_X_MOVING_STATUS = 123,
    DXL_X_PRESENT_PWM = 124,
    DXL_X_PRESENT_CURRENT = 126,
    DXL_X_PRESENT_VELOCITY = 128,
    DXL_X_PRESENT_POSITION = 132,
    DXL_X_VELOCITY_TRAJECTORY = 136,
    DXL_X_POSITION_TRAJECTORY = 140,
    DXL_X_PRESENT_INPUT_VOLTAGE = 144,
    DXL_X_PRESENT_TEMPERATURE = 146,

    /**
     * MX Series Values
     */
    DXL_MX_MODEL_NUMBER = 0,
    DXL_MX_FIRMWARE_VERSION = 2,
    DXL_MX_ID = 3,
    DXL_MX_BAUD_RATE = 4,
    DXL_MX_RETURN_DELAY_TIME = 5,
    DXL_MX_CW_ANGLE_LIMIT = 6,
    DXL_MX_CCW_ANGLE_LIMIT = 8,
    DXL_MX_DRIVE_MODE = 10,
    DXL_MX_LIMIT_TEMPERATURE = 11,
    DXL_MX_DOWN_LIMIT_VOLTAGE = 12,
    DXL_MX_UP_LIMIT_VOLTAGE = 13,
    DXL_MX_MAX_TORQUE = 14,
    DXL_MX_RETURN_LEVEL = 16,
    DXL_MX_ALARM_LED = 17,
    DXL_MX_ALARM_SHUTDOWN = 18,
    DXL_MX_MULTI_TURN_OFFSET_L = 20,
    DXL_MX_MULTI_TURN_OFFSET_H = 21,
    DXL_MX_RESOLUTION_DIVIDER = 22,
    DXL_MX_TORQUE_ENABLE = 24,
    DXL_MX_LED = 25,
    DXL_MX_D_GAIN = 26,
    DXL_MX_I_GAIN = 27,
    DXL_MX_P_GAIN = 28,
    DXL_MX_GOAL_POSITION_L = 30,
    DXL_MX_GOAL_POSITION_H = 31,
    DXL_MX_GOAL_SPEED_L = 32,
    DXL_MX_GOAL_SPEED_H = 33,
    DXL_MX_TORQUE_LIMIT_L = 34,
    DXL_MX_TORQUE_LIMIT_H = 35,
    DXL_MX_PRESENT_POSITION_L = 36,
    DXL_MX_PRESENT_POSITION_H = 37,
    DXL_MX_PRESENT_SPEED_L = 38,
    DXL_MX_PRESENT_SPEED_H = 39,
    DXL_MX_PRESENT_LOAD_L = 40,
    DXL_MX_PRESENT_LOAD_H = 41,
    DXL_MX_PRESENT_VOLTAGE = 42,
    DXL_MX_PRESENT_TEMPERATURE = 43,
    DXL_MX_REGISTERED_INSTRUCTION = 44,
    DXL_MX_MOVING = 46,
    DXL_MX_LOCK = 47,
    DXL_MX_PUNCH_L = 48,
    DXL_MX_PUNCH_H = 49,
    DXL_MX_PRESENT_CURRENT_L = 68,
    DXL_MX_PRESENT_CURRENT_H = 69,
    DXL_MX_TORQUE_CONTROL_ENABLE = 70,
    DXL_MX_GOAL_TORQUE_L = 71,
    DXL_MX_GOAL_TORQUE_H = 72,
    DXL_MX_GOAL_ACCELERATION = 73,

} DynamixelControl;


/**
 * Instruction codes for various commands
 */
typedef enum DynamixelInstructionEnum
{
    DXL_PING = 1,
    DXL_READ_DATA = 2,
    DXL_WRITE_DATA = 3,
    DXL_REG_WRITE = 4,
    DXL_ACTION = 5,
    DXL_RESET = 6,
    DXL_SYNC_WRITE = 0x83,
    DXL_BULK_READ = 0x92,
    DXL_BROADCAST = 254,

} DynamixelInstruction;

/**
 * Error return codes
 */
typedef enum DynamixelErrorCodeEnum
{
    DXL_PRO_UNDOCUMENTED_ERROR2 = 64,
    DXL_PRO_UNDOCUMENTED_ERROR1 = 32,
    DXL_PRO_ELECTRIC_SHOCK_ERROR = 16,
    DXL_PRO_ENCODER_ERROR = 8,
    DXL_PRO_OVERHEATING_ERROR = 4,
    DXL_PRO_HALL_ERROR = 2,
    DXL_PRO_INPUT_VOLTAGE_ERROR = 1,
    DXL_PRO_NO_ERROR = 0,

    DXL_INSTRUCTION_ERROR = 64,
    DXL_OVERLOAD_ERROR = 32,
    DXL_CHECKSUM_ERROR = 16,
    DXL_RANGE_ERROR = 8,
    DXL_OVERHEATING_ERROR = 4,
    DXL_ANGLE_LIMIT_ERROR = 2,
    DXL_INPUT_VOLTAGE_ERROR = 1,
    DXL_NO_ERROR = 0,

} DynamixelErrorCode;

}

#endif  // DYNAMIXEL_CONST_H__
