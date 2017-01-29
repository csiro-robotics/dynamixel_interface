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
 * dynamixel_driver and dynamixel_controller packages are adapted from software provided by Brian Axelrod (on behalf of 
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
 * @file   dynamixel_driver.cpp
 * @author Tom Molnar (Tom.Molnar@data61.csiro.au), Brian Axelrod
 * @date   January, 2017
 * @brief  Implements the hardware abstraction for communicating with dynamixels
 */


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <dynamixel_driver/dynamixel_const.h>
#include <dynamixel_driver/dynamixel_driver.h>


//Macros for combining bytes

#define LOBYTE(w) ((uint8_t)(w))
#define HIBYTE(w) ((uint8_t)(((uint16_t)(w) >> 8) & 0xFF))
#define MAKEWORD(low, high) \
((uint16_t)((((uint16_t)(high)) << 8) | ((uint8_t)(low))))
#define MAKEINT(lowWord, highWord) \
((uint32_t)((((uint32_t)(highWord)) << 16) | ((uint32_t)(lowWord))))

using namespace std;

/*******************  IMPORTANT This code was written for little-endian cpus (forex intel)   ****************/
namespace dynamixel_driver
{

/**
 * Constructor. Initialises port and packet handling objects and sets the 
 * baud rate.
 * @param device  The serial port to connect to
 * @param baud    The baud rate to use
 * @param series  The servo series in use (MX, XM or Pro)  
 */
DynamixelDriver::DynamixelDriver(std::string device="/dev/ttyUSB0",
                         int baud=1000000, std::string series="MX")
{


    ROS_INFO("Device is '%s' with baud rate '%d', servos are of series '%s'",device.c_str(), baud, series.c_str());

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux
    portHandler_ = dynamixel::PortHandler::getPortHandler(device.c_str());

    if (!strncmp(series.c_str(), "MX", 2))
    {
    	servo_series_ = 'M';
    }
    else if (!strncmp(series.c_str(), "XM", 2))
    {
    	servo_series_ = 'X';
    }
    else if (!strncmp(series.c_str(), "PRO", 3))
	{
		servo_series_ = 'P';
	}
	else
	{
		ROS_ERROR("Invlaid Motor Series: %s, Valid series: 'MX', 'XM, 'PRO'", series.c_str());
		throw 1;
	}


    // Initialize PacketHandler instances for each protocol version
    packetHandlerP1_ = dynamixel::PacketHandler::getPacketHandler(1.0);
    packetHandlerP2_ = dynamixel::PacketHandler::getPacketHandler(2.0);

	// Open port
    if (portHandler_->openPort())
    {
        ROS_INFO("Succeeded to open the port(%s)!", device.c_str());
    }
    else
    {
        ROS_ERROR("Failed to open the port!");
        throw 2;
    }

    // Set port baudrate
    if (portHandler_->setBaudRate(baud))
    {
        ROS_INFO("Succeeded to change the baudrate(%d)\n!", portHandler_->getBaudRate());
    }
    else
    {
        ROS_ERROR("Failed to change the baudrate!");
        throw 3;
    }


}

/**
 * Destructor. Closes and releases serial port.
 */
DynamixelDriver::~DynamixelDriver()
{
    portHandler_->closePort();
    delete portHandler_;
    delete packetHandlerP1_;
    delete packetHandlerP2_;
}

/**
 * Ping the specified id, used to check if a dynamixel with that ID is on the bus
 * @param servo_id The ID to ping on the bus.
 * @return True if a dynamixel responds, false otherwise.
 */
bool DynamixelDriver::ping(int servo_id)
{
	uint8_t error;
	int dxl_comm_result;

	//ping dynamixel on bus
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->ping(portHandler_, servo_id, &error);
	}
	else if ((servo_series_ == 'X') || (servo_series_ == 'P'))
	{
		dxl_comm_result = packetHandlerP2_->ping(portHandler_, servo_id, &error);
	}
	else
	{
		return false;
	} 

	//CHECK IF PING SUCCEEDED
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		//ROS_INFO("Dynamixel with id %d found", servo_id);
		return true;
	}
	else
	{
		//ROS_ERROR("ping failure: %d", dxl_comm_result);
		return false;
	}


	return false;

}

/**
 * Get's the model number from the dynamixel's eeprom
 * @param servo_id The ID of the servo to retrieve from
 * @param model_number Stores the model_number returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getModelNumber(int servo_id, uint16_t& model_number)
{
	uint8_t error;
	int dxl_comm_result;

	//ping dynamixel on bus
	if (servo_series_ == 'M')
	{
		//read in first 2 bytes of eeprom (same for all series)
		dxl_comm_result = packetHandlerP1_->read2ByteTxRx(portHandler_, servo_id, 0, &model_number, &error);
	}
	else if ((servo_series_ == 'X') || (servo_series_ == 'P'))
	{
		//read in first 2 bytes of eeprom (same for all series)
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, 0, &model_number, &error);
	}
	else
	{
		return false;
	} 

	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Get's the model info values from the dynamixel's eeprom. Pro and XM series only
 * @param servo_id The ID of the servo to retrieve from
 * @param model_info Stores the model info returned
 * @return True on comm success with pro or xm series, false otherwise.
 */
bool DynamixelDriver::getModelInfo(int servo_id, uint32_t& model_info)
{
	uint8_t error;
	int dxl_comm_result;

	// no model information on MX series
	if (servo_series_ == 'M')
	{
		return false;
	}
	else if ((servo_series_ == 'X') || (servo_series_ == 'P'))
	{
		//address = 2, size = 4
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_PRO_MODEL_INFO, &model_info, &error);

		if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

/**
 * Get's the firmware version number from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param firmware_version Stores the firmware_version returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getFirmwareVersion(int servo_id, uint8_t& firmware_version)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_FIRMWARE_VERSION, &firmware_version, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_X_FIRMWARE_VERSION, &firmware_version, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_PRO_FIRMWARE_VERSION, &firmware_version, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Get's the baud rate from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param baud_rate Stores the baud_rate returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getBaudRate(int servo_id, uint8_t& baud_rate)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_BAUD_RATE, &baud_rate, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_X_BAUD_RATE, &baud_rate, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_PRO_BAUD_RATE, &baud_rate, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Get's the return delay time from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param return_delay_time Stores the value returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getReturnDelayTime(int servo_id, uint8_t& return_delay_time)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_RETURN_DELAY_TIME, &return_delay_time, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_X_RETURN_DELAY_TIME, &return_delay_time, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_PRO_RETURN_DELAY_TIME, &return_delay_time, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Get's the operating_mode from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param operating_mode Stores the value returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getOperatingMode(int servo_id, uint8_t &operating_mode)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		return false;
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_X_OPERATING_MODE, &operating_mode, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_PRO_OPERATING_MODE, &operating_mode, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Get's the angle limits from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param min_angle_limit Stores the min angle limit returned
 * @param max_angle_limit Stores the max angle limit returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getAngleLimits(int servo_id, uint32_t& min_angle_limit, uint32_t& max_angle_limit)
{

	if (getMaxAngleLimit(servo_id, max_angle_limit) == true)
	{
		return getMinAngleLimit(servo_id, min_angle_limit);
	}
	else
	{
		return false;
	}

}

/**
 * Get's the maximum angle limit from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param angle Stores the value returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getMaxAngleLimit(int servo_id, uint32_t& angle)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read2ByteTxRx(portHandler_, servo_id, DXL_MX_CCW_ANGLE_LIMIT, 
				(uint16_t*) &angle, &error);

	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_X_MAX_POSITION_LIMIT, 
				&angle, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_PRO_MAX_ANGLE_LIMIT, 
				&angle, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Get's the minimum angle limit from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param angle Stores the value returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getMinAngleLimit(int servo_id, uint32_t& angle)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read2ByteTxRx(portHandler_, servo_id, DXL_MX_CW_ANGLE_LIMIT, 
				(uint16_t*) &angle, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_X_MIN_POSITION_LIMIT, 
				&angle, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_PRO_MIN_ANGLE_LIMIT, 
				&angle, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}
    
/**
 * Retrieves the voltage limits from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param min_voltage_limit Stores the min angle limit returned
 * @param max_voltage_limit Stores the max angle limit returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getVoltageLimits(int servo_id, float& min_voltage_limit, float& max_voltage_limit)
{

	if (getMaxVoltageLimit(servo_id, max_voltage_limit) == true)
	{
		return getMinVoltageLimit(servo_id, min_voltage_limit);
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves minimum voltage limit from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param min_voltage_limit Stores value returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getMinVoltageLimit(int servo_id, float& min_voltage_limit)
{

	uint8_t error;
	int dxl_comm_result;
	uint16_t data = 0;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_DOWN_LIMIT_VOLTAGE, 
				(uint8_t*) &data, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_X_MIN_VOLTAGE_LIMIT, 
				&data, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_PRO_DOWN_LIMIT_VOLTAGE, 
				&data, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		min_voltage_limit = (data) / 10;
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the maximum voltage from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param max_voltage_limit Stores value returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getMaxVoltageLimit(int servo_id, float& max_voltage_limit)
{

	uint8_t error;
	int dxl_comm_result;
	uint16_t data = 0;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_UP_LIMIT_VOLTAGE, 
				(uint8_t*) &data, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_X_MAX_VOLTAGE_LIMIT, 
				&data, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_PRO_UP_LIMIT_VOLTAGE, 
				&data, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		max_voltage_limit = (data) / 10;
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the maximum temperature limit from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param max_temperature Stores value returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getTemperatureLimit(int servo_id, uint8_t& max_temperature)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_LIMIT_TEMPERATURE, 
				&max_temperature, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_X_TEMPERATURE_LIMIT, 
				&max_temperature, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_PRO_LIMIT_TEMPERATURE, 
				&max_temperature, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the maximum torque limit from the dynamixel's eeprom.
 * @param servo_id The ID of the servo to retrieve from
 * @param max_torque Stores value returned
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getMaxTorque(int servo_id, uint16_t& max_torque)
{


	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_MAX_TORQUE, 
				(uint8_t*) &max_torque, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_X_CURRENT_LIMIT, 
				&max_torque, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_PRO_MAX_TORQUE, 
				&max_torque, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the maximum torque limit from the dynamixel's ram.
 * @param servo_id The ID of the servo to retrieve from
 * @param torque_enabled Stores the status of torque enable
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::getTorqueEnabled(int servo_id, bool& torque_enabled)
{

	uint8_t error;
	int dxl_comm_result;
	uint8_t data = 0;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_TORQUE_ENABLE, &data, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_X_TORQUE_ENABLE, &data, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_PRO_TORQUE_ENABLE, &data, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		torque_enabled = (data > 0);
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the current target position from the dynamixel's ram.
 * @param servo_id The ID of the servo to retrieve from
 * @param target_position Stores the value returned
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::getTargetPosition(int servo_id, int32_t& target_position)
{
	uint8_t error;
	int dxl_comm_result;
	int16_t data = 0;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read2ByteTxRx(portHandler_, servo_id, DXL_MX_GOAL_POSITION_L, 
				(uint16_t*) &data, &error);
		target_position = data;
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_X_GOAL_POSITION, 
				(uint32_t*) &target_position, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_PRO_GOAL_POSITION, 
				(uint32_t*) &target_position, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Retrieves the current target_velocity from the dynamixel's ram.
 * @param servo_id The ID of the servo to retrieve from
 * @param target_velocity Stores the value returned
 * @return True on comm success, false otherwise.
 */    
bool DynamixelDriver::getTargetVelocity(int servo_id, int32_t& target_velocity)
{

	uint8_t error;
	int dxl_comm_result;
	int16_t data = 0;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read2ByteTxRx(portHandler_, servo_id, DXL_MX_GOAL_SPEED_L, 
				(uint16_t*) &data, &error);
		target_velocity = data;
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_X_GOAL_VELOCITY, 
				(uint32_t*) &target_velocity, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_PRO_GOAL_VELOCITY, 
				(uint32_t*) &target_velocity, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the current position from the dynamixel's ram.
 * @param servo_id The ID of the servo to retrieve from
 * @param position Stores the value returned
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::getPosition(int servo_id, int32_t& position)
{

	uint8_t error;
	int dxl_comm_result;
	int16_t data = 0;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read2ByteTxRx(portHandler_, servo_id, DXL_MX_PRESENT_POSITION_L, 
				(uint16_t*) &data, &error);
		position = data;
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_X_PRESENT_POSITION, 
				(uint32_t*) &data, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_PRO_PRESENT_POSITION, 
				(uint32_t*) &data, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the current velocity from the dynamixel's ram.
 * @param servo_id The ID of the servo to retrieve from
 * @param velocity Stores the value returned
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::getVelocity(int servo_id, int32_t& velocity)
{

	uint8_t error;
	int dxl_comm_result;
	int16_t data = 0;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read2ByteTxRx(portHandler_, servo_id, DXL_MX_PRESENT_SPEED_L, 
				(uint16_t*) &data, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_X_PRESENT_POSITION, 
				(uint32_t*) &velocity, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read4ByteTxRx(portHandler_, servo_id, DXL_PRO_PRESENT_POSITION, 
				(uint32_t*) &velocity, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the current value from the dynamixel's ram.
 * @param servo_id The ID of the servo to retrieve from
 * @param current Stores the value returned
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::getCurrent(int servo_id, uint16_t& current)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read2ByteTxRx(portHandler_, servo_id, DXL_MX_PRESENT_CURRENT_L, 
				&current, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_X_PRESENT_CURRENT, 
				&current, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_PRO_PRESENT_CURRENT, 
				&current, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the current voltage value from the dynamixel's ram.
 * @param servo_id The ID of the servo to retrieve from
 * @param voltage Stores the value returned
 * @return True on comm success, false otherwise.
 */ 
bool DynamixelDriver::getVoltage(int servo_id, float& voltage)
{

	uint8_t error;
	int dxl_comm_result;
	uint16_t data = 0;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_PRESENT_VOLTAGE, 
				(uint8_t*) &data, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_X_PRESENT_INPUT_VOLTAGE, 
				&data, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read2ByteTxRx(portHandler_, servo_id, DXL_PRO_PRESENT_VOLTAGE, 
				&data, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		voltage = (data) / 10;
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Retrieves the current temperature value from the dynamixel's ram.
 * @param servo_id The ID of the servo to retrieve from
 * @param temperature Stores the value returned
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::getTemperature(int servo_id, uint8_t& temperature)
{
	
	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->read1ByteTxRx(portHandler_, servo_id, DXL_MX_LIMIT_TEMPERATURE, 
				&temperature, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_X_TEMPERATURE_LIMIT, 
				&temperature, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->read1ByteTxRx(portHandler_, servo_id, DXL_PRO_LIMIT_TEMPERATURE, 
				&temperature, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/** 
 * Retrieves arbitrary register readings from the Dynamixel, can be used in cases where provided getters are
 * insufficient or to read multiple contiguous values at once.
 * @param servo_id The ID of the servo to retrieve from
 * @param address The address value in the control table the dynamixel will start reading from
 * @param length The number of bytes to read consecutively from the control table
 * @param response Array to store the raw dynamixel response.
 */
bool DynamixelDriver::readRegisters(int servo_id, uint32_t address, uint32_t length, uint8_t *response)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->readTxRx(portHandler_, servo_id, address, length, response, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->readTxRx(portHandler_, servo_id, address, length, response, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->readTxRx(portHandler_, servo_id, address, length, response, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

// *********************** BULK_READ METHODS *************************** //

/**
 * Bulk Reads the following values in one instruction
 *
 * - Present Position
 * - Present Velocity
 * - Present Current (or load if MX Series)
 *
 * @param servo_ids Pointer to a list of ID's to respond. Dynamixels will respond in order of list index
 * @param responses Pointer map of dynamixel ID's to dynamixel response vectors, response vectors are a list of 
 * parameter values in the order given above.
 * @return True on comm success, false otherwise
 */
bool DynamixelDriver::getBulkStateInfo(std::vector<int> *servo_ids, std::map<int, std::vector<int32_t> > *responses)
{
	
	
	std::vector<int32_t> response;
	std::vector<uint8_t> data;

	std::map<int, std::vector<uint8_t> > *raw = new std::map<int, std::vector<uint8_t> >;

	uint32_t value = 0;

	if (servo_series_ == 'M')
	{		
		//Read data from dynamixels
		if( bulkRead(servo_ids, DXL_MX_PRESENT_POSITION_L, 6, raw) )
		{
			//DECODE RAW DATA
			for (int i = 0; i < servo_ids->size(); i++)
			{

				//get raw data response
				std::vector<uint8_t> data = raw->at(servo_ids->at(i));

				//get position (data[0] - data[1])
				value = MAKEWORD(data[0], data[1]);
				response.push_back(value);

				//get velocity (data[2] - data[3])
				value = MAKEWORD(data[2], data[3]);
				response.push_back(value);

				//get load (data[4] - data[5])
				value = MAKEWORD(data[4], data[5]);
				response.push_back(value);

				//place responses into return data
				responses->insert(std::pair<int, std::vector<int32_t> >(servo_ids->at(i), response));

				response.clear();
				data.clear();

			}
			return true;
		}
		else
		{
			return false;
		}

	}
	else if (servo_series_ == 'X')
	{
		//read data from dynamixels
		if( syncRead(servo_ids, DXL_X_PRESENT_CURRENT, 10, raw) )
		{
			//DECODE RAW DATA
			for (int i = 0; i < servo_ids->size(); i++)
			{

				//get raw data response
				std::vector<uint8_t> data = raw->at(servo_ids->at(i));

				//get position (data[0] - data[1])
				value = MAKEINT(MAKEWORD(data[6], data[7]),MAKEWORD(data[8], data[9]));
				response.push_back(value);

				//get velocity (data[2] - data[3])
				value = MAKEINT(MAKEWORD(data[2], data[3]),MAKEWORD(data[4], data[5]));
				response.push_back(value);

				//get load (data[4] - data[5])
				value = MAKEWORD(data[0], data[1]);
				response.push_back(value);

				//place responses into return data
				responses->insert(std::pair<int, std::vector<int32_t> >(servo_ids->at(i), response));

				response.clear();
				data.clear();
			
			}
			return true;
		}
		else
		{
			return false;
		}
	}
	else if (servo_series_ == 'P')
	{
		//read data from dynamixels
		if( syncRead(servo_ids, DXL_PRO_PRESENT_POSITION, 10, raw) )
		{
			//DECODE RAW DATA
			for (int i = 0; i < servo_ids->size(); i++)
			{

				//get raw data response
				std::vector<uint8_t> data = raw->at(servo_ids->at(i));

				//get position (data[0] - data[1])
				value = MAKEINT(MAKEWORD(data[0], data[1]),MAKEWORD(data[2], data[3]));
				response.push_back(value);

				//get velocity (data[2] - data[3])
				value = MAKEINT(MAKEWORD(data[4], data[5]),MAKEWORD(data[6], data[7]));
				response.push_back(value);

				//get load (data[4] - data[5])
				value = MAKEWORD(data[8], data[9]);
				response.push_back(value);

				//place responses into return data
				responses->insert(std::pair<int, std::vector<int32_t> >(servo_ids->at(i), response));

				response.clear();
				data.clear();
			
			}
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
	return false;
}

/**
 * Bulk Reads the following servo state variables in one instruction
 *
 *  - goal position
 *  - goal velocity
 *  - goal torque (current)
 *  - present position
 *  - present velocity
 *  - present load (current)
 *  - present voltage
 *  - present temperature
 *
 * @param servo_ids Pointer to a list of ID's to respond. Dynamixels will respond in order of list index
 * @param responses Pointer map of dynamixel ID's to dynamixel response vectors, response vectors are a list of 
 * parameter values in the order given above.
 * @return True on comm success, false otherwise
 */
bool DynamixelDriver::getBulkStatusInfo(std::vector<int> *servo_ids,
                           std::map<int, std::vector<int32_t> >  *responses)
{
	
	std::vector<int32_t> response;
	std::vector<uint8_t> data;

	std::map<int, std::vector<uint8_t> > *raw = new std::map<int, std::vector<uint8_t> >;

	uint32_t value = 0;

	if (servo_series_ == 'M')
	{		
		//Read data from dynamixels
		if( bulkRead(servo_ids, DXL_MX_GOAL_POSITION_L, 43, raw) )
		{
			//DECODE RAW DATA
			for (int i = 0; i < servo_ids->size(); i++)
			{

				//get raw data response
				std::vector<uint8_t> data = raw->at(servo_ids->at(i));

				//get goal position (data[0] - data[1])
				value = MAKEWORD(data[0], data[1]);
				response.push_back(value);

				//get goal velocity (data[2] - data[3])
				value = MAKEWORD(data[2], data[3]);
				response.push_back(value);

				//get goal torque (data[41] - data[42])
				value = MAKEWORD(data[41], data[42]);
				response.push_back(value);

				//get present position (data[6] - data[7])
				value = MAKEWORD(data[6], data[7]);
				response.push_back(value);

				//get present velocity (data[8] - data[9])
				value = MAKEWORD(data[8], data[9]);
				response.push_back(value);

				//get present torque (data[10] - data[11])
				value = MAKEWORD(data[10], data[11]);
				response.push_back(value);

				//get present voltage (data[12])
				value = data[12];
				response.push_back(value);

				//get present temperature (data[13])
				value = data[13];
				response.push_back(value);

				//place responses into return data
				responses->insert(std::pair<int, std::vector<int32_t> >(servo_ids->at(i), response));

				response.clear();
				data.clear();

			}
			return true;
		}
		else
		{
			return false;
		}

	}
	else if (servo_series_ == 'X')
	{
		//read data from dynamixels
		if( syncRead(servo_ids, DXL_X_GOAL_CURRENT, 45, raw) )
		{
			//DECODE RAW DATA
			for (int i = 0; i < servo_ids->size(); i++)
			{

				//get raw data response
				std::vector<uint8_t> data = raw->at(servo_ids->at(i));

				//get goal position
				value = MAKEINT(MAKEWORD(data[14], data[15]),MAKEWORD(data[16], data[17]));
				response.push_back(value);

				//get goal velocity
				value = MAKEINT(MAKEWORD(data[2], data[3]),MAKEWORD(data[4], data[5]));
				response.push_back(value);

				//get goal torque
				value = MAKEWORD(data[0], data[1]);
				response.push_back(value);

				//get present position
				value = MAKEINT(MAKEWORD(data[30], data[31]),MAKEWORD(data[32], data[33]));
				response.push_back(value);

				//get present velocity
				value = MAKEINT(MAKEWORD(data[26], data[27]),MAKEWORD(data[28], data[29]));
				response.push_back(value);

				//get present torque
				value = MAKEWORD(data[24], data[25]);
				response.push_back(value);

				//get voltage
				value = MAKEWORD(data[42], data[43]);
				response.push_back(value);

				//get temperature
				value = data[44];
				response.push_back(value);

				//place responses into return data
				responses->insert(std::pair<int, std::vector<int32_t> >(servo_ids->at(i), response));

				response.clear();
				data.clear();
			
			}
			return true;
		}
		else
		{
			return false;
		}
	}
	else if (servo_series_ == 'P')
	{
		//read data from dynamixels
		if( syncRead(servo_ids, DXL_PRO_GOAL_POSITION, 30, raw) )
		{
			//DECODE RAW DATA
			for (int i = 0; i < servo_ids->size(); i++)
			{
				//get raw data response
				std::vector<uint8_t> data = raw->at(servo_ids->at(i));

				//get goal position
				value = MAKEINT(MAKEWORD(data[0], data[1]),MAKEWORD(data[2], data[3]));
				response.push_back(value);

				//get goal velocity
				value = MAKEINT(MAKEWORD(data[4], data[5]),MAKEWORD(data[6], data[7]));
				response.push_back(value);

				//get goal torque
				value = MAKEWORD(data[8], data[9]);
				response.push_back(value);

				//get present position
				value = MAKEINT(MAKEWORD(data[15], data[16]),MAKEWORD(data[17], data[18]));
				response.push_back(value);

				//get present velocity
				value = MAKEINT(MAKEWORD(data[19], data[20]),MAKEWORD(data[21], data[22]));
				response.push_back(value);

				//get present torque
				value = MAKEWORD(data[23], data[24]);
				response.push_back(value);

				//get voltage
				value = MAKEWORD(data[25], data[26]);
				response.push_back(value);

				//get temperature
				value = data[27];
				response.push_back(value);

				//place responses into return data
				responses->insert(std::pair<int, std::vector<int32_t> >(servo_ids->at(i), response));

				response.clear();
				data.clear();
			}
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
	return false;
}

// ********************** Protected Read Methods *********************** //


/** 
 * Performs the bulk read for each protocol. A bulk read is a broadcast instruction on a bus that commands a list
 * of dynamixels to respond in order with a read of a specified address and length (which can be different for each
 * dynamixel). This protocol can be used to read many parameters from many dynamixels on a bus in just one
 * instruction.
 * @param servo_ids Pointer to a list of ID's to respond. Dynamixels will respond in order of list index
 * @param address The address value in the control table the dynamixels will start reading from
 * @param length The number of bytes to read consecutively from the control table
 * @param responses Pointer map of dynamixel ID's to dynamixel response vectors, response vectors are a raw array of
 * the bytes read from the control table of each dynamixel
 */
bool DynamixelDriver::bulkRead(std::vector<int> *servo_ids,
                       uint16_t address,
                       uint16_t length,
                       std::map<int, std::vector<uint8_t> >  *responses)
{

	int dxl_comm_result;
	uint8_t byte;
	bool success;
	std::vector<uint8_t> *response;

	//MX SERIES ONLY
	if (servo_series_ != 'M')
	{
		return false;
	}

    // Initialize GroupSyncRead instance
    dynamixel::GroupBulkRead GroupBulkRead(portHandler_, packetHandlerP1_);

    // add all dyanmixels to sync read
    for (int i = 0; i < servo_ids->size(); i++)
    {
    	GroupBulkRead.addParam(servo_ids->at(i), address, length);
    }

    //perform sync read
   	dxl_comm_result = GroupBulkRead.txRxPacket();

   	if (dxl_comm_result != COMM_SUCCESS)
   	{
   		return false;
   	}

 	//get all responses back from read
    for (int i = 0; i < servo_ids->size(); i++)
    {

    	// new vector for each dynamixel
    	response = new std::vector<uint8_t>;

    	if(GroupBulkRead.isAvailable(servo_ids->at(i), address, length))
    	{

	        // Get values from read and place into vector
	        for (int j = 0; j < length; j++)
	        {
	            byte = GroupBulkRead.getData(servo_ids->at(i), address + j, 1);
	            response->push_back(byte);

	        }

    		//place vector into map of responses
	        responses->insert(std::pair<int, std::vector<uint8_t> >(servo_ids->at(i), *response));

    	}
    	else
    	{
    		return false;
    	}

    } 

    return true;

}


/** 
 * Performs the bulk read for each protocol. A bulk read is a broadcast instruction on a bus that commands a list
 * of dynamixels to respond in order with a read of a specified address and length (the same value for all 
 * dynamixels read) This protocol can be used to read many parameters from many dynamixels on a bus in 
 * just one instruction.
 * @param servo_ids Pointer to a list of ID's to respond. Dynamixels will respond in order of list index
 * @param address The address value in the control table the dynamixels will start reading from
 * @param length The number of bytes to read consecutively from the control table
 * @param responses Pointer map of dynamixel ID's to dynamixel response vectors, response vectors are a raw array of
 * the bytes read from the control table of each dynamixel
 */
bool DynamixelDriver::syncRead(std::vector<int> *servo_ids,
                       uint16_t address,
                       uint16_t length,
                       std::map<int, std::vector<uint8_t> >  *responses)
{

	int dxl_comm_result;
	uint8_t byte;
	bool success;
	std::vector<uint8_t> *response;

	//PRO AND X SERIES ONLY
	if ((servo_series_ != 'X') && (servo_series_ != 'P'))
	{
		return false;
	}

    // Initialize GroupSyncRead instance
    dynamixel::GroupSyncRead GroupSyncRead(portHandler_, packetHandlerP2_, address, length);

    // add all dyanmixels to sync read
    for (int i = 0; i < servo_ids->size(); i++)
    {
    	GroupSyncRead.addParam(servo_ids->at(i));
    }

    //perform sync read
   	dxl_comm_result = GroupSyncRead.txRxPacket();

   	if (dxl_comm_result != COMM_SUCCESS)
   	{
   		return false;
   	}

 	//get all responses back from read
    for (int i = 0; i < servo_ids->size(); i++)
    {

    	// new vector for each dynamixel
    	response = new std::vector<uint8_t>;

    	if(GroupSyncRead.isAvailable(servo_ids->at(i), address, length))
    	{

	        // Get values from read and place into vector
	        for (int j = 0; j < length; j++)
	        {

	        	byte = GroupSyncRead.getData(servo_ids->at(i), address + j, 1);
	            response->push_back(byte);

	        }

    		//place vector into map of responses
	        responses->insert(std::pair<int, std::vector<uint8_t> >(servo_ids->at(i), *response));

    	}
    	else
    	{
    		return false;
    	}

    }  	

    return true;

}


// **************************** SETTERS ******************************** //



/**
 * Writes a new ID value to the Dynamixel's eeprom. The value MUST be unique or all dynamixel's
 * with the same ID on a bus will fail to respond.
 * @param servo_id The ID of the servo to write to
 * @param new_id The new ID to set this dynamixel to
 * @return True on comm success iff new_id not already in use, false otherwise.
 */  
bool DynamixelDriver::setId(int servo_id, uint8_t new_id)
{

	uint8_t error;
	int dxl_comm_result;

	if (ping(new_id))
	{
		ROS_ERROR("ID Collision avoided! Could not servo id from %d to %d, already in use", servo_id, new_id);
		return false;
	}

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_ID, new_id, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_X_ID, new_id, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_PRO_ID, new_id, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Writes a new baud rate value to the dynamixels eeprom.
 * @param servo_id The ID of the servo to write to
 * @param baud_rate The new baud rate for the dynamixel. Values are defined as follows:
 *  - Value = 0-249: 	Baud = 2Mbps/ (value + 1)
 *  - Value = 250: 		Baud = 2.25Mbps
 *  - Value = 251: 		Baud = 2.5Mbps
 *  - Value = 252: 		Baud = 3Mbps
 *  - Value = 253,254: 	INVALID, DO NOT SET
 * @return True on comm success and valid baud rate, false otherwise.
 */ 
bool DynamixelDriver::setBaudRate(int servo_id, uint8_t baud_rate)
{

	uint8_t error;
	int dxl_comm_result;

	if (baud_rate > 252)
	{
		return false;
	}

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_BAUD_RATE, baud_rate, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_X_BAUD_RATE, baud_rate, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_PRO_BAUD_RATE, baud_rate, 
				&error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Writes a new return delay time value to the dynamixels eeprom.
 * @param servo_id The ID of the servo to write to
 * @param return_delay_time The value to write to the register
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::setReturnDelayTime(int servo_id, uint8_t return_delay_time)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_RETURN_DELAY_TIME, 
				return_delay_time, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_X_RETURN_DELAY_TIME, 
				return_delay_time, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_PRO_RETURN_DELAY_TIME, 
				return_delay_time, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the Operating mode of the Dynamixel. The three possible control modes are: Position, Velocity or Torque.
 * The way these modes are enabled differs for each series of dynamixel. XM and Pro have an operating mode register
 * that can be used to changed the control type. For MX series, the method of control is defined by the values in 
 * the angle limit registers and the torque_control_enable register. Note torque control is not available on the 
 * MX-28 models. 
 * @param servo_id The ID of the servo to write to
 * @param operating_mode The method of control: values are defined as:
 *  - 0: Torque Control
 *  - 1: Velocity Control
 *  - 3: Position Control 
 * @return True on comm success and valid operating mode, false otherwise.
 */  
bool DynamixelDriver::setOperatingMode(int servo_id, uint8_t operating_mode)
{
	uint8_t error;
	int dxl_comm_result = 0;
	uint16_t model_num;
	bool success = true;
	
	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{

		//MX Series has no operating mode register, instead operating mode depends on
		//angle limits and torque_control_enable register
        if (operating_mode == 3)
        {
        	//Position control, set normal angle limits
            success = setAngleLimits(servo_id, 0, 4095); //Master mode, normal roatation
         	
         	//Torque control mode, there is actually a register for this
            success = getModelNumber(servo_id, model_num);
            if ( (model_num == 310) || (model_num == 320) ) //No torque control on MX-28
            {
     			dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_TORQUE_CONTROL_ENABLE, 
     					0, &error);       	
            }

        }
        else if (operating_mode == 1)
        {
        	//Velocity control, turn off angle limits
            success = setAngleLimits(servo_id, 0, 0); //Master mode, normal roatation
        	//Torque control mode, there is actually a register for this
            success = getModelNumber(servo_id, model_num);
            if ( (model_num == 310) || (model_num == 320) ) //No torque control on MX-28
            {
     			dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_TORQUE_CONTROL_ENABLE, 
     					0, &error);       	
            }
        } 
        else if (operating_mode == 0)
        {
        	//Torque control, turn off angle limits
            success = setAngleLimits(servo_id, 0, 0); //Master mode, normal roatation
        	//Torque control mode, there is actually a register for this
            success = getModelNumber(servo_id, model_num);
            if ( (model_num == 310) || (model_num == 320) ) //No torque control on MX-28
            {
     			dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_TORQUE_CONTROL_ENABLE, 
     					1, &error);       	
            }
        }
        else
        {
        	return false;
        }
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_X_OPERATING_MODE, 
				operating_mode, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_PRO_OPERATING_MODE, 
				operating_mode, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127) && (success))
	{
		return true;
	}
	else
	{
		return false;
	}
}


/**
 * Sets the dynamixel to reverse it's response directions. not this also moves the zero encoder value by 
 * 180 degrees.
 * @param servo_id The ID of the servo to write to
 * @param reverse boolean dynamixel direction value (true for reversed, false for normal)
 */
bool DynamixelDriver::setReverseDirection(int servo_id, bool reverse)
{
	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_DRIVE_MODE,  
				(reverse == true) ? 1 : 0, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_X_DRIVE_MODE, 
				(reverse == true) ? 1 : 0, &error);
	}
	else if (servo_series_ == 'P')
	{
		return false;
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Sets the minimum and maximum angle limits for the dynamixel
 * @param servo_id The ID of the servo to write to
 * @param min_angle the minimum angle limit (in encoder values)
 * @param max_angle the maximum angle limit (in encoder values)
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::setAngleLimits(int servo_id, int32_t min_angle, int32_t max_angle)
{

	if (setMaxAngleLimit(servo_id, max_angle) == true)
	{
		return setMinAngleLimit(servo_id, min_angle);
	}
	else
	{
		return false;
	}

}

/**
* Sets the minimum angle limit for the dynamixel
* @param servo_id The ID of the servo to write to
* @param angle the minimum angle limit (in encoder values)
* @return True on comm success, false otherwise.
*/ 
bool DynamixelDriver::setMinAngleLimit(int servo_id, int32_t angle)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write2ByteTxRx(portHandler_, servo_id, DXL_MX_CW_ANGLE_LIMIT, 
				(int16_t) angle, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_X_MIN_POSITION_LIMIT, 
				angle, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_PRO_MIN_ANGLE_LIMIT, 
				angle, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the maximum angle limit for the dynamixel
 * @param servo_id The ID of the servo to write to
 * @param angle the maximum angle limit (in encoder values)
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::setMaxAngleLimit(int servo_id, int32_t angle)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{

		dxl_comm_result = packetHandlerP1_->write2ByteTxRx(portHandler_, servo_id, DXL_MX_CCW_ANGLE_LIMIT, 
				(int16_t) angle, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_X_MAX_POSITION_LIMIT, 
				angle, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_PRO_MAX_ANGLE_LIMIT, 
				angle, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the maximum temperature limit for the dynamixel
 * @param servo_id The ID of the servo to write to
 * @param max_temperature the maximum temperature limit
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::setTemperatureLimit(int servo_id, uint8_t max_temperature)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_LIMIT_TEMPERATURE, 
				max_temperature, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_X_TEMPERATURE_LIMIT, 
				max_temperature, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_PRO_LIMIT_TEMPERATURE, 
				max_temperature, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the maximum torque limit for the dynamixel
 * @param servo_id The ID of the servo to write to
 * @param max_torque the maximum torque limit
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setMaxTorque(int servo_id, uint16_t max_torque)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write2ByteTxRx(portHandler_, servo_id, DXL_MX_MAX_TORQUE, 
				max_torque, &error);
		dxl_comm_result = packetHandlerP1_->write2ByteTxRx(portHandler_, servo_id, DXL_MX_TORQUE_LIMIT_L, 
				max_torque, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_X_CURRENT_LIMIT, 
				max_torque, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_PRO_MAX_TORQUE, 
				max_torque, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the torque enable register of the dynamixel. This value defines the on/off state of the servo.
 * @param servo_id The ID of the servo to write to
 * @param on The state of the servo (true = on, false = off).
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::setTorqueEnabled(int servo_id, bool on)
{

	uint8_t error;
	uint16_t model_num;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = getModelNumber(servo_id, model_num);
        if (((model_num == 310) || (model_num == 320) ) && (on == false))
        {
 			dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_TORQUE_CONTROL_ENABLE, 
 				0, &error);       	
        }
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_TORQUE_ENABLE, 
				(uint8_t) on, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_X_TORQUE_ENABLE, 
				(uint8_t) on, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write1ByteTxRx(portHandler_, servo_id, DXL_PRO_TORQUE_ENABLE, 
				(uint8_t) on, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the PID gains for the dynamixel. note that the PID gains are available based on series and control
 * type:
 *  - MX Series:
 *      - Position: P,I,D
 *      - Velocity: P,I,D
 *      - Torque:   P,I,D
 *  - XM Series:
 *      - Position: P,I,D
 *      - Velocity: P,I
 *      - Torque:   None
 *  - Pro Series:
 *      - Position: P
 *      - Velocity: P,I
 *      - Torque:   None
 *
 * @param servo_id The ID of the servo to write to
 * @param operating_mode The operating mode to set gains for. @see setOperatingMode
 * @param p_gain The proportional gain value to write
 * @param i_gain The integral gain value to write
 * @param d_gain The derivative gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setPIDGains(int servo_id, uint8_t operating_mode, double p_gain, double i_gain, double d_gain)
{

	uint16_t p_val, i_val, d_val;

	//Convert values based on servo series
	if (servo_series_ == 'M')
	{
		//ROS_INFO("%f, %f, %f", p_gain, i_gain, d_gain);
		p_val = (uint16_t) (p_gain * 8.0);
		i_val = (uint16_t) (i_gain * (1000.0 / 2048.0));
		d_val = (uint16_t) (d_gain / (4.0 / 1000.0));
		//ROS_INFO("%d, %d, %d", p_val, i_val, d_val);
	}
	else if (servo_series_ == 'X')
	{
		p_val = (uint16_t) (p_gain * 128.0);
		i_val = (uint16_t) (i_gain * 65536.0);
		d_val = (uint16_t) (d_gain * 16.0);
	}
	else if (servo_series_ == 'P')
	{
		return false;
	}
	else
	{
		return false;
	}

	//set the register values for the given control mode,
	//This changes based on servo series, a value of gain < 0
	//indicates that the gain should be left to the default
	if (operating_mode == 3) //Position Control
	{
		if (!(p_gain < 0))
		{
			if(!setPositionProportionalGain(servo_id, p_val))
			{
				return false;
			}
		}

		if (!(i_gain < 0))
		{
			if(!setPositionIntegralGain(servo_id, i_val))
			{
				return false;
			}
		}

		if (!(d_gain < 0))
		{
			if(!setPositionDerivativeGain(servo_id, d_val))
			{
				return false;
			}
		}
	}
	else if (operating_mode == 1) //Velocity Control
	{

		if (!(p_gain < 0))
		{
			if(!setVelocityProportionalGain(servo_id, p_val))
			{
				return false;
			}
		}

		if (!(i_gain < 0))
		{
			if(!setVelocityIntegralGain(servo_id, i_val))
			{
				return false;
			}
		}

		if (!(d_gain < 0))
		{
			if(!setVelocityDerivativeGain(servo_id, d_val))
			{
				return false;
			}
		}
	}
	else if (operating_mode == 0) //torque control
	{

		if (!(p_gain < 0))
		{
			if(!setTorqueProportionalGain(servo_id, p_val))
			{
				return false;
			}
		}

		if (!(i_gain < 0))
		{
			if(!setTorqueIntegralGain(servo_id, i_val))
			{
				return false;
			}
		}

		if (!(d_gain < 0))
		{
			if(!setTorqueDerivativeGain(servo_id, d_val))
			{
				return false;
			}
		}
	}

	return true;

}

/**
 * Sets the proportional gain value for the position control mode if available. @see setPIDGains
 * @param servo_id The ID of the servo to write to
 * @param gain The proportional gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setPositionProportionalGain(int servo_id, uint16_t gain)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_P_GAIN,  
				(uint8_t) (gain & 0x00FF), &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_X_POSITION_P_GAIN,  
				gain, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_PRO_POSITION_P_GAIN,  
				gain, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the integral gain value for the position control mode if available. @see setPIDGains
 * @param servo_id The ID of the servo to write to
 * @param gain The integral gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setPositionIntegralGain(int servo_id, uint16_t gain)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_D_GAIN,  
				(uint8_t) (gain & 0x00FF), &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_X_POSITION_D_GAIN,  
				gain, &error);
	}
	else if (servo_series_ == 'P')
	{
		return false;
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Sets the derivative gain value for the position control mode if available. @see setPIDGains
 * @param servo_id The ID of the servo to write to
 * @param gain The derivative gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setPositionDerivativeGain(int servo_id, uint16_t gain)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_D_GAIN,  
				(uint8_t) (gain & 0x00FF), &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_X_POSITION_D_GAIN,  
				gain, &error);
	}
	else if (servo_series_ == 'P')
	{
		return false;
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Sets the proportional gain value for the velocity control mode if available. @see setPIDGains
 * @param servo_id The ID of the servo to write to
 * @param gain The proportional gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setVelocityProportionalGain(int servo_id, uint16_t gain)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_P_GAIN,  
				(uint8_t) (gain & 0x00FF), &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_X_VELOCITY_P_GAIN,  
				gain, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_PRO_VELOCITY_P_GAIN,  
				gain, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Sets the integral gain value for the velocity control mode if available. @see setPIDGains
 * @param servo_id The ID of the servo to write to
 * @param gain The integral gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setVelocityIntegralGain(int servo_id, uint16_t gain)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_I_GAIN,  
				(uint8_t) (gain & 0x00FF), &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_X_VELOCITY_I_GAIN,  
				gain, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write2ByteTxRx(portHandler_, servo_id, DXL_PRO_VELOCITY_I_GAIN,  
				gain, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the derivative gain value for the velocity control mode if available. @see setPIDGains
 * @param servo_id The ID of the servo to write to
 * @param gain The derivative gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setVelocityDerivativeGain(int servo_id, uint16_t gain)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_D_GAIN,  
				(uint8_t) (gain & 0x00FF), &error);
	}
	else if (servo_series_ == 'X')
	{
		return false;
	}
	else if (servo_series_ == 'P')
	{
		return false;
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Sets the proportional gain value for the torque control mode if available. @see setPIDGains
 * @param servo_id The ID of the servo to write to
 * @param gain The proportional gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setTorqueProportionalGain(int servo_id, uint16_t gain)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_P_GAIN,  
				(uint8_t) (gain & 0x00FF), &error);
	}
	else if (servo_series_ == 'X')
	{
		return false;
	}
	else if (servo_series_ == 'P')
	{
		return false;
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Sets the integral gain value for the torque control mode if available. @see setPIDGains
 * @param servo_id The ID of the servo to write to
 * @param gain The integral gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setTorqueIntegralGain(int servo_id, uint16_t  gain)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_I_GAIN,  
				(uint8_t) (gain & 0x00FF), &error);
	}
	else if (servo_series_ == 'X')
	{
		return false;
	}
	else if (servo_series_ == 'P')
	{
		return false;
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Sets the derivative gain value for the torque control mode if available. @see setPIDGains
 * @param servo_id The ID of the servo to write to
 * @param gain The derivative gain value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setTorqueDerivativeGain(int servo_id, uint16_t gain)
{

	uint8_t error;
	int dxl_comm_result;
	
	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write1ByteTxRx(portHandler_, servo_id, DXL_MX_D_GAIN,  
				(uint8_t) (gain & 0x00FF), &error);
	}
	else if (servo_series_ == 'X')
	{
		return false;
	}
	else if (servo_series_ == 'P')
	{
		return false;
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Sets the goal position of the dynamixel.
 * @param servo_id The ID of the servo to write to
 * @param position The position value to write
 * @return True on comm success, false otherwise.
 */  
bool DynamixelDriver::setPosition(int servo_id, uint32_t position)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write2ByteTxRx(portHandler_, servo_id, DXL_MX_GOAL_POSITION_L, 
				(uint16_t) position, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_X_GOAL_POSITION, 
				position, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_PRO_GOAL_POSITION, 
				position, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the goal velocity of the dynamixel.
 * @param servo_id The ID of the servo to write to
 * @param velocity The velocity value to write
 * @return True on comm success, false otherwise.
 */ 
bool DynamixelDriver::setVelocity(int servo_id, int32_t velocity)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write2ByteTxRx(portHandler_, servo_id, DXL_MX_GOAL_SPEED_L, 
				(uint16_t) velocity, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_X_GOAL_VELOCITY, 
				velocity, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_PRO_GOAL_VELOCITY, 
				velocity, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Sets the profile velocity of the dynamixel. Profile velocity is how fast
 * the servo should move between positions.
 * @param servo_id The ID of the servo to write to
 * @param velocity The profile velocity value to write
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setProfileVelocity(int servo_id, int32_t velocity)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->write2ByteTxRx(portHandler_, servo_id, DXL_MX_GOAL_SPEED_L, 
				(uint16_t) velocity, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_X_PROFILE_VELOCITY, 
				velocity, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->write4ByteTxRx(portHandler_, servo_id, DXL_PRO_GOAL_VELOCITY, 
				velocity, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

/** 
 * CAUTION: THIS FUNCTION IS PROVIDED AS IS, WRITE VALUES AND ADDRESSESARE NOT CHECKED FOR SENSIBILITY OR VALIDITIY, 
 * ENSURE THAT VALUES ARE CHECKED FOR CORRECT ADDRESS AND VALUES BEFORE CALLING THIS FUNCTION, OTHERWISE DYNAMIXELS 
 * MAY HAVE TO BE RESET. Writes arbitrary register values to the Dynamixel, can be used in cases where provided 
 * setters are insufficient or to write multiple contiguous values at once. 
 * @param servo_id The ID of the servo to write to
 * @param address The address value in the control table the dynamixel will start writing to
 * @param length The number of bytes to write consecutively to the control table
 * @param data Array containing the value to be written.
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::writeRegisters(int servo_id, uint32_t address, uint32_t length, uint8_t *data)
{

	uint8_t error;
	int dxl_comm_result;

	//Read address and size always depends on servo series
	if (servo_series_ == 'M')
	{
		dxl_comm_result = packetHandlerP1_->writeTxRx(portHandler_, servo_id, address, length, data, &error);
	}
	else if (servo_series_ == 'X')
	{
		dxl_comm_result = packetHandlerP2_->writeTxRx(portHandler_, servo_id, address, length, data, &error);
	}
	else if (servo_series_ == 'P')
	{
		dxl_comm_result = packetHandlerP2_->writeTxRx(portHandler_, servo_id, address, length, data, &error);
	}
	else
	{
		return false;
	}

	// check return value
	if ((dxl_comm_result == COMM_SUCCESS) && !(error & 127))
	{
		return true;
	}
	else
	{
		return false;
	}

}

// *********************** SYNC_WRITE METHODS *************************** //


/**
 * Set many dynamixels with new position values in one instruction. @see syncWrite.
 * @param value_pairs  A vector of tuples, each tuple containing a dynamixel ID and a position value.
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setMultiPosition(std::vector<std::vector<int> > value_pairs)
{
	
	if (servo_series_ == 'M')
	{
		return syncWrite(value_pairs, DXL_MX_GOAL_POSITION_L, 2, 1.0);
	} 
	else if (servo_series_ == 'X')
	{
		return syncWrite(value_pairs, DXL_X_GOAL_POSITION, 4, 2.0);
	}
	else if (servo_series_ == 'P')
	{
		return syncWrite(value_pairs, DXL_PRO_GOAL_POSITION, 4, 2.0);
	}
	else
	{
		return false;
	}

	return false;

}

/**
 * Set many dynamixels with new velocity values in one instruction. @see syncWrite.
 * @param value_pairs  A vector of tuples, each tuple is a value pair containing a dynamixel ID and a profile 
 * velocity value.
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setMultiVelocity(std::vector<std::vector<int> > value_pairs)
{

	if (servo_series_ == 'M')
	{
		return syncWrite(value_pairs, DXL_MX_GOAL_SPEED_L, 2, 1.0);
	} 
	else if (servo_series_ == 'X')
	{
		return syncWrite(value_pairs, DXL_X_GOAL_VELOCITY, 4, 2.0);
	}
	else if (servo_series_ == 'P')
	{
		return syncWrite(value_pairs, DXL_PRO_GOAL_VELOCITY, 4, 2.0);
	}
	else
	{
		return false;
	}

	return false;
}

/**
 * Set many dynamixels with new profile velocity values in one instruction. @see syncWrite.
 * @param value_pairs  A vector of tuples, each tuple is a value pair containing a dynamixel ID and a profile 
 * velocity value.
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setMultiProfileVelocity(std::vector<std::vector<int> > value_pairs)
{

	if (servo_series_ == 'M')
	{
		return syncWrite(value_pairs, DXL_MX_GOAL_SPEED_L, 2, 1.0);
	} 
	else if (servo_series_ == 'X')
	{
		return syncWrite(value_pairs, DXL_X_PROFILE_VELOCITY, 4, 2.0);
	}
	else if (servo_series_ == 'P')
	{
		return syncWrite(value_pairs, DXL_PRO_GOAL_VELOCITY, 4, 2.0);
	}
	else
	{
		return false;
	}

	return false;
}

/**
 * Set many dynamixels with new torque enabled values in one instruction. @see syncWrite.
 * @param value_pairs  A vector of tuples, each tuple containing a dynamixel ID and a torque enabled value (1 or 0).
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setMultiTorqueEnabled(std::vector<std::vector<int> > value_pairs)
{

	if (servo_series_ == 'M')
	{
		return syncWrite(value_pairs, DXL_MX_TORQUE_ENABLE, 1, 1.0);
	} 
	else if (servo_series_ == 'X')
	{
		return syncWrite(value_pairs, DXL_X_TORQUE_ENABLE, 1, 2.0);
	}
	else if (servo_series_ == 'P')
	{
		return syncWrite(value_pairs, DXL_PRO_TORQUE_ENABLE, 1, 2.0);
	}
	else
	{
		return false;
	}

	return false;
}

/**
 * Set many dynamixels with new torque values in one instruction. @see syncWrite.
 * @param value_pairs  A vector of tuples, each tuple is a value pair containing a dynamixel ID and a profile 
 * velocity value.
 * @return True on comm success, false otherwise.
 */
bool DynamixelDriver::setMultiTorque(std::vector<std::vector<int> > value_pairs)
{

	if (servo_series_ == 'M')
	{
		return syncWrite(value_pairs, DXL_MX_GOAL_TORQUE_L, 2, 1.0);
	} 
	else if (servo_series_ == 'X')
	{
		return syncWrite(value_pairs, DXL_X_GOAL_CURRENT, 2, 2.0);
	}
	else if (servo_series_ == 'P')
	{
		return syncWrite(value_pairs, DXL_PRO_GOAL_TORQUE, 2, 2.0);
	}
	else
	{
		return false;
	}

	return false;
}


// ********************** Protected Write Methods *********************** //


/** 
 * Performs the sync write for each protocol. A sync write is a broadcast instruction on a bus that commands a list
 * of dynamixels to write a value into a specified address (the value written can be different for each dynamixel
 * but the address is universal). This can be used to update a parameter (say goal position) for many dynamixels, 
 * each with a unique value, all in one instruction.
 * @param value_pairs A vector of tuples, each tuple containing a dynamixel ID and a write value.
 * @param address The address value in the control table the dynamixels will write to
 * @param length The number of bytes to write
 * @param protocol The protocol version to use
 */
bool DynamixelDriver::syncWrite(std::vector<std::vector<int> > value_pairs, uint32_t address, uint32_t length, 
		float protocol) 
{
	uint8_t dxl_comm_result;
	bool success;
	dynamixel::PacketHandler *pHandle;

	//This prevents a scoping issue
	if (protocol == 1.0)
	{
		pHandle = packetHandlerP1_;
	}
	else if (protocol == 2.0)
	{
		pHandle = packetHandlerP2_;
	}
	else
	{
		return false;
	}


    // Initialize GroupSyncWrite instance
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, pHandle, address, length);

	//Add parameters
	for (int i = 0; i < value_pairs.size(); i++)
	{	
		groupSyncWrite.addParam(value_pairs[i][0], (uint8_t*) &value_pairs[i][1]);
	}

	//Transmit packet and check success
	dxl_comm_result = groupSyncWrite.txPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		return false;
	}

	return true;
}


}