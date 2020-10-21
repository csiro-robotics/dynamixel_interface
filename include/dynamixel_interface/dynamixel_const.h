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


namespace dynamixel_interface
{
/// Dynamixel types
enum DynamixelSeriesType
{
  kSeriesAX = 0,
  kSeriesRX = 1,
  kSeriesDX = 2,
  kSeriesEX = 3,
  kSeriesLegacyMX = 4,
  kSeriesMX = 5,
  kSeriesX = 6,
  kSeriesLegacyPro = 7,
  kSeriesP = 8,
  kSeriesUnknown = 9
};

/// Legacy control table, for older dynamixels:
/// - MX (1.0)
/// - AX
/// - RX
enum DynamixelLegacyRegisterTable
{
  // EEPROM
  kRegLegacy_ModelNumber = 0,
  kRegLegacy_FirmwareVersion = 2,
  kRegLegacy_ID = 3,
  kRegLegacy_BaudRate = 4,
  kRegLegacy_ReturnDelayTime = 5,
  kRegLegacy_CWAngleLimit = 6,
  kRegLegacy_CCWAngleLimit = 8,
  kRegLegacy_DriveMode = 10,
  kRegLegacy_TemperatureLimit = 11,
  kRegLegacy_MinVoltageLimit = 12,
  kRegLegacy_MaxVoltageLimit = 13,
  kRegLegacy_MaxTorque = 14,
  kRegLegacy_ReturnLevel = 16,
  kRegLegacy_AlarmLED = 17,
  kRegLegacy_AlarmShutdown = 18,
  kRegLegacy_MultiTurnOffset = 20,
  kRegLegacy_ResolutionDivider = 22,

  // RAM
  kRegLegacy_TorqueEnable = 24,
  kRegLegacy_LED = 25,

  // - MX (1.0) specific
  kRegLegacy_DGain = 26,
  kRegLegacy_IGain = 27,
  kRegLegacy_PGain = 28,

  // - AX/RX specific
  kRegLegacy_CWComplianceMargin = 26,
  kRegLegacy_CCWComplianceMargin = 27,
  kRegLegacy_CWComplianceSlope = 28,
  kRegLegacy_CCWComplianceSlope = 29,

  kRegLegacy_GoalPosition = 30,
  kRegLegacy_MovingSpeed = 32,
  kRegLegacy_TorqueLimit = 34,
  kRegLegacy_PresentPosition = 36,
  kRegLegacy_PresentSpeed = 38,
  kRegLegacy_PresentLoad = 40,
  kRegLegacy_PresentVoltage = 42,
  kRegLegacy_PresentTemperature = 43,
  kRegLegacy_RegisteredInstruction = 44,
  kRegLegacy_Moving = 46,
  kRegLegacy_Lock = 47,
  kRegLegacy_Punch = 48,
  kRegLegacy_RealtimeTick = 50,
  kRegLegacy_SensedCurrent = 56,
  kRegLegacy_PresentCurrent = 68,
  kRegLegacy_TorqueControlEnable = 70,
  kRegLegacy_GoalTorque = 71,
  kRegLegacy_GoalAcceleration = 73,
};


/// Standard control table, for newer models of dynamixels supporting protocol 2.0:
/// - MX (2.0)
/// - X (2.0) (excepting the XL-320, which has a unique table)
enum DynamixelStandardRegisterTable
{
  // EEPROM
  kRegStandard_ModelNumber = 0,
  kRegStandard_ModelInfo = 2,
  kRegStandard_FirmwareVersion = 6,
  kRegStandard_ID = 7,
  kRegStandard_BaudRate = 8,
  kRegStandard_ReturnDelayTime = 9,
  kRegStandard_DriveMode = 10,
  kRegStandard_OperatingMode = 11,
  kRegStandard_ShadowID = 12,
  kRegStandard_ProtocolVersion = 13,
  kRegStandard_HomingOffset = 20,
  kRegStandard_MovingThreshold = 24,
  kRegStandard_TemperatureLimit = 31,
  kRegStandard_MaxVoltageLimit = 32,
  kRegStandard_MinVoltageLimit = 34,
  kRegStandard_PWMLimit = 36,
  kRegStandard_CurrentLimit = 38,
  kRegStandard_AccelerationLimit = 40,
  kRegStandard_VelocityLimit = 44,
  kRegStandard_MaxPositionLimit = 48,
  kRegStandard_MinPositionLimit = 52,
  kRegStandard_DataPort1Mode = 56,
  kRegStandard_DataPort2Mode = 57,
  kRegStandard_DataPort3Mode = 58,
  kRegStandard_Shutdown = 63,

  // RAM
  kRegStandard_TorqueEnable = 64,
  kRegStandard_LED = 65,
  kRegStandard_StatusReturnLevel = 68,
  kRegStandard_RegisteredInstruction = 69,
  kRegStandard_HardwareErrorStatus = 70,
  kRegStandard_VelocityIGain = 76,
  kRegStandard_VelocityPGain = 78,
  kRegStandard_PositionDGain = 80,
  kRegStandard_PositionIGain = 82,
  kRegStandard_PositionPGain = 84,
  kRegStandard_Feedforward2ndGain = 88,
  kRegStandard_Feedforward1stGain = 90,
  kRegStandard_BusWatchdog = 98,
  kRegStandard_GoalPWM = 100,
  kRegStandard_GoalCurrent = 102,
  kRegStandard_GoalVelocity = 104,
  kRegStandard_ProfileAcceleration = 108,
  kRegStandard_ProfileVelocity = 112,
  kRegStandard_GoalPosition = 116,
  kRegStandard_RealtimeTick = 120,
  kRegStandard_Moving = 122,
  kRegStandard_MovingStatus = 123,
  kRegStandard_PresentPWM = 124,
  kRegStandard_PresentCurrent = 126,
  kRegStandard_PresentVelocity = 128,
  kRegStandard_PresentPosition = 132,
  kRegStandard_VelocityTrajectory = 136,
  kRegStandard_PositionTrajectory = 140,
  kRegStandard_PresentInputVoltage = 144,
  kRegStandard_PresentTemperature = 146,
  kRegStandard_DataPort1 = 152,
  kRegStandard_DataPort2 = 154,
  kRegStandard_DataPort3 = 156,
  kRegStandard_IndirectAddress1 = 168,
  kRegStandard_IndirectData1 = 224,
};

/// Control table for Dynamixel P series (new pro)
/// - Dynamixel-P
/// - Dynamixel PRO+, sometimes listed as PRO(A)
enum DynamixelProRegisterTable
{
  // EEPROM
  kRegP_ModelNumber = 0,
  kRegP_ModelInfo = 2,
  kRegP_FirmwareVersion = 6,
  kRegP_ID = 7,
  kRegP_BaudRate = 8,
  kRegP_ReturnDelayTime = 9,
  kRegP_DriveMode = 10,
  kRegP_OperatingMode = 11,
  kRegP_ShadowID = 12,
  kRegP_ProtocolType = 13,
  kRegP_HomingOffset = 20,
  kRegP_MovingThreshold = 24,
  kRegP_TemperatureLimit = 31,
  kRegP_MaxVoltageLimit = 32,
  kRegP_MinVoltageLimit = 34,
  kRegP_PWMLimit = 36,
  kRegP_CurrentLimit = 38,
  kRegP_AccelerationLimit = 40,
  kRegP_VelocityLimit = 44,
  kRegP_MaxPositionLimit = 48,
  kRegP_MinPositionLimit = 52,
  kRegP_DataPort1Mode = 56,
  kRegP_DataPort2Mode = 57,
  kRegP_DataPort3Mode = 58,
  kRegP_DataPort4Mode = 59,
  kRegP_Shutdown = 63,
  kRegP_IndirectAddress1 = 168,

  // RAM
  kRegP_TorqueEnable = 512,
  kRegP_RedLED = 513,
  kRegP_GreenLED = 514,
  kRegP_BlueLED = 515,
  kRegP_StatusReturnLevel = 516,
  kRegP_RegisteredInstruction = 517,
  kRegP_HardwareErrorStatus = 518,
  kRegP_VelocityIGain = 524,
  kRegP_VelocityPGain = 526,
  kRegP_PositionDGain = 528,
  kRegP_PositionIGain = 530,
  kRegP_PositionPGain = 532,
  kRegP_Feedforward2ndGain = 536,
  kRegP_Feedforward1stGain = 538,
  kRegP_BusWatchdog = 546,
  kRegP_GoalPWM = 548,
  kRegP_GoalCurrent = 550,
  kRegP_GoalVelocity = 552,
  kRegP_ProfileAcceleration = 556,
  kRegP_ProfileVelocity = 560,
  kRegP_GoalPosition = 564,
  kRegP_RealtimeTick = 568,
  kRegP_Moving = 570,
  kRegP_MovingStatus = 571,
  kRegP_PresentPWM = 572,
  kRegP_PresentCurrent = 574,
  kRegP_PresentVelocity = 576,
  kRegP_PresentPosition = 580,
  kRegP_VelocityTrajectory = 584,
  kRegP_PositionTrajectory = 588,
  kRegP_PresentInputVoltage = 592,
  kRegP_PresentTemperature = 594,
  kRegP_DataPort1 = 600,
  kRegP_DataPort2 = 602,
  kRegP_DataPort3 = 604,
  kRegP_DataPort4 = 606,
  kRegP_IndirectData1 = 634
};


/// Control table/register addresses for each series of dynamixel
enum DynamixelLegacyProRegisterTable
{
  // EEPROM
  kRegLegacyPro_ModelNumber = 0,
  kRegLegacyPro_ModelInfo = 2,
  kRegLegacyPro_FirmwareVersion = 6,
  kRegLegacyPro_ID = 7,
  kRegLegacyPro_BaudRate = 8,
  kRegLegacyPro_ReturnDelayTime = 9,
  kRegLegacyPro_OperatingMode = 11,
  kRegLegacyPro_LimitTemperature = 21,
  kRegLegacyPro_DownLimitVoltage = 24,
  kRegLegacyPro_UpLimitVoltage = 22,
  kRegLegacyPro_LED = 25,
  kRegLegacyPro_AccelLimit = 26,
  kRegLegacyPro_VelocityLimit = 32,
  kRegLegacyPro_MaxTorque = 30,
  kRegLegacyPro_MaxAngleLimit = 36,
  kRegLegacyPro_MinAngleLimit = 40,
  kRegLegacyPro_DataPort1Mode = 44,
  kRegLegacyPro_DataPort2Mode = 45,
  kRegLegacyPro_DataPort3Mode = 46,
  kRegLegacyPro_DataPort4Mode = 47,

  // RAM
  kRegLegacyPro_VelocityIGain = 586,
  kRegLegacyPro_VelocityPGain = 588,
  kRegLegacyPro_PositionPGain = 594,
  kRegLegacyPro_TorqueEnable = 562,
  kRegLegacyPro_GoalPosition = 596,
  kRegLegacyPro_GoalVelocity = 600,
  kRegLegacyPro_GoalTorque = 604,
  kRegLegacyPro_PresentPosition = 611,
  kRegLegacyPro_PresentVelocity = 615,
  kRegLegacyPro_PresentCurrent = 621,
  kRegLegacyPro_PresentVoltage = 623,
  kRegLegacyPro_PresentTemperature = 625,

  kRegLegacyPro_DataPort1 = 626,
  kRegLegacyPro_DataPort2 = 628,
  kRegLegacyPro_DataPort3 = 630,
  kRegLegacyPro_DataPort4 = 632,

  kRegLegacyPro_RegisteredInstruction = 890,
  kRegLegacyPro_Moving = 610,
  kRegLegacyPro_ReturnLevel = 891,
  kRegLegacyPro_HardwareErrorStatus = 892,
};


/// The different control modes available on the dynamixel servos. The values chosen for each type reflect those used
/// on the motors themselves.
enum DynamixelControlMode
{
  UNKNOWN = -1,

  kModeTorqueControl = 0,
  kModeVelocityControl = 1,
  kModePositionControl = 3,

  kModeExtendedPositionControl = 4,
  kModeCurrentBasedPositionControl = 5,

  kModePWMControl = 16

};


/// Instruction codes for various commands
enum DynamixelInstruction
{
  kInstPing = 1,
  kInstReadData = 2,
  kInstWriteData = 3,
  kInstRegWrite = 4,
  kInstAction = 5,
  kInstReset = 6,

  kInstSyncWrite = 0x83,
  kInstBulkRead = 0x92,

  kInstBroadcast = 254,

};


/// Error return codes
enum DynamixelErrorCode
{
  kErrorNoError = 0,
  // Common error codes
  kErrorOverload = 32,
  kErrorElectricShock = 16,
  kErrorMotorEncoder = 8,
  kErrorOverheating = 4,
  kErrorMotorHall = 2,
  kErrorInputVoltage = 1,
  // Legacy specific error codes
  kErrorLegacyInstruction = 64,
  kErrorLegacyChecksum = 16,
  kErrorLegacyRange = 8,
  kErrorLegacyAngleLimit = 2,

};

}  // namespace dynamixel_interface

#endif  // DYNAMIXEL_CONST_H__
