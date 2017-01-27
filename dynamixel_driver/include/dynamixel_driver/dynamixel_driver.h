/**
 * @file   dynamixel_driver.h
 * @author Tom Molnar (Tom.Molnar@data61.csiro.au)
 * @date   January, 2017
 * @brief  Defines the hardware abstraction methods for communicating with dynamixels
 */


#ifndef DYNAMIXEL_DRIVER_H_
#define DYNAMIXEL_DRIVER_H_

#include <stdint.h>

#include <map>
#include <string>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace dynamixel_driver 
{

/**
 * Provides the handling of the low level communications between the 
 * dynamixels and the controller
 */
class DynamixelDriver
{
public:

    /**
     * Constructor. Initialises port and packet handling objects and sets the 
     * baud rate.
     * @param device  The serial port to connect to
     * @param baud    The baud rate to use
     * @param series  The servo series in use (MX, XM or Pro)  
     */
    DynamixelDriver(std::string device, int baud, std::string series);

    /**
     * Destructor. Closes and releases serial port.
     */
    ~DynamixelDriver();

    /**
     * Ping the specified id, used to check if a dynamixel with that ID is on the bus
     * @param servo_id The ID to ping on the bus.
     * @return True if a dynamixel responds, false otherwise.
     */
    bool ping(int servo_id);
    
    // ************************************ GETTERS ***************************************** //


    /**
     * Retrieves the model number from the dynamixel's eeprom
     * @param servo_id The ID of the servo to retrieve from
     * @param model_number Stores the model_number returned
     * @return True on comm success, false otherwise.
     */
    bool getModelNumber(int servo_id, uint16_t& model_number);

    /**
     * Retrieves the model info values from the dynamixel's eeprom. Pro and XM series only
     * @param servo_id The ID of the servo to retrieve from
     * @param model_info Stores the model info returned
     * @return True on comm success with pro or xm series, false otherwise.
     */
    bool getModelInfo(int servo_id, uint32_t& model_info);

    /**
     * Retrieves the firmware version number from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param firmware_version Stores the firmware_version returned
     * @return True on comm success, false otherwise.
     */
    bool getFirmwareVersion(int servo_id, uint8_t& firmware_version);

    /**
     * Retrieves the baud rate from the dynamixel's eeprom. @see setBaudRate
     * @param servo_id The ID of the servo to retrieve from
     * @param baud_rate Stores the baud_rate returned
     * @return True on comm success, false otherwise.
     */
    bool getBaudRate(int servo_id, uint8_t& baud_rate);

    /**
     * Retrieves the return delay time from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param return_delay_time Stores the value returned
     * @return True on comm success, false otherwise.
     */
    bool getReturnDelayTime(int servo_id, uint8_t& return_delay_time);


    /**
     * Retrieves the operating_mode from the dynamixel's eeprom. XM and Pro series only
     * @param servo_id The ID of the servo to retrieve from
     * @param operating_mode Stores the value returned
     * @return True on comm success with XM or Pro series, false otherwise.
     */
    bool getOperatingMode(int servo_id, uint8_t &operating_mode);
    
    /**
     * Retrieves the angle limits from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param min_angle_limit Stores the min angle limit returned
     * @param max_angle_limit Stores the max angle limit returned
     * @return True on comm success, false otherwise.
     */
    bool getAngleLimits(int servo_id, uint32_t& min_angle_limit, uint32_t& max_angle_limit);

    /**
     * Retrieves the maximum angle limit from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param angle Stores the value returned
     * @return True on comm success, false otherwise.
     */
    bool getMaxAngleLimit(int servo_id, uint32_t& angle);

    /**
     * Retrieves the minimum angle limit from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param angle Stores the value returned
     * @return True on comm success, false otherwise.
     */
    bool getMinAngleLimit(int servo_id, uint32_t& angle);
    
    /**
     * Retrieves the voltage limits from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param min_voltage_limit Stores the min angle limit returned
     * @param max_voltage_limit Stores the max angle limit returned
     * @return True on comm success, false otherwise.
     */
    bool getVoltageLimits(int servo_id, float& min_voltage_limit, float& max_voltage_limit);

    /**
     * Retrieves minimum voltage limit from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param min_voltage_limit Stores the value returned
     * @return True on comm success, false otherwise.
     */
    bool getMinVoltageLimit(int servo_id, float& min_voltage_limit);

    /**
     * Retrieves the maximum voltage from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param max_voltage_limit Stores the value returned
     * @return True on comm success, false otherwise.
     */
    bool getMaxVoltageLimit(int servo_id, float& max_voltage_limit);
    
    /**
     * Retrieves the maximum temperature limit from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param max_temperature Stores the value returned
     * @return True on comm success, false otherwise.
     */
    bool getTemperatureLimit(int servo_id, uint8_t& max_temperature);

    /**
     * Retrieves the maximum torque limit from the dynamixel's eeprom.
     * @param servo_id The ID of the servo to retrieve from
     * @param max_torque Stores the value returned
     * @return True on comm success, false otherwise.
     */
    bool getMaxTorque(int servo_id, uint16_t& max_torque);


    /**
     * Retrieves the torque enabled value from the dynamixel's ram.
     * @param servo_id The ID of the servo to retrieve from
     * @param torque_enabled Stores the status of torque enable
     * @return True on comm success, false otherwise.
     */
    bool getTorqueEnabled(int servo_id, bool& torque_enabled);

    /**
     * Retrieves the current target position from the dynamixel's ram.
     * @param servo_id The ID of the servo to retrieve from
     * @param target_position Stores the value returned
     * @return True on comm success, false otherwise.
     */    
    bool getTargetPosition(int servo_id, int32_t& target_position);


    /**
     * Retrieves the current target_velocity from the dynamixel's ram.
     * @param servo_id The ID of the servo to retrieve from
     * @param target_velocity Stores the value returned
     * @return True on comm success, false otherwise.
     */    
    bool getTargetVelocity(int servo_id, int32_t& target_velocity);

    /**
     * Retrieves the current position from the dynamixel's ram.
     * @param servo_id The ID of the servo to retrieve from
     * @param position Stores the value returned
     * @return True on comm success, false otherwise.
     */  
    bool getPosition(int servo_id, int32_t& position);

    /**
     * Retrieves the current velocity from the dynamixel's ram.
     * @param servo_id The ID of the servo to retrieve from
     * @param velocity Stores the value returned
     * @return True on comm success, false otherwise.
     */  
    bool getVelocity(int servo_id, int32_t& velocity);

    /**
     * Retrieves the current load value from the dynamixel's ram. In the case of XM and Pro series, this value is the 
     * same as that returned by getCurrent() as they use current to measure load. For MX this value is a percentage 
     * of the limit defined in the max torque register.
     * @param servo_id The ID of the servo to retrieve from
     * @param load Stores the value returned
     * @return True on comm success, false otherwise.
     */  
    bool getLoad(int servo_id, uint16_t& load);


    /**
     * Retrieves the current value from the dynamixel's ram.
     * @param servo_id The ID of the servo to retrieve from
     * @param current Stores the value returned
     * @return True on comm success, false otherwise.
     */  
    bool getCurrent(int servo_id, uint16_t& current);

    /**
     * Retrieves the current voltage value from the dynamixel's ram.
     * @param servo_id The ID of the servo to retrieve from
     * @param voltage Stores the value returned
     * @return True on comm success, false otherwise.
     */  
    bool getVoltage(int servo_id, float& voltage);

    /**
     * Retrieves the current temperature value from the dynamixel's ram.
     * @param servo_id The ID of the servo to retrieve from
     * @param temperature Stores the value returned
     * @return True on comm success, false otherwise.
     */  
    bool getTemperature(int servo_id, uint8_t& temperature);
    

    /** 
     * Retrieves arbitrary register readings from the Dynamixel, can be used in cases where provided getters are
     * insufficient or to read multiple contiguous values at once.
     * @param servo_id The ID of the servo to retrieve from
     * @param address The address value in the control table the dynamixel will start reading from
     * @param length The number of bytes to read consecutively from the control table
     * @param response Array to store the raw dynamixel response.
     */
    bool readRegisters(int servo_id, uint32_t address, uint32_t length, uint8_t *response);


    // *********************** BULK_READ METHODS *************************** //


    /* Bulk Reads the following values in one instruction
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
    bool getBulkStateInfo(std::vector<int> *servo_ids,
                           std::map<int, std::vector<int32_t> >  *responses);

   
    // **************************** SETTERS ******************************** //

    /**
     * Writes a new ID value to the Dynamixel's eeprom. The value MUST be unique or all dynamixel's
     * with the same ID on a bus will fail to respond.
     * @param servo_id The ID of the servo to write to
     * @param new_id The new ID to set this dynamixel to
     * @return True on comm success iff new_id not already in use, false otherwise.
     */  
    bool setId(int servo_id, uint8_t new_id);

    /**
     * Writes a new baud rate value to the dynamixels eeprom.
     * @param servo_id The ID of the servo to write to
     * @param baud_rate The new baud rate for the dynamixel. Values are defined as follows:
     *  - Value = 0-249: Baud = 2Mbps/ (value + 1)
     *  - Value = 250: Baud = 2.25Mbps
     *  - Value = 251: Baud = 2.5Mbps
     *  - Value = 252: Baud = 3Mbps
     *  - Value = 253,254: INVALID, DO NOT SET
     * @return True on comm success and valid baud rate, false otherwise.
     */  
    bool setBaudRate(int servo_id, uint8_t baud_rate);

    /**
     * Writes a new return delay time value to the dynamixels eeprom.
     * @param servo_id The ID of the servo to write to
     * @param return_delay_time The value to write to the register
     * @return True on comm success, false otherwise.
     */  
    bool setReturnDelayTime(int servo_id, uint8_t return_delay_time);

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
    bool setOperatingMode(int servo_id, uint8_t operating_mode);
    
    /**
     * Sets the dynamixel to reverse it's response directions. not this also moves the zero encoder value by 
     * 180 degrees.
     * @param servo_id The ID of the servo to write to
     * @param reverse boolean dynamixel direction value (true for reversed, false for normal)
     */
    bool setReverseDirection(int servo_id, bool reverse);


    /**
     * Sets the minimum and maximum angle limits for the dynamixel
     * @param servo_id The ID of the servo to write to
     * @param min_angle the minimum angle limit (in encoder values)
     * @param max_angle the maximum angle limit (in encoder values)
     * @return True on comm success, false otherwise.
     */  
    bool setAngleLimits(int servo_id, int32_t min_angle, int32_t max_angle); 

     /**
     * Sets the minimum angle limit for the dynamixel
     * @param servo_id The ID of the servo to write to
     * @param angle The minimum angle limit (in encoder values)
     * @return True on comm success, false otherwise.
     */  
    bool setMinAngleLimit(int servo_id, int32_t angle);

    /**
     * Sets the maximum angle limit for the dynamixel
     * @param servo_id The ID of the servo to write to
     * @param angle The maximum angle limit (in encoder values)
     * @return True on comm success, false otherwise.
     */  
    bool setMaxAngleLimit(int servo_id, int32_t angle);
   
    /**
     * Sets the maximum temperature limit for the dynamixel
     * @param servo_id The ID of the servo to write to
     * @param max_temperature the maximum temperature limit
     * @return True on comm success, false otherwise.
     */  
    bool setTemperatureLimit(int servo_id, uint8_t max_temperature);

    /**
     * Sets the maximum torque limit for the dynamixel
     * @param servo_id The ID of the servo to write to
     * @param max_torque the maximum torque limit
     * @return True on comm success, false otherwise.
     */  
    bool setMaxTorque(int servo_id, uint16_t max_torque);

    /**
     * Sets the torque enable register of the dynamixel. This value defines the on/off state of the servo.
     * @param servo_id The ID of the servo to write to
     * @param on The state of the servo (true = on, false = off).
     * @return True on comm success, false otherwise.
     */  
    bool setTorqueEnabled(int servo_id, bool on);

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
    bool setPIDGains(int servo_id, uint8_t operating_mode, double p_gain, double i_gain, double d_gain);

    /**
     * Sets the proportional gain value for the position control mode if available. @see setPIDGains
     * @param servo_id The ID of the servo to write to
     * @param gain The proportional gain value to write
     * @return True on comm success, false otherwise.
     */
    bool setPositionProportionalGain(int servo_id, uint16_t gain);

    /**
     * Sets the integral gain value for the position control mode if available. @see setPIDGains
     * @param servo_id The ID of the servo to write to
     * @param gain The integral gain value to write
     * @return True on comm success, false otherwise.
     */
    bool setPositionIntegralGain(int servo_id, uint16_t gain);

    /**
     * Sets the derivative gain value for the position control mode if available. @see setPIDGains
     * @param servo_id The ID of the servo to write to
     * @param gain The derivative gain value to write
     * @return True on comm success, false otherwise.
     */
    bool setPositionDerivativeGain(int servo_id, uint16_t gain);

    /**
     * Sets the proportional gain value for the velocity control mode if available. @see setPIDGains
     * @param servo_id The ID of the servo to write to
     * @param gain The proportional gain value to write
     * @return True on comm success, false otherwise.
     */
    bool setVelocityProportionalGain(int servo_id, uint16_t gain);

    /**
     * Sets the integral gain value for the velocity control mode if available. @see setPIDGains
     * @param servo_id The ID of the servo to write to
     * @param gain The integral gain value to write
     * @return True on comm success, false otherwise.
     */
    bool setVelocityIntegralGain(int servo_id, uint16_t gain);

    /**
     * Sets the derivative gain value for the velocity control mode if available. @see setPIDGains
     * @param servo_id The ID of the servo to write to
     * @param gain The derivative gain value to write
     * @return True on comm success, false otherwise.
     */
    bool setVelocityDerivativeGain(int servo_id, uint16_t gain);

    /**
     * Sets the proportional gain value for the torque control mode if available. @see setPIDGains
     * @param servo_id The ID of the servo to write to
     * @param gain The proportional gain value to write
     * @return True on comm success, false otherwise.
     */
    bool setTorqueProportionalGain(int servo_id, uint16_t gain);

    /**
     * Sets the integral gain value for the torque control mode if available. @see setPIDGains
     * @param servo_id The ID of the servo to write to
     * @param gain The integral gain value to write
     * @return True on comm success, false otherwise.
     */
    bool setTorqueIntegralGain(int servo_id, uint16_t gain);

    /**
     * Sets the derivative gain value for the torque control mode if available. @see setPIDGains
     * @param servo_id The ID of the servo to write to
     * @param gain The derivative gain value to write
     * @return True on comm success, false otherwise.
     */
    bool setTorqueDerivativeGain(int servo_id, uint16_t gain);

    /**
     * Sets the goal position of the dynamixel.
     * @param servo_id The ID of the servo to write to
     * @param position The position value to write
     * @return True on comm success, false otherwise.
     */  
    bool setPosition(int servo_id, uint32_t position);

    /**
     * Sets the goal velocity of the dynamixel.
     * @param servo_id The ID of the servo to write to
     * @param velocity The velocity value to write
     * @return True on comm success, false otherwise.
     */ 
    bool setVelocity(int servo_id, int32_t velocity);

    /**
     * Sets the profile velocity of the dynamixel. Profile velocity is how fast
     * the servo should move between positions.
     * @param servo_id The ID of the servo to write to
     * @param velocity The profile velocity value to write
     * @return True on comm success, false otherwise.
     */
    bool setProfileVelocity(int servo_id, int32_t velocity);


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
    bool writeRegisters(int servo_id, uint32_t address, uint32_t length, uint8_t *data);


    // *********************** SYNC_WRITE METHODS *************************** //

    /**
     * Set many dynamixels with new position values in one instruction. @see syncWrite.
     * @param value_pairs  A vector of tuples, each tuple containing a dynamixel ID and a position value.
     * @return True on comm success, false otherwise.
     */
    bool setMultiPosition(std::vector<std::vector<int> > value_pairs);

    /**
     * Set many dynamixels with new profile velocity values in one instruction. @see syncWrite.
     * @param value_pairs  A vector of tuples, each tuple is a value pair containing a dynamixel ID and a profile 
     * velocity value.
     * @return True on comm success, false otherwise.
     */
    bool setMultiProfileVelocity(std::vector<std::vector<int> > value_pairs);

    /**
     * Set many dynamixels with new velocity values in one instruction. @see syncWrite.
     * @param value_pairs  A vector of tuples, each tuple is a value pair containing a dynamixel ID and a profile 
     * velocity value.
     * @return True on comm success, false otherwise.
     */
    bool setMultiVelocity(std::vector<std::vector<int> > value_pairs);

    /**
     * Set many dynamixels with new torque values in one instruction. @see syncWrite.
     * @param value_pairs  A vector of tuples, each tuple is a value pair containing a dynamixel ID and a profile 
     * velocity value.
     * @return True on comm success, false otherwise.
     */
    bool setMultiTorque(std::vector<std::vector<int> > value_pairs);

    /**
     * Set many dynamixels with new torque enabled values in one instruction. @see syncWrite.
     * @param value_pairs  A vector of tuples, each tuple containing a dynamixel ID and a torque enabled value (1 or 0).
     * @return True on comm success, false otherwise.
     */
    bool setMultiTorqueEnabled(std::vector<std::vector<int> > value_pairs);

protected:

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
    bool bulkRead(std::vector<int> *servo_ids,
                       uint16_t address,
                       uint16_t length,
                       std::map<int, std::vector<uint8_t> >  *responses);     


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
    bool syncRead(std::vector<int> *servo_ids,
                       uint16_t address,
                       uint16_t length,
                       std::map<int, std::vector<uint8_t> >  *responses);

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
    bool syncWrite(std::vector<std::vector<int> > value_pairs, 
                    uint32_t address, 
                    uint32_t length,
                    float protocol);

private:
    
    /** The port handler object. The dynamixel sdk serial object. */
    dynamixel::PortHandler *portHandler_;

    /** packet handler for protocol 1.0. Provides raw response deconstruction. */
    dynamixel::PacketHandler *packetHandlerP1_;

    /** packet handler for protocol 1.0. Provides raw response deconstruction. */
    dynamixel::PacketHandler *packetHandlerP2_;

    /** The motor series this driver is communicating with */
    char servo_series_;
};

}

#endif //DYNAMIXEL_DRIVER_H_
