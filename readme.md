# The Dynamixel Interface Package

This package aims to provide a scalable and easily configurable interface for controlling many dynamixel's across multiple serial devices. Currently the following series of motors are supported:

- AX
- RX
- DX
- EX
- MX
- XM
- PRO
- PRO+
- P

Features:

- Easily adjustable configuration file
- Synchronous across multiple serial ports (using threaded IO)
- All dynamixel states are published to a single message
- Supports position, velocity and current* control modes.
  - In position control mode, can supply profile velocities as well

*current control not available on all models.

## INSTALLATION NOTES

This package depends on the dynamixel_sdk ROS package released by Robotis. To install the sdk:

``` bash

sudo apt install ros-$ROSDISTRO-dynamixel-sdk
 https://github.com/ROBOTIS-GIT/DynamixelSDK and place the ROS folder from the sdk into your workspace. Currently the sdk is confirmed to be compatable with the following versions of ROS: jade, indigo and kinetic.

```

Also required is for the user to have access to the dialout group (for serial communications)

```bash

sudo usermod -a -G dialout $USER

```

## USAGE NOTES

(For detailed setup instructions, consult the tutorials in the subdirectory of this package)

To use the dynamixel_interface_controller to operate servos:

- Create a controller_config.yaml with the necessary parameters for each servo (see the example in the config/ folder for an example and explanation of the various configuration options)
- Launch the dynamixel_interface_controller_node, loading in the custom config file (see the example in the launch/ folder)

During operation:

- The controller communicates using the sensor_msgs::JointState message.
- Current joint states are published to /joint_states
- Commands should be sent to /desired_joint_states
- For each control mode
  - Position Control: only position and velocity are considered in the command message, velocity controls the moving speed to the goal position of the dynamixel.
    If no velocity is specified the default is used
  - Velocity Control: only the velocity value is considered
  - Torque Control: only the effort value is considered

  - Note that all non-empty arrays must have the same length as the array of joint_names, otherwise the command is invalid

  A note on units:
  - Positions are in radians
  - Velocities are in radians/second
  - Effort is in mA of current

Dynamixel Interface Controller. Provides a simple abstracted interface that can control many dynamixels across multiple serial ports using only two topics.

## A NOTE ON FTDI USB INTERFACES FOR SERIAL COMMUINICATIONS

since 16.04, the default latency_timer value for the ftdi driver was changed from 1ms to 16ms. This value significantly affects the speed the driver will be able to communicate with the dynamixels, resulting in very low achievable loop rates. To correct this, a script is included that can change a serial port into 'low latency' mode, which requires the installation of the setserial command:

```bash

sudo apt install setserial

```

then, run the script (no roscore required) with:

```bash

rosrun dynamixel_interface set_lowlatency_mode.sh <port>

```

If setserial is not available as a command on your system, the value for the device can be set directly in the kernel:

``` bash

echo "1" | sudo tee /sys/bus/usb-serial/devices/<device>/latency_timer

```
