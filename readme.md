
These packages aim to provide a scalable and easily configurable interface for controlling many dynamixel's across multiple serial devices. Currently the following series of motors are supported:

- MX
- XM
- PRO

Features:

- Easily adjustable configuration file
- Synchronous across multiple serial ports (using threaded IO)
- All dynamixel states are published to a single message
- Supports position, velocity and torque* control modes.
	- In position control mode, can supply profile velocities as well 


*Torque control is unavailable on the MX-28 model of dynamixel.


INSTALLATION NOTES:

This package depends on the dynamixel_sdk ROS package released by Robotis. To install download the sdk from: https://github.com/ROBOTIS-GIT/DynamixelSDK and place the ROS folder from the sdk into your workspace. Currently the sdk is confirmed to be compatable with the following versions of ROS: jade, indigo and kinetic.


USAGE NOTES:

1. In the config folder of your robot's package:
	- Create a controller_config.yaml with the necessary parameters for each servo
	- see the example in the dynamixel_controller/config/ folder for an example and explanation of the various configuration options

2. In the launch folder of your robot's package:
	- Create a dynamixel_controller.launch file:
	- Follow the same format as the example provided in the dynamixel_controller/launch/ folder in this package
	- Make sure said launch file loads the controller_config.yaml from your robot's package and NOT the default one included in dynamixel_controller.

3. In the top level launch file for your robot:
	- launch the file created in step 2 BEFORE launching any higher level controllers

4. During operation:

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
	- Efforts are a decimal fraction of the maximum torque for each servo