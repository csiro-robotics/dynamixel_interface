
Dynamixel Controller. Provides a simple abstracted interface that can control many dynamixels across multiple serial ports using only two topics.

To use this controller with your robot:

1. In the config folder of your robot's package:
	- Create a controller_config.yaml with the necessary parameters for each motor
	- see the example in the config folder here for the correct format

2. In the launch folder of your robot's package:
	- Create a dynamixel_controller.launch file:
	- Follow the same format as the example provided in the launch folder in this package
	- Make sure said launch file loads the controller_config.yaml from your robot's package and NOT the default one included here.

3. In the top level launch file for your robot:
	- launch the file created in step 2 BEFORE launching any higher level controllers
	
4. During operation:

	- The controller communicates using the sensor_msgs::JointState message.
	- Current joint states are published to /joint_states
	- Commands should be sent to /desired_joint_states

	A note on units:
	- Positions are in radians
	- Velocities are in radians/second
	- Efforts are a decimal fraction of the maximum torque for each servo