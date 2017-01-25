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