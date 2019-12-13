Dynamixel Config File Information
The two yaml files in this folder allow configuration of your specific robot as well as the properties of the motors in use. In general motor_data.yaml will not need to be changed, however the method for calculating the registor value conversions is outlined below for clarity.

~
- name: XM430-W210			#The name of the dynamixel
  model_number: 1030		#Model no. found in "Control Table of EEPROM Area" in the Robotis e-manual
  cpr: 4096					#Resolution, found in specifications in the Robotis e-manual
  gear_reduction: 4.367		# 1/(Profile velocity unit value) ie: 1/0.229 = 4.367
  effort_ratio: 1193		#TBC
  current_ratio: 2.69		#Motor goal current
~

controller_config.yaml contains comments which outline what each parameter is used for. In the example, publish_rate is supplied as 500, be aware that depending on your hardware (such as Raspberry Pi) it may need to be significantly lower. The Mogwae series hexapods which are based on Raspberry Pi 3b run between 25-50Hz. 

When defining servos ensure that the name property is unique across all ports, servo ID only needs to be unique on the port it runs on.