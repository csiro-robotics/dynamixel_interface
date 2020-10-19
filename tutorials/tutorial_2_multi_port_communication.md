# Tutorial 2: Multi-Port Communications

This tutorial will show you how to configure the controller for controlling multiple dynamixels simulataneously through different serial ports. Note that this tutorial requires additional hardware in the form of a second USB2Dynamixel or equivalent TTL or RS-485 device.
Tutorial Level: Beginner
Previous Tutorial: Using the controller

## Step 1: edit the controller_config.yaml file

This tutorial assumes that you have a package setup according to the previous tutorial: Using the controller. Open the controller_config.yaml file in my_dynamixel_project, it should look similar to this:

```yaml

publish_rate: 200                 # desired rate for joint state updates. actual rate may be less depending on number
                                  # of dynamixels on each port
control_mode: Position            # control mode, either 'Position', 'Velocity', or 'Torque'
disable_torque_on_shutdown: true  # with this enabled the servos will switch off when the controller closes
echo_joint_commands: false        # debug flag to echo write commands sent to the controller, useful for measuring write rates

## The below values are used as global defaults and are applied for each servo unless overridden in the entry for the servo below

global_joint_speed: 5.0           # maximum joint speed (rad/s) (in position or velocity control)
global_torque_limit: 1.0          # maximum motor torque for all modes, given as a fraction of rated max (0-1)
global_p_gain: -1.0               # proportional gain value (values > 0 are set, -1.0 indicates to leave on default)
global_i_gain: -1.0               # integral gain value (values > 0 are set, -1.0 indicates to leave on default)
global_d_gain: -1.0               # derivative gain value (values > 0 are set, -1.0 indicates to leave on default)

## PORT AND SERVO CONFIGURATIONS
ports:
  # PORT LIST
  - name: Port_1                  # name for this port in config
    device: /dev/ttyUSB0          # serial device this port will communicate on
    baudrate: 2000000             # baudrate in use (see dynamixel_driver.h for valid values)
    series: MX                    # motor series in use, must be 'MX', 'XM', or 'PRO'
    servos:
    # SERVO LIST FOR THIS PORT
        - id: 1                   # (ID set in servo eeprom, must be unique on this port)
          joint_name: joint_1     # (MUST BE UNIQUE ACROSS ALL PORTS)
          #
          # The three values below are mandatory, they define the orientation and zeroing of the dynamixel:
          #
          init: 2048              # initial (0 rad) servo position (in raw encoder count)
          min: 0                  # minimum servo position (in raw encoder count)
          max: 4095               # maximum servo position, Note when MIN > MAX ROTATION IS REVERSED
          #
          # The below arguments are all optional and override the global values:
          #
          joint_speed: 5.0        # maximum joint speed (rad/s) (in position or velocity control)
          torque_limit: 1.0       # maximum motor torque for all modes, given as a fraction of rated max (0-1)
          p_gain: -1.0            # proportional gain value (values > 0 are set, -1.0 indicates to leave on default)
          i_gain: -1.0            # integral gain value (values > 0 are set, -1.0 indicates to leave on default)
          d_gain: -1.0            # derivative gain value (values > 0 are set, -1.0 indicates to leave on default)

        - id: 2
          joint_name: joint_2
          init: 2048
          min: 0
          max: 4095
          #
          # This servo doesn't have any optional values defined, the global defaults will be used
          #

```

Adding a second serial port is very very easy, simply paste in the following at the end of the file:

```yaml

  - name: Port_2
    device: /dev/ttyUSB1
    baudrate: 3000000
    use_legacy_protocol: false
    group_read_enabled: true
    group_write_enabled: true
    servos:
        - id: 1               # id only needs to be unique for each port and so id 1 can be reused here
          joint_name: joint_3 # name DOES have to be unique though, so we continue the naming scheme
          init: 1000
          min: 1000
          max: 3000

        - id: 2
          joint_name: joint_4
          init: 2048
          min: 0
          max: 4095

```

Here we can see we have defined a second serial port. this serial port can be configured differently to the first one, with different baudrate and even motor series. It is also important to note that as this is an entirely separate serial bus, the ID's on it can be configured independently from the first port. The only thing that must remain globally unique is the joint name, as this is used to identify the servo when sending and receiving commands. For something different this time, go ahead and change the control_mode to Velocity mode by editing the control_mode parameter at the top of the file. You may also have to change the configuration of motors to suit the hardware you have available. Once that is done, save and close the file.

That is all the changes we needed to make to define new motors, ports and switch the control mode. The controller is designed so that, through this file, it can be quickly configured to whatever application you need. We also do not need to edit the launch file.

## Step 2: Launching and controlling the servos

Launch the controller the same way we did in the first tutorial:

```bash

roslaunch my_dynamixel_project dynamixel_interface_controller.launch

```

Now, as before, there will be the same topics available to us. Go ahead and run

```bash

rostopic echo -w 5 -c /joint_states

```

You should see, if your configuration matches the one above, the following output:

```bash

header:
 seq: 84
 stamp:
 secs: 1485821873
 nsecs: 625054383
 frame_id: ''
 name: ['joint_1', 'joint_2', 'joint_3', 'joint_4']
 position: [0.0000, 0.0000, 0.0000, 0.0000]
 velocity: [0.0000, 0.0000, 0.0000, 0.0000]
 effort: [0.0000, 0.0000, 0.0000, 0.0000]

```

And to publish a command, we can do the following:

```bash

rostopic pub /desired_joint_state sensor_msgs/JointState '{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4'], velocity: [1.0, 1.0, 1.0, 1.0]}' --once

```

This will command all the motors to move at a velocity of 1 rad/s. As you can see, all joint state information from all servos across all ports is published into the one message simultaneously, this makes it very easy to control any configuration of motors as you never need to worry about which motors are on which port, as long as you send the right value to the right joint_name, the controller takes care of the rest!

That's it! From here it should be straightforward to configure this controller for whatever application you need.
