# Tutorial 1: Using the controller

This tutorial describes the basics of how to setup the dynamixel_interface_controller 
Tutorial Level: Beginner

## Step 1: Create a package

To begin, create a package that will allow us to create and manage our own configuration of the controller. This tutorial assumes you have an initialised catkin workspace as per the tutorial: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment. Create the package with:

```bash

cd ~/catkin_ws/src
catkin_create_pkg my_dynamixel_project dynamixel_interface roscpp std_msgs sensor_msgs

```

## Step 2: Create a controller configuration file

Now that we have a package setup, it is time to create the controller config file that will define how our controller runs. First, create a config folder within your package

```bash

roscd my_dynamixel_project
mkdir config
cd config

```

Next, create the config file. In the config folder, create a file named controller_config.yaml and copy in the following text:

```yaml

# GLOBAL OPERATION PARAMETERS
loop_rate: 100                    # desired rate for joint state updates. actual rate may be less depending on number
                                  # of dynamixels on each port
control_mode: position            # control mode, either 'position', 'velocity', or 'effort'
disable_torque_on_shutdown: true  # with this enabled the motors will switch off when the controller closes
ignore_input_velocity: false      # ignore input velocity commands in position mode (no profile velocity)
diagnostics_rate: 1               # rate to publish diagnostic information
dataport_rate: 1                  # rate to read from dynamixel external dataports

# The below values are used as global defaults and are applied for each servo unless overridden in the entry for the servo below
global_max_vel: 5.0               # maximum joint speed (rad/s) (in position or velocity control)
global_torque_limit: 1.0          # maximum motor torque for all modes, given as a fraction of rated max (0-1)

```

These values define the overall operation of the controller. In particular control_mode defines what method of control and what commands our servos will respond to. Publish rates are in hz and can be fractional as well. note that the controller may max out it's publish rate depending on how many dynamixels are being controlled, so don't always expect the actual rate to be as high as you input. The global parameters are designed to make it easy to configure multiple dynamixels that will all have the same parameters and thus let you cut down on the size of your config file. We want the servos to stop moving when the controller shuts down so we enable the disable_torque_on_shutdown flag.

Now we will define out serial ports and servos, paste the following at the end of the file:

```yaml

# PORT AND SERVO CONFIGURATIONS
ports:

  # PORT LIST
  - name: Port_1               # name for this port in config
    device: /dev/ttyUSB0       # serial device this port will communicate on
    baudrate: 1000000          # baudrate in use
    use_legacy_protocol: false # wether to use new 2.0 protocol (false) or legacy 1.0 protocol (true)
    group_read_enabled: true   # specify whether to use group comms for reading
    group_write_enabled: true  # specify whether to use group comms for writing
    servos:
        # SERVO LIST FOR THIS PORT
        - id: 1                # (ID set in servo eeprom, must be unique on this port)
          joint_name: joint_1  # (MUST BE UNIQUE ACROSS ALL PORTS)
          #
          # The three values below are mandatory, they define the orientation and zeroing of the dynamixel:
          #
          zero_pos: 2048       # initial (0 rad) servo position (in raw encoder count)
          min_pos: 0           # minimum servo position (in raw encoder count)
          max_pos: 4095        # maximum servo position, Note when MIN > MAX ROTATION IS REVERSED
          #
          # The below arguments are all optional and override the global values:
          #
          max_vel: 5.0         # maximum joint speed (rad/s) (in position or velocity control)
          torque_limit: 1.0    # maximum motor torque for all modes, given as a fraction of rated max (0-1)

        - id: 2
          joint_name: joint_2
          zero_pos: 2048
          min_pos: 0
          max_pos: 4095
          #
          # This servo doesn't have any optional values defined, the global defaults will be used
          #

```

Here we can see that we have a list within a list. The outer parameter, ports, defines a list of different port objects. This controller can handle multiple serial objects at once. note that the name of the port is just for identification. make sure you set the device, baudrate and series to match the dynamixels you are currently using. Then for each port object there is a parameter, servos, which defines a list of servos that are on that port. For this example we will control two servos on the one port. so make sure you set the ID's of the servos to match. The three compulsory values, init, min and max, define the zero position, orientation and limit of the dynamixel. these values are set in the actual raw position counts used by the servos themselves, rather than in radians. The values below them are override values and are optional, if any one of those parameters is not specified for a motor, the global is used instead.

Once you have configured the servos accordingly, save and close the File.

## Step 3: Creating the launch file

This step is easy. first, create a launch folder in your project:

```bash

roscd my_dynamixel_project
mkdir launch
cd launch

```

Then, in the launch folder, create a file named dynamixel_interface_controller.launch. paste in the follwing text:

```xml

<launch>
  <node name="dynamixel_interface_node" pkg="dynamixel_interface" type="dynamixel_interface_controller_node" output="screen">
    <rosparam command="load" file="$(find dynamixel_interface)/config/controller_config.yaml" />
  </node>
</launch>

```

That's it, all the launch file does is load the controller_config.yaml file from our package and launch the controller. Save and close the file.

## Step 4: Launching the controller

First, make sure your dynamixel's are connected and the config file matches their id's, baudrate and port. Now it's time to launch the controller, run the following command:

```bash

roslaunch my_dynamixel_project dynamixel_interface_controller.launch

```

If all goes well and your controller starts up you should see output similar to this:

```bash

[INFO] [1485821510.959546110]: Device is '/dev/ttyUSB0' with baud rate '1000000', using protocol 2
[INFO] [1485821510.960439630]: Succeeded to open the port(/dev/ttyUSB0)!
[INFO] [1485821510.962058345]: Succeeded to change the baudrate(1000000)!
[INFO] [1485821510.963828396]: Joint Name: joint_1, ID: 1, Model: MX106
[INFO] [1485821510.979844529]: Joint Name: joint_2, ID: 2, Model: MX64

```

## Step 5: Controlling the dynamixels

Now that the controller is up and running, run

```bash

rostopic list

```

You will see the following topics:

```bash

/desired_joint_state
/joint_states
/rosout
/rosout_agg

```

The current states of the servos are published on /joint_states. go ahead and run

```bash

rostopic echo -w 5 -c /joint_states

```

This will diplay the current joint states, formatted nicely for us to view. the output should look something like:

```bash

header:
 seq: 45
 stamp:
 secs: 1485821873
 nsecs: 625054383
 frame_id: ''
 name: ['joint_1', 'joint_2']
 position: [0.0000, 0.0000]
 velocity: [0.0000, 0.0000]
 effort: [0.0000, 0.0000]

```

This is the sensor_msgs::JointState message that the controller uses to send and receive joint state information. we are interested in the four arrays: name, position, velocity and effort. These form an in-order matrix of joint state information. we see that joint_1 has position 0 in the name array, thus all the values in position 0 of the other arrays correspond to joint_1. the same goes for joint_2 and position 1.

Now we want to send commands to the servos. We can do this on the command line by publishing to /desired_joint_state, go ahead and run

```bash

rostopic pub /desired_joint_state sensor_msgs/JointState '{name: ['joint_1', 'joint_2'], position: [3, -3]}' --once

```

The servos should move to the specified positions very quickly, this is because the default_joint_speed limit is quite high. To move the servos more slowly in position mode, we can also specify a velocity value, run the following:

```bash

rostopic pub /desired_joint_state sensor_msgs/JointState '{name: ['joint_1', 'joint_2'], position: [0, 0], velocity: [0.5, 0.5]}' --once

```

The servos should move the their desired positions much slower this time. Note that if you were to run the original command again, the motors would still move slowly, This is because the motors store their last defined velocity and continue to use that until it is overwritten.

That is it! The controller is always used in this manner, the only thing that changes are the configuaration in the controller_config.yaml file, and the values you write to control the dynamixels. From here the controller can be easily extended.

Next: Multi Port communication
