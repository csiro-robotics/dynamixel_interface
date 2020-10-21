/// @file    dynamixel_interface_main.cpp
/// @brief   entry point for the ros dynamixel interface
///
/// @author  Tom Molnar (tom.molnar@data61.csiro.au)
/// @date    November 2020
/// @version 0.0.1
///
/// CSIRO Autonomous Systems Laboratory
/// Queensland Centre for Advanced Technologies
/// PO Box 883, Kenmore, QLD 4069, Australia
///
/// (c) Copyright CSIRO 2020
///
/// All rights reserved, no part of this program may be used
/// without explicit permission of CSIRO

#include <dynamixel_interface/dynamixel_interface_controller.h>
#include <ros/ros.h>

/// main
int main(int argc, char **argv)
{
  // ros initialise
  ros::init(argc, argv, "dynamixel_interface_node");

  // create controller
  dynamixel_interface::DynamixelInterfaceController controller;

  // parse params
  if (!controller.parseParameters())
  {
    return 1;
  }

  // intialise
  if (!controller.initialise())
  {
    return 2;
  }

  // initialise rate controller
  ros::Rate rate(controller.getLoopRate());

  // Loop and process
  while (ros::ok())
  {
    controller.loop();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
