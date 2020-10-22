# Copyright (c) 2020
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Tom Molnar
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_interface)

# C++ standards setup.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

################################################################################
# Packages
################################################################################

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  roscpp
  xmlrpcpp
  roslib
  message_generation
  sensor_msgs
  std_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml-cpp REQUIRED yaml-cpp)
if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  DataPort.msg
  DataPorts.msg
  ServoDiag.msg
  ServoDiags.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES dynamixel_interface_controller
   CATKIN_DEPENDS roscpp roslib xmlrpcpp sensor_msgs std_msgs dynamixel_sdk message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Library
add_library(dynamixel_interface src/dynamixel_interface_controller.cpp src/dynamixel_interface_driver.cpp)
add_dependencies(dynamixel_interface ${dynamixel_interface_EXPORTED_TARGETS})
target_include_directories(dynamixel_interface
  PRIVATE
    include
  )
target_include_directories(dynamixel_interface SYSTEM
  PRIVATE
    "${catkin_INCLUDE_DIRS}"
    "${yaml-cpp_INCLUDE_DIRS}"
  )
target_link_libraries(dynamixel_interface
  PRIVATE
    yaml-cpp
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
  )

add_executable(dynamixel_interface_controller_node src/dynamixel_interface_main.cpp)
add_dependencies(dynamixel_interface_controller_node ${dynamixel_interface_EXPORTED_TARGETS} dynamixel_interface_generate_messages_cpp)
target_include_directories(dynamixel_interface_controller_node
  PRIVATE
    include
)
target_include_directories(dynamixel_interface_controller_node SYSTEM
  PRIVATE
    "${catkin_INCLUDE_DIRS}"
    "${yaml-cpp_INCLUDE_DIRS}"
)
target_link_libraries(dynamixel_interface_controller_node
  PRIVATE
    dynamixel_interface
    yaml-cpp
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS dynamixel_interface dynamixel_interface_controller_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install launch and config info for running
install(DIRECTORY launch config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)