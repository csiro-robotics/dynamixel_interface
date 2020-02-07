cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_interface_controller)

################################################################################
# Packages
################################################################################

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  dynamixel_interface_driver
  dynamixel_sdk
  roscpp
  roslib
  message_generation
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml-cpp REQUIRED yaml-cpp)

# There's no version hint in the yaml-cpp headers, so get the version number
# from pkg-config.

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  DataPort.msg
  ServoState.msg
  ServoMode.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



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
   CATKIN_DEPENDS roscpp sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_definitions(-g -O0)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_executable(dynamixel_interface_controller_node src/dynamixel_interface_controller.cpp)

add_dependencies(dynamixel_interface_controller_node ${dynamixel_interface_controller_EXPORTED_TARGETS} dynamixel_interface_controller_generate_messages_cpp)

target_link_libraries(
  dynamixel_interface_controller_node
  yaml-cpp
	${catkin_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
)

#############
## Install ##
#############



#############
## Testing ##
#############


