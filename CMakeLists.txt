cmake_minimum_required(VERSION 2.8.3)
project(sawyer_aim)

find_package(catkin REQUIRED COMPONENTS
	rospy
	intera_core_msgs
	gazebo_msgs
)


catkin_package(
	CATKIN_DEPENDS 
	rospy
	intera_core_msgs
	gazebo_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/load_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

