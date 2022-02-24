cmake_minimum_required(VERSION 3.0.2)
project(towr_gui_kivy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # For custom msg
  message_generation
  xpp_msgs
  xpp_states
  # For custom msg

)

add_message_files(
  FILES
  TowrCommandGUI.msg
  TowrCommand2.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  xpp_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime xpp_states
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

