cmake_minimum_required(VERSION 3.0.2)
project(flow_net)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  python_service_starter
  sensor_msgs
)
catkin_python_setup()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_service_files(
  FILES
  FlowNet.srv
)

#add_message_files(
#    FILES
#)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
  #LIBRARIES flow_net
  CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)

install(DIRECTORY
    #controller
    #launch
    #flow_net
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
    scripts/flow_net_interface.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
