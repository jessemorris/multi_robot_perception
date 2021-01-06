cmake_minimum_required(VERSION 3.0.2)
project(rostk_plotting)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  visualization_msgs
  roscpp
  rospy
  std_msgs
  rostk_pyutils
  cv_bridge
  sensor_msgs
)
catkin_python_setup()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



#add_service_files(
#  FILES
#)

#add_message_files(
#    FILES
#)

#generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs)

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

catkin_install_python(PROGRAMS
    scripts/plotting_main.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
