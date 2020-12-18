cmake_minimum_required(VERSION 3.0.2)
project(mask_rcnn)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  rostk_pyutils
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
  MaskRcnnVisualise.srv
  MaskRcnnVdoSlam.srv
)

#add_message_files(
#    FILES
#)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
  #LIBRARIES mask_rcnn
  CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)

install(DIRECTORY
    #controller
    #launch
    #mask_rcnn
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
    scripts/mask_rcnn_interface.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
