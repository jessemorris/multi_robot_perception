cmake_minimum_required(VERSION 3.0.2)
project(vdo_slam_tests)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  visualization_msgs
  image_transport
  roscpp
  rospy
  std_msgs
  rostk_pyutils
  python_service_starter
  cv_bridge
  flow_net
  mask_rcnn
  mono_depth_2
  sensor_msgs
  vision_msgs
)
catkin_python_setup()

find_package(OpenCV)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
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

add_executable(vdo_orb_test_node
		src/vdo_orb_test.cpp
		)

target_link_libraries(vdo_orb_test_node
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
    -lvdo_slam
		)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(image_preprocessing_node
		  ${catkin_EXPORTED_LIBRARIES}
		  ${catkin_LIBRARIES}
		  ${OpenCV_LIBS})
endif()


## Kitti GT Depth Node ######
add_executable(kitti_gt_depth_node
		src/kitti_gt_depth.cpp
		)

target_link_libraries(kitti_gt_depth_node
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
    -lvdo_slam
		)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(kitti_gt_depth_node
		  ${catkin_EXPORTED_LIBRARIES}
		  ${catkin_LIBRARIES}
		  ${OpenCV_LIBS})
endif()

install(DIRECTORY
    #controller
    #launch
    #flow_net
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
    scripts/kitti_comparison_mono.py
    scripts/kitti_comparison_flow.py
    scripts/kitti_comparison_mask.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
