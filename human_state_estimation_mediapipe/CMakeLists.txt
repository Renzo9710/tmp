cmake_minimum_required(VERSION 3.0.2)
project(human_state_estimation_mediapipe)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Release mode
set(CMAKE_BUILD_TYPE Release)

# Optimization, use -O2 if any problems occur
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Suppress deprecated warnings
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  tf
  tf_conversions
)

find_package (OpenCV REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MsgMediapipeData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include extern/eigen3
  LIBRARIES human_state_estimation_mp_depth
  CATKIN_DEPENDS roscpp rospy tf cv_bridge image_transport sensor_msgs std_msgs geometry_msgs message_runtime 
  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  extern/eigen3
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library 
add_library(human_state_estimation_mp_depth STATIC
  src/human_state_estimation_mp_depth.cpp
)

target_link_libraries(human_state_estimation_mp_depth
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(human_state_estimation_mp_depth human_state_estimation_mediapipe_generate_messages_cpp)

add_executable(human_state_estimation_mp_depth_node src/human_state_estimation_mp_depth_node.cpp)

target_link_libraries(human_state_estimation_mp_depth_node
  human_state_estimation_mp_depth
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_executable(hand_tracking_node src/hand_tracking_node.cpp)
target_link_libraries(hand_tracking_node
  ${catkin_LIBRARIES}
)



#############
## Install ##
#############
catkin_install_python(PROGRAMS scripts/mediapipe_human_hand_estimation_ros.py DESTINATION  ${CATKIN_PACKAGE_BIN_DESTINATION})


# Mark executables and/or libraries for installation
install(TARGETS human_state_estimation_mp_depth human_state_estimation_mp_depth_node hand_tracking_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/ extern/eigen3/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
