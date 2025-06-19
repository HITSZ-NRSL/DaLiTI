cmake_minimum_required(VERSION 2.8.3)
project(rebvo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# debug
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# set(CMAKE_CXX_FLAGS "-std=c++14 -fsanitize=address") # to debug heap
# debug

find_package(catkin REQUIRED COMPONENTS
  tf
  tf_conversions
  roscpp
  rospy
  rosbag
  cv_bridge
  image_transport
	
  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs

  daliti

  nodelet
)

add_message_files(
   FILES
   Keyline.msg
   EdgeMap.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
message("=====================${PCL_LIBRARY_DIRS}===========")
find_package(OpenCV REQUIRED QUIET)

include_directories( "/usr/include/eigen3" )
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rebvo_nodelet 
  CATKIN_DEPENDS nodelet cloud_msgs daliti geometry_msgs nav_msgs roscpp rospy std_msgs pcl pcl_ros tf tf_conversions
  DEPENDS EIGEN3 PCL OpenCV 
)

include_directories(
  include
  ../../../include
  include/rebvo
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  include
  ../../../rebvolib/
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

FILE(GLOB incFiles "include/rebvo/*.h")

add_library(rebvo_nodelet src/rebvo_nodelet.cpp ${incFiles})
target_link_libraries(rebvo_nodelet rebvolib  X11 v4l2 m gd lapack ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(rebvo_nodelet rebvo_generate_messages_cpp)

add_executable(rebvo src/rebvo_node.cpp ${incFiles})
target_link_libraries(rebvo rebvolib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_subdirectory(isamlib)
link_libraries(isamlib)






