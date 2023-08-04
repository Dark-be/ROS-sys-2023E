cmake_minimum_required(VERSION 3.0.2)
project(image_proc)

find_package( OpenCV 4 REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  roscpp
  serial
  roslib
  rospy
  std_msgs
  tf
  serial
)


include_directories( ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} )
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
)
add_executable(image_proc src/main.cpp)
target_link_libraries(image_proc ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp )
