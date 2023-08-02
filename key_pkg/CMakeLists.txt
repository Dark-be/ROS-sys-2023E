cmake_minimum_required(VERSION 3.0.2)
project(key_pkg)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_pkg
#  CATKIN_DEPENDS roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(key_node src/key_node.cpp)
target_link_libraries(key_node
  ${catkin_LIBRARIES}
)