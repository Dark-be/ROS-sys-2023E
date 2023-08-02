cmake_minimum_required(VERSION 3.0.2)
project(servo_pkg)

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

file(GLOB hdrs ./src/servo_driver/inc/*.h)
file(GLOB srs ./src/servo_driver/*.cpp)

message(STATUS "hdrs: ${hdrs}")
message(STATUS "srs: ${srs}")

include_directories(
  include
  ./src/servo_driver/inc/
  ${catkin_INCLUDE_DIRS}
)

add_executable(servo_node src/servo_node.cpp ${srs})
target_link_libraries(servo_node
  ${catkin_LIBRARIES}
)
