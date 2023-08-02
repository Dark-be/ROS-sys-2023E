cmake_minimum_required(VERSION 3.0.2)
project(control_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_pkg
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
find_package(OpenCV 4 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
