cmake_minimum_required(VERSION 2.8.3)
project(mtc_logic)
add_definitions(-std=gnu++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  genmsg
  tf
)

## Find yaml-cpp system package
find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES datmo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}   # <-- add this for yaml-cpp headers
  ../common_src
  ./src
)

set(COMMON_SRCS
    ../common_src/globals.cpp
    ../common_src/globals.h
    ../common_src/state.cpp
    ../common_src/state.h
    ../common_src/state_machine.cpp
    ../common_src/state_machine.h
    ../common_src/loco.cpp
    ../common_src/loco.h
)

add_executable(mtc_l_decision_node src/decision_node.cpp src/point.h src/point.cpp ${COMMON_SRCS})

target_link_libraries(mtc_l_decision_node
  ${catkin_LIBRARIES}
  yaml-cpp          # <-- link yaml-cpp
)
