cmake_minimum_required(VERSION 2.8.3)
project(naze_ros)

find_package(catkin REQUIRED COMPONENTS
  relative_nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  cmake_modules
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/gains.cfg
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread dynamic_reconfigure
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

## Declare a cpp library
 add_library(serial_lib
   src/serial.cc
   include/serial/serial.h
   include/serial/v8stdint.h
   src/impl/unix.cc
   src/impl/list_ports/list_ports_linux.cc
 )
target_link_libraries(serial_lib rt pthread)

## Declare a cpp executable
add_executable(naze_ros_node 
    src/naze_ros_node.cpp
    src/naze_ros.cpp
    src/msp.cpp
    src/mspdata.cpp
    include/naze_ros/naze_ros.h
    include/serial/msp.h
    include/serial/mspdata.h
)

## Add cmake target dependencies of the executable/library
add_dependencies(naze_ros_node ${PROJECT_NAME}_gencfg relative_nav_msgs_generate_messages_cpp)
add_dependencies(naze_ros_node serial_lib)

target_link_libraries(naze_ros_node serial_lib
  ${catkin_LIBRARIES}
)
