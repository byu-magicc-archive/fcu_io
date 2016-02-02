cmake_minimum_required(VERSION 2.8.3)
project(fcu_io)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  roscpp
  rospy
  std_msgs
  cmake_modules
  dynamic_reconfigure
  tf
)

add_message_files(
  FILES
  GPS.msg
  Command.msg
)

generate_messages( 
  DEPENDENCIES
  std_msgs 
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/gains.cfg
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread dynamic_reconfigure
    CATKIN_DEPENDS message_runtime
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
add_executable(fcu_io_node
    src/fcu_io_node.cpp
    src/fcu_io.cpp
    src/msp.cpp
    src/mspdata.cpp
    include/fcu_io/fcu_io.h
    include/serial/msp.h
    include/serial/mspdata.h
)

## Add cmake target dependencies of the executable/library
add_dependencies(fcu_io_node fcu_io_generate_messages_cpp)
add_dependencies(fcu_io_node serial_lib)

target_link_libraries(fcu_io_node serial_lib
  ${catkin_LIBRARIES}
)
