cmake_minimum_required(VERSION 2.8.3)
project(gohi_hw_rev)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  geometry_msgs
  hardware_interface
  realtime_tools
  roscpp
  std_msgs
  gohi_hw_rev_msgs
  sensor_msgs
)

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")


generate_messages(
  DEPENDENCIES std_msgs sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gohi_hw
#  CATKIN_DEPENDS controller_manager geometry_msgs hardware_interface realtime_tools roscpp std_msgs 
#  DEPENDS system_lib
)


include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  include/sdk/hf_link/
  include/sdk/robot_abstract/

)



 add_executable(gohi_hw_node_rev src/main.cpp src/HIGO_ROS.cpp src/transport_tcp_client.cpp src/transport_udp_client.cpp src/HIGO_AP.cpp include/sdk/hf_link/hf_link_modbus.cpp include/sdk/hf_link/hf_link_state_machine_modbus.cpp)
 add_dependencies(gohi_hw_node_rev  gohi_hw_rev_msgs_gencpp)
 target_link_libraries(gohi_hw_node_rev ${catkin_LIBRARIES})


