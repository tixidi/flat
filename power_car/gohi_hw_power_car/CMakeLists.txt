cmake_minimum_required(VERSION 2.8.3)
project(gohi_hw_power_car)

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
  power_car_msgs
  
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



 add_executable(power_car_hw_node src/main.cpp src/HIGO_ROS.cpp src/transport_tcp.cpp src/HIGO_AP.cpp include/sdk/hf_link/hf_link_modbus.cpp include/sdk/hf_link/hf_link_state_machine_modbus.cpp include/sdk/hf_link/robot_control.cpp)
 add_dependencies(power_car_hw_node  power_car_msgs_gencpp)
 target_link_libraries(power_car_hw_node ${catkin_LIBRARIES})


