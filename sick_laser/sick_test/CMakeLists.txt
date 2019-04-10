cmake_minimum_required(VERSION 2.8.3)
project(lms1xx)



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
  sick_msgs

)

find_package(console_bridge REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gohi_hw
#  CATKIN_DEPENDS controller_manager geometry_msgs hardware_interface realtime_tools roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  include/LMS1xx/
  ${console_bridge_INCLUDE_DIRS}
)


add_executable(LMS1xx_node src/LMS1xx_node.cpp src/LMS1xx.cpp)
add_dependencies(LMS1xx_node  sick_msgs_gencpp)
target_link_libraries(LMS1xx_node  GL GLU glut ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})


