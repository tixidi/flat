cmake_minimum_required(VERSION 2.8.3)
project(gohi_hw_rev_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

 add_message_files(
   FILES
   robot_state.msg
   roll_config.msg 
   stair_config.msg
   idcard_write_config.msg
   idcard_read_config.msg
   laser_range_config.msg
   robot_desire_point_config.msg
   sick_range.msg
   laser_range_config1.msg 
   flat_car_state.msg
   brake_config.msg
   imu_state.msg
   


   #parameters.msg
 )


 #add_service_files(
 #  FILES
 #  GetParamList.srv
 #  SetParamList.srv
 #)

 generate_messages(
   DEPENDENCIES
   sensor_msgs   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_generation message_runtime sensor_msgs std_msgs 
 # DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
