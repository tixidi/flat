cmake_minimum_required(VERSION 2.8.3)
project(gohi_hw_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

 add_message_files(
   FILES
   idcard_write_config.msg
   idcard_read_config.msg
   laxian_data.msg


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
