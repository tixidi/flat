#ifndef HIGO_ROS_
#define HIGO_ROS_

#include <vector>
#include <string>
#include <sstream>
#include <iostream> 



#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>




#include <hardware_interface/joint_state_interface.h>   //ref diffbot.h
#include <hardware_interface/joint_command_interface.h> //ref diffbot.h
#include <hardware_interface/robot_hw.h>                //ref diffbot.h
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager/controller_manager.h>      //ref diffbot.h



#include <gohi_hw_sensor_msgs/idcard_write_config.h>
#include <gohi_hw_sensor_msgs/idcard_read_config.h>
#include <gohi_hw_sensor_msgs/laxian_data.h>





// for ros headers
#include <ros/ros.h>
#include <ros/callback_queue.h>

//first modify**************************
// for hf link and transport
// #include <gohi_hw/transport.h>
// #include <gohi_hw/transport_serial.h>
#include <gohi_hw/transport_tcp.h>

// #include <hf_link.h>
#include <hf_link_modbus.h>

#include <gohi_hw/HIGO_AP.h>





class HIGO_ROS : public  hardware_interface::RobotHW
{

public:
	HIGO_ROS(ros::NodeHandle &nh, std::string url, std::string config_addr);
    double getFreq()const
    {
       	return controller_freq_;
    }
	void mainloop();

private:
	//communication with embeded system
	HIGO_AP higo_ap_;
	ros::NodeHandle nh_;
	ros::CallbackQueue queue_;
	// publish the robot state for diagnose system
	ros::Publisher robot_state_publisher_;
	ros::Publisher robot_cmd_publisher_;
	ros::Publisher stair_cmd_publisher_;
	ros::Publisher roll_cmd_publisher_;
	ros::Publisher idcard_read_config_publisher_;
	ros::Publisher laxian_data_publisher_;
	ros::Subscriber idcard_write_config_subscriber_;



	ros::ServiceServer getparam_srv_;
	ros::ServiceServer setparam_srv_;

	//parameter list
	std::string base_mode_;
	bool with_arm_;
	double controller_freq_;

	//hardware resource

	gohi_hw_sensor_msgs::idcard_write_config idcard_write_config_;
	gohi_hw_sensor_msgs::idcard_read_config  idcard_read_config_;
	gohi_hw_sensor_msgs::laxian_data  laxian_data_;
	unsigned char idcard_write_flag;
	unsigned char idcard_read_flag;
	
	
	




	inline void writeBufferUpdate()
	{




	}

	inline void readBufferUpdate()
	{









	}
	 void idcard_write_config_callback(const gohi_hw_sensor_msgs::idcard_write_config& msg);
};




#endif
