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


#include <gohi_hw_idnev_msgs/idnev_state.h>
#include <gohi_hw_idnev_msgs/start_idnev.h>
#include <gohi_hw_idnev_msgs/stair_config.h>
#include <gohi_hw_idnev_msgs/idcard_write_config.h>
#include <gohi_hw_idnev_msgs/roll_config.h>
#include <gohi_hw_idnev_msgs/idcard_read_config.h>



// for ros headers
#include <ros/ros.h>
#include <ros/callback_queue.h>


// for hf link and transport
#include <gohi_hw/transport.h>
#include <gohi_hw/transport_serial.h>
// #include <hf_link.h>
#include <hf_link_modbus.h>

#include <gohi_hw/HIGO_AP.h>




class HIGO_ROS : public  hardware_interface::RobotHW
{

public:
	HIGO_ROS(ros::NodeHandle &nh, std::string url, std::string config_addr,std::string idConfig_addr);
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
	ros::Publisher flat_cmd_publisher_;
	ros::Publisher power_cmd_publisher_;
	ros::Publisher robot_cmd_publisher_;
	ros::Publisher stair_cmd_publisher_;
	ros::Publisher roll_cmd_publisher_;
	ros::Publisher idnev_state_publisher_;
	

	//subscriber the robot state for diagnose system
	ros::Subscriber idnev_state_subscriber_;
	ros::Subscriber start_idnev_state_subscriber_;	

	ros::ServiceServer getparam_srv_;
	ros::ServiceServer setparam_srv_;

	//parameter list
	std::string base_mode_;
	bool with_arm_;
	double controller_freq_;

	//hardware resource
	gohi_hw_idnev_msgs::idnev_state idnev_state_;
	gohi_hw_idnev_msgs::stair_config stair_config;
    gohi_hw_idnev_msgs::roll_config roll_config_;

	unsigned char idcard_write_flag;
	unsigned char idcard_read_flag;
	unsigned char start_idnev_cmd_flag_;
	
	




	inline void writeBufferUpdate()
	{




	}

	inline void readBufferUpdate()
	{



	}
void idcard_read_config_callback(const gohi_hw_idnev_msgs::idcard_read_config& msg);
void start_idcard_read_config_callback(const gohi_hw_idnev_msgs::start_idnev& msg);

};




#endif
