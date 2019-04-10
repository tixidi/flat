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


#include <gohi_hw_monitor_msgs/robot_state.h>
#include <gohi_hw_monitor_msgs/roll_config.h>
#include <gohi_hw_monitor_msgs/stair_config.h>
#include <gohi_hw_monitor_msgs/idcard_read_config.h>
#include <gohi_hw_monitor_msgs/idcard_write_config.h>
#include <gohi_hw_monitor_msgs/laser_range_config.h>
#include <gohi_hw_monitor_msgs/robot_desire_point_config.h>
#include <gohi_hw_monitor_msgs/sick_range.h>
#include <gohi_hw_monitor_msgs/imu_state.h>
#include <sensor_msgs/BatteryState.h>


// for ros headers
#include <ros/ros.h>
#include <ros/callback_queue.h>

// for hf link and transport

#include <gohi_hw/transport_tcp_client.h>
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

	ros::Publisher flat_cmd_publisher_;
	ros::Publisher power_cmd_publisher_;
	ros::Publisher stair_cmd_publisher_;
	ros::Publisher roll_cmd_publisher_;

	ros::Publisher idcard_write_config_publisher_;
	ros::Publisher laser_range_config_publisher_;
	ros::Publisher robot_desire_point_config_publisher_;

    ros::Subscriber idcard_read_config_subscriber_;
	ros::Subscriber sick_range_read_config_subscriber_;
	ros::Subscriber imu_state_subscriber_;
	ros::Subscriber battery_state_subscriber_;
	ros::Subscriber stair_car_state_subscriber_;
	ros::Subscriber power_car_state_subscriber_;
	ros::Subscriber flat_car_state_subscriber_;
	


	ros::ServiceServer getparam_srv_;
	ros::ServiceServer setparam_srv_;

	//parameter list
	std::string base_mode_;
	bool with_arm_;
	double controller_freq_;

	//hardware resource
	gohi_hw_monitor_msgs::robot_state robot_state;
	gohi_hw_monitor_msgs::roll_config roll_vel_cmd_config_ ;
	gohi_hw_monitor_msgs::stair_config stair_vel_cmd_config_;
			
	gohi_hw_monitor_msgs::idcard_write_config idcard_write_config_;
	gohi_hw_monitor_msgs::idcard_read_config idcard_read_config_;
	gohi_hw_monitor_msgs::laser_range_config laser_range_config_;
	gohi_hw_monitor_msgs::robot_desire_point_config robot_desire_point_config_;
	gohi_hw_monitor_msgs::imu_state imu_state_;

	

	unsigned char idcard_read_flag;
	unsigned char sick_range_read_flag;
	unsigned char imu_read_flag;

	


	inline void writeBufferUpdate()
	{
		// higo_ap_.getRobotAbstract()->expect_motor_speed.servo1 = wheel_cmd_[0];
		// higo_ap_.getRobotAbstract()->expect_motor_speed.servo2 = wheel_cmd_[1];
		// higo_ap_.getRobotAbstract()->expect_motor_speed.servo3 = wheel_cmd_[2];
		// higo_ap_.getRobotAbstract()->ask_expect_motor_speed.servo1 = wheel_cmd_[0];//同机器人速度冲突，2选1
		// higo_ap_.getRobotAbstract()->ask_expect_motor_speed.servo2 = wheel_cmd_[1];
		// higo_ap_.getRobotAbstract()->ask_expect_motor_speed.servo3 = wheel_cmd_[2];



	}

	inline void readBufferUpdate()
	{


        // robot_state.battery_voltage = higo_ap_.getRobotAbstract()->system_info.battery_voltage;
        // robot_state.cpu_temperature = higo_ap_.getRobotAbstract()->system_info.cpu_temperature;
        // robot_state.cpu_usage = higo_ap_.getRobotAbstract()->system_info.cpu_usage;
        // robot_state.system_time = higo_ap_.getRobotAbstract()->system_info.system_time;



	}
	void idcard_read_config_callback(const gohi_hw_monitor_msgs::idcard_read_config& msg);
	void laser_range_read_config_callback(const gohi_hw_monitor_msgs::sick_range& msg);
	void imu_state_read_callback(const gohi_hw_monitor_msgs::imu_state& msg);
	void battery_state_read_callback(const sensor_msgs::BatteryState& msg);
	void stair_state_read_callback(const gohi_hw_monitor_msgs::robot_state& msg);
	void flat_state_read_callback(const gohi_hw_monitor_msgs::robot_state& msg);
	void power_state_read_callback(const gohi_hw_monitor_msgs::robot_state& msg);

};




#endif
