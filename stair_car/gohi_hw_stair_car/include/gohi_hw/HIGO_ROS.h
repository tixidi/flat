#ifndef HIGO_ROS_
#define HIGO_ROS_

#include <vector>
#include <string>
#include <sstream>
#include <iostream> 



#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <gohi_hw/base_cmd_interface.h>
#include <gohi_hw/base_state_interface.h>


#include <hardware_interface/joint_state_interface.h>   //ref diffbot.h
#include <hardware_interface/joint_command_interface.h> //ref diffbot.h
#include <hardware_interface/robot_hw.h>                //ref diffbot.h
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager/controller_manager.h>      //ref diffbot.h


#include <stair_car_msgs/robot_state.h>
#include <stair_car_msgs/roll_config.h>
#include <stair_car_msgs/stair_config.h>
#include <stair_car_msgs/brake_config.h>
#include <stair_car_msgs/relay_state.h>
#include <stair_car_msgs/laxian_data.h>
// for ros headers
#include <ros/ros.h>
#include <ros/callback_queue.h>

// for hf link and transport
// #include <gohi_hw/transport.h>
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
	void delay(int time);


public:
	unsigned char allow_set_stair_position_flag;  //松开刹车标志
	unsigned char update_data_cmd;  //允许升降位置状态更新
	unsigned char SQ_reset_not_allow_set_position;  //如果行程开关出发，那么该标志位被置为1，接着就不执行设置位置命令
	unsigned char start_flag;
	unsigned char allow_judge_laxian_data;   //允许判断拉线数据，从而老得到差值
	unsigned char allow_loose_break;
private:
	//communication with embeded system
	HIGO_AP higo_ap_;
	ros::NodeHandle nh_;
	ros::CallbackQueue queue_;
	// publish the robot state for diagnose system
	ros::Publisher robot_state_publisher_;
	ros::Publisher relay_state_publisher_;

	ros::Subscriber stair_cmd_subscribe_;
	ros::Subscriber roll_cmd_subscribe_;
	ros::Subscriber brake_cmd_subscribe_;
	ros::Subscriber get_laxian_data_subscribe_;
	 
	boost::asio::io_service io_service;


	ros::ServiceServer getparam_srv_;
	ros::ServiceServer setparam_srv_;

	//parameter list
	std::string base_mode_;
	bool with_arm_;
	double controller_freq_;
	unsigned char roll_config_callback_flag;
	unsigned char stair_config_call_back_flag;

	unsigned char brake_config_callback_flag;
	unsigned char brake_config_callback_flag1;
	

	//hardware resource
	stair_car_msgs::robot_state robot_state;
	// stair_car_msgs::relay_state relay_state;
	stair_car_msgs::roll_config roll_config ;
	stair_car_msgs::stair_config stair_config;
	stair_car_msgs::brake_config brake_config;

    std::vector<double> wheel_pos_, wheel_vel_, wheel_eff_, wheel_cmd_;

    double x_, y_, theta_, x_cmd_, y_cmd_, theta_cmd_;
    double x_vel_, y_vel_, theta_vel_;


	int count;
	unsigned char relay_state;
	float laxian_length;
	float laxian_length_new;
	unsigned char update_stair_;
	unsigned char update_laxian_;
	unsigned char direction_down;  //为1表示方向向下    为0表示方向向上
	unsigned char direction_down_;
    // hardware_interface::PositionJointInterface servo_pos_interface_;
	//ref   ����diffbot.h 
	//ref   ����diffbot.h 
	//ref 2 ����/opt/ros/indigo/include/hardware_interface/joint_state_interface.h 
	hardware_interface::JointStateInterface jnt_state_interface_;//wheel_pos_, wheel_vel_, wheel_eff_, 
	//ref 1 ����diffbot.h  
	//ref 2 ����/opt/ros/indigo/include/hardware_interface/joint_command_interface.h    
	// hardware_interface::PositionJointInterface servo_pos_interface_; 
	hardware_interface::VelocityJointInterface base_vel_interface_;//wheel_cmd_;

	hardware_interface::BaseStateInterface     base_state_interface_;// x_, y_, theta_,x_vel_, y_vel_, theta_vel_;
	hardware_interface::BaseVelocityInterface  base_velocity_interface_;// x_cmd_, y_cmd_, theta_cmd_;

	inline void writeBufferUpdate()
	{
		// higo_ap_.getRobotAbstract()->expect_motor_speed.servo1 = wheel_cmd_[0];
		// higo_ap_.getRobotAbstract()->expect_motor_speed.servo2 = wheel_cmd_[1];
		// higo_ap_.getRobotAbstract()->expect_motor_speed.servo3 = wheel_cmd_[2];
		// higo_ap_.getRobotAbstract()->ask_expect_motor_speed.servo1 = wheel_cmd_[0];//同机器人速度冲突，2选1
		// higo_ap_.getRobotAbstract()->ask_expect_motor_speed.servo2 = wheel_cmd_[1];
		// higo_ap_.getRobotAbstract()->ask_expect_motor_speed.servo3 = wheel_cmd_[2];

		higo_ap_.getRobotAbstract()->expect_robot_speed.x = x_cmd_;
		higo_ap_.getRobotAbstract()->expect_robot_speed.y = y_cmd_;
		higo_ap_.getRobotAbstract()->expect_robot_speed.z = theta_cmd_;



	}

	inline void readBufferUpdate()
	{
		x_ = higo_ap_.getRobotAbstract()->measure_global_coordinate.x;
		y_ = higo_ap_.getRobotAbstract()->measure_global_coordinate.y;
		theta_ = higo_ap_.getRobotAbstract()->measure_global_coordinate.z;

		x_vel_ = higo_ap_.getRobotAbstract()->measure_robot_speed.x;       //
		y_vel_ = higo_ap_.getRobotAbstract()->measure_robot_speed.y;
		theta_vel_ = higo_ap_.getRobotAbstract()->measure_robot_speed.z;

		wheel_pos_[0] = higo_ap_.getRobotAbstract()->measure_motor_mileage.servo1;
		wheel_pos_[1] = higo_ap_.getRobotAbstract()->measure_motor_mileage.servo2;
		wheel_pos_[2] = higo_ap_.getRobotAbstract()->measure_motor_mileage.servo3;

        robot_state.battery_voltage = higo_ap_.getRobotAbstract()->system_info.battery_voltage;
        robot_state.cpu_temperature = higo_ap_.getRobotAbstract()->system_info.cpu_temperature;
        robot_state.cpu_usage = higo_ap_.getRobotAbstract()->system_info.cpu_usage;
        robot_state.system_time = higo_ap_.getRobotAbstract()->system_info.system_time;




		wheel_vel_[0] = higo_ap_.getRobotAbstract()->ask_measure_motor_speed.servo1;
		wheel_vel_[1] = higo_ap_.getRobotAbstract()->ask_measure_motor_speed.servo2;
		wheel_vel_[2] = higo_ap_.getRobotAbstract()->ask_measure_motor_speed.servo3;
		
        // wheel_eff_[0] = 5;
		// wheel_eff_[1] = 6;
		// wheel_eff_[2] = 7;
		


	}

	 void stair_cmd_callback(const stair_car_msgs::stair_configConstPtr& msg);
	 void roll_cmd_callback(const stair_car_msgs::roll_configConstPtr& msg);
	 void brake_cmd_callback(const stair_car_msgs::brake_configConstPtr& msg);
	 void laxian_data_callback(const stair_car_msgs::laxian_dataConstPtr& msg);

	 

};




#endif
