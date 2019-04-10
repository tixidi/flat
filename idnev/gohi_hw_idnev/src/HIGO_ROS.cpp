#include <gohi_hw/HIGO_ROS.h>




void HIGO_ROS::idcard_read_config_callback(const gohi_hw_idnev_msgs::idcard_read_config& msg)
 {
	std::cerr << "idcard is  " << std::endl;
	higo_ap_.getRobotAbstract()->id_info_data.id_number=msg.data[13];
	std::cerr <<"ID number =" <<msg.data[13]<<std::endl; 								  
 }



void HIGO_ROS::start_idcard_read_config_callback(const gohi_hw_idnev_msgs::start_idnev& msg)
 {
	//  std::cerr << "set brake  is ------------------------------- " << std::endl;
	start_idnev_cmd_flag_ =msg.start_idnev_cmd;					  
 }


HIGO_ROS::HIGO_ROS(ros::NodeHandle &nh, std::string url, std::string config_addr,std::string idConfig_addr) :
		higo_ap_(url, config_addr,idConfig_addr),
		nh_(nh)
	{
		//get the parameter
		nh_.setCallbackQueue(&queue_);
        base_mode_ = "2diff-wheel";
		with_arm_ = false;
		controller_freq_ = 100;
		idcard_write_flag=0;
		nh_.getParam("base_mode", base_mode_);
		nh_.getParam("with_arm", with_arm_);
		nh_.getParam("freq", controller_freq_);

		idnev_state_publisher_  = nh_.advertise<gohi_hw_idnev_msgs::idnev_state>("/idnev_state_data", 1000);
		idnev_state_subscriber_ = nh_.subscribe("/idcard_read_config/cmd", 1,  &HIGO_ROS::idcard_read_config_callback, this);
        start_idnev_state_subscriber_ = nh_.subscribe("/start_idnev/cmd", 1,  &HIGO_ROS::start_idcard_read_config_callback, this);

           
   		flat_cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("/flat_car_mobile_base/flat_car_mobile_base_controller/cmd_vel", 1000);//flat-car-cmd_vel
	    stair_cmd_publisher_ = nh_.advertise<gohi_hw_idnev_msgs::stair_config>("/stair_car_mobile_base/stair_controller/cmd_vel", 1000);//stair-car-position-cmd_vel
		roll_cmd_publisher_ = nh_.advertise<gohi_hw_idnev_msgs::roll_config>("/stair_car_mobile_base/roll_controller/cmd_vel", 1000);//stair-car-speed-cmd_vel
		power_cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("/power_car_mobile_base/power_car_mobile_base_controller/cmd_vel", 1000);//flat-car-cmd_vel
		
		idnev_state_.x_speed =0;
		idnev_state_.id_number =0;
		idnev_state_.Rz =0;
		idnev_state_.stair_position =0;
		idnev_state_.stair_speed =0;
		idnev_state_.stair_type =0;
		idnev_state_.roll_speed  =0;
		start_idnev_cmd_flag_ =0;
		// higo_ap_.getRobotAbstract()->set_car_speed_flag =0;

 		if (higo_ap_.initialize_ok())
		{
			ROS_INFO("system initialized succeed, ready for communication");
		}
		else
		{
			ROS_ERROR("hf link initialized failed, please check the hardware");
		}

	}


	void HIGO_ROS::mainloop()
	{
		ros::CallbackQueue cm_callback_queue;
	    geometry_msgs::Twist flat_twist;
		geometry_msgs::Twist power_twist;
		ros::NodeHandle cm_nh("hw_idnev");
		cm_nh.setCallbackQueue(&cm_callback_queue);
		controller_manager::ControllerManager cm(this, cm_nh);

		ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
		ros::AsyncSpinner hw_spinner(1, &queue_);
		ros::Rate rate(controller_freq_);
		cm_spinner.start();
		hw_spinner.start();

		int count = 0;
		static int time_count =0;
		ros::Time currentTime = ros::Time::now();
		while (ros::ok())
		{	  

			if(start_idnev_cmd_flag_ ==1)
			{
				//  higo_ap_.updateCommand(READ_EULER_ANGLE, count,0);//38
				if(higo_ap_.dataAnalysis(higo_ap_.getRobotAbstract()->id_info_data))
				{
					if(higo_ap_.set_car_speed_flag)
					{
						higo_ap_.read_id_num_temp =higo_ap_.getRobotAbstract()->id_info_data.id_number;
						idnev_state_.x_speed =higo_ap_.getRobotAbstract()->id_info_data.x_speed;
						idnev_state_.Rz =higo_ap_.getRobotAbstract()->id_info_data.Rz;
						idnev_state_.id_number =higo_ap_.getRobotAbstract()->id_info_data.id_number;				
						idnev_state_.stair_position =higo_ap_.getRobotAbstract()->id_info_data.stair_position;
						idnev_state_.stair_speed = higo_ap_.getRobotAbstract()->id_info_data.stair_speed;
						idnev_state_.stair_type = higo_ap_.getRobotAbstract()->id_info_data.stair_type;
						idnev_state_.roll_speed = higo_ap_.getRobotAbstract()->id_info_data.roll_speed;
						higo_ap_.set_car_speed_flag =0;
						stair_config.speed =(float)idnev_state_.stair_speed;
						stair_config.type =(float)idnev_state_.stair_type;
						stair_config.position =(float)idnev_state_.stair_position;
						stair_config.start_idnev =0;
		                roll_config_.m1_speed =idnev_state_.roll_speed;
						roll_config_.start_idnev =1;
						roll_cmd_publisher_.publish(roll_config_);
						time_count =0;			
					}
				}
				else
				{
					//do nothing 
				}
				if((time_count%5 ==0)&&time_count<20){				
					stair_cmd_publisher_.publish(stair_config);
					stair_cmd_publisher_.publish(stair_config);
					stair_cmd_publisher_.publish(stair_config);
					stair_cmd_publisher_.publish(stair_config);
					
				}
				time_count++;
				
				//flat-car speed control
				flat_twist.linear.x = (float)idnev_state_.x_speed;
				flat_twist.linear.y = 0;
				flat_twist.linear.z = 0;
				flat_twist.angular.x = 0;
				flat_twist.angular.y = 0;
				flat_twist.angular.z = (float)idnev_state_.Rz;				
				 flat_cmd_publisher_.publish(flat_twist);	

				//power-car speed control
				power_twist.linear.x = (float)idnev_state_.x_speed;
				power_twist.linear.y = 0;
				power_twist.linear.z = 0;
				power_twist.angular.x = 0;
				power_twist.angular.y = 0;
				power_twist.angular.z = (float)idnev_state_.Rz;				
				 power_cmd_publisher_.publish(power_twist);	

					
				// std::cerr <<"measure pitch  " <<robot->euler_angle.pitch*180.0/32768  <<std::endl;  
				// std::cerr <<"measure roll  " <<robot->euler_angle.roll *180.0/32768<<std::endl;   
				// std::cerr <<"measure yaw  " <<robot->euler_angle.yaw*180.0/32768<<std::endl;   

				// imu_state_.euler_x=higo_ap_.getRobotAbstract()->euler_angle.pitch;
				// imu_state_.euler_y=higo_ap_.getRobotAbstract()->euler_angle.roll;				
				// imu_state_.euler_z=higo_ap_.getRobotAbstract()->euler_angle.yaw;					
				//----------------------------------------------------
				std::cerr<<"22222222222222222222222222222222222222222222"<<std::endl;
			}
			else
			{
				stair_config.speed =0;
				stair_config.type =0;
				stair_config.position =0;
				stair_config.start_idnev =1;
				stair_cmd_publisher_.publish(stair_config);
				roll_config_.m1_speed =idnev_state_.roll_speed;
				roll_config_.start_idnev =0;
				roll_cmd_publisher_.publish(roll_config_);
				std::cerr<<"1111111111111111111111111111111111111111111"<<std::endl;
			}
			
			readBufferUpdate();

			cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));

			writeBufferUpdate();     
			 
			rate.sleep();
			

			count++;


			
			currentTime = ros::Time::now();
		}

		cm_spinner.stop();
		hw_spinner.stop();

	}

