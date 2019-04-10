#include <gohi_hw/HIGO_ROS.h>


void HIGO_ROS::idcard_read_config_callback(const gohi_hw_monitor_msgs::idcard_read_config& msg)
 {
	std::cerr << "idcard is  " << std::endl;

	higo_ap_.getRobotAbstract()->Temperature_Data.read_from_reg_data3=msg.data[0];
	higo_ap_.getRobotAbstract()->Temperature_Data.read_from_reg_data4 =msg.data[1];
	

	higo_ap_.getRobotAbstract()->laxian_length.length_data=msg.data[2];

	higo_ap_.getRobotAbstract()->rfid_read_data.singnal_intensity=msg.data[3];
	higo_ap_.getRobotAbstract()->rfid_read_data.read_state=msg.data[4];
	higo_ap_.getRobotAbstract()->rfid_read_data.write_state=msg.data[5];

	higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data1=msg.data[6];
	higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data2=msg.data[7];
	higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data3=msg.data[8];
	higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data4=msg.data[9];
	higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data5=msg.data[10];
	higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data6=msg.data[11];    
	higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data7=msg.data[12];
	higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data8=msg.data[13];  


	// higo_ap_.getRobotAbstract()->euler_angle.pitch=msg.data[14];
	// higo_ap_.getRobotAbstract()->euler_angle.roll =msg.data[15];
	// higo_ap_.getRobotAbstract()->euler_angle.yaw=msg.data[16];


	std::cerr <<"measure temp_for: " <<msg.data[0]<<"  'C" <<std::endl; 
	std::cerr <<"measure temp_back  " <<msg.data[1]  <<"  'C"<<std::endl;  
	std::cerr <<"measure Length: " <<msg.data[2]<<"  mm" <<std::endl; 


	std::cerr <<"measure singnal " <<msg.data[5]<<std::endl;   
	std::cerr <<"measure read_state  " <<msg.data[4]<<std::endl;   	
	std::cerr <<"measure write_state  " <<msg.data[3]<<std::endl; 

	std::cerr <<"ID1 interface   " <<msg.data[6]<<std::endl; 				  
	std::cerr <<"ID2 interface   " <<msg.data[7]<<std::endl; 
	std::cerr <<"ID3 interface   " <<msg.data[8]<<std::endl; 
	std::cerr <<"ID4 interface   " <<msg.data[9]<<std::endl; 
	std::cerr <<"ID5 interface   " <<msg.data[10]<<std::endl; 
	std::cerr <<"ID6 interface   " <<msg.data[11]<<std::endl; 
	std::cerr <<"ID7 interface   " <<msg.data[12]<<std::endl; 								
	std::cerr <<"ID8 interface   " <<msg.data[13]<<std::endl; 		




  	idcard_read_flag=1;
								  
 }

 void HIGO_ROS::laser_range_read_config_callback(const gohi_hw_monitor_msgs::sick_range& msg)
 {
	std::cerr << "sick range is  " << std::endl;

    // memcpy(&msg.data[0],&higo_ap_.getRobotAbstract()->laser_scan_data[0],360);   
  	
	 for(int kk=0;kk<241;kk++) 
	  {
		  higo_ap_.getRobotAbstract()->laser_scan_data[kk]=msg.data[kk];
	  }
	  sick_range_read_flag=1;

								  
 }

 void HIGO_ROS::imu_state_read_callback(const gohi_hw_monitor_msgs::imu_state& msg)
 {
	std::cerr << "imu read is  " << std::endl;

  	
	higo_ap_.getRobotAbstract()->euler_angle.pitch=msg.euler_x;
	higo_ap_.getRobotAbstract()->euler_angle.roll =msg.euler_y;
	higo_ap_.getRobotAbstract()->euler_angle.yaw=msg.euler_z;
    
	std::cerr <<"measure pitch  " <<msg.euler_x  <<std::endl;  
	std::cerr <<"measure roll  " <<msg.euler_y<<std::endl;   
	std::cerr <<"measure yaw  " <<msg.euler_z<<std::endl;   

	imu_read_flag=1;
								  
 }

 void HIGO_ROS::battery_state_read_callback(const sensor_msgs::BatteryState& msg)
 {	
	higo_ap_.getRobotAbstract()->bms_battey_.current_capacity=(short int)msg.current;
	higo_ap_.getRobotAbstract()->bms_battey_.total_voltage =msg.voltage;
	higo_ap_.getRobotAbstract()->bms_battey_.battery_health=msg.power_supply_health;
	higo_ap_.getRobotAbstract()->bms_battey_.battery_capacity_percentage =msg.percentage;
								  
 }


 void HIGO_ROS::stair_state_read_callback(const gohi_hw_monitor_msgs::robot_state& msg)
 {	
	higo_ap_.getRobotAbstract()->moter_speed_state_.mot3_speed=msg.motor3_speed;
	higo_ap_.getRobotAbstract()->moter_speed_state_.mot4_speed =msg.motor4_speed;
	higo_ap_.getRobotAbstract()->moter_error_state_.mot3_error=msg.motor3_error_state;
	higo_ap_.getRobotAbstract()->moter_error_state_.mot4_error =msg.motor4_error_state;
								  
 }
  void HIGO_ROS::power_state_read_callback(const gohi_hw_monitor_msgs::robot_state& msg)
 {	
	 
	higo_ap_.getRobotAbstract()->moter_speed_state_.mot5_speed=msg.motor5_speed;
	higo_ap_.getRobotAbstract()->moter_speed_state_.mot6_speed =msg.motor6_speed;
	higo_ap_.getRobotAbstract()->moter_error_state_.mot5_error =msg.motor5_error_state;
	higo_ap_.getRobotAbstract()->moter_error_state_.mot6_error =msg.motor6_error_state;

								  
 }
  void HIGO_ROS::flat_state_read_callback(const gohi_hw_monitor_msgs::robot_state& msg)
 {	
	 
	higo_ap_.getRobotAbstract()->moter_speed_state_.mot1_speed=msg.motor1_speed;
	higo_ap_.getRobotAbstract()->moter_speed_state_.mot2_speed =msg.motor2_speed;
	higo_ap_.getRobotAbstract()->moter_error_state_.mot1_error=msg.motor1_error_state;
	higo_ap_.getRobotAbstract()->moter_error_state_.mot2_error=msg.motor2_error_state;

								  
 }

HIGO_ROS::HIGO_ROS(ros::NodeHandle &nh, std::string url, std::string config_addr) :
		higo_ap_(url, config_addr),
		nh_(nh)
	{
		//get the parameter
		nh_.setCallbackQueue(&queue_);
        base_mode_ = "2diff-wheel";
		with_arm_ = false;
		controller_freq_ = 10;
		nh_.getParam("base_mode", base_mode_);
		nh_.getParam("with_arm", with_arm_);
		nh_.getParam("freq", controller_freq_);
//      public more topics
		power_cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("/power_car_mobile_base/power_car_mobile_base_controller/cmd_vel", 1000);
		flat_cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("/flat_car_mobile_base/flat_car_mobile_base_controller/cmd_vel", 1000);
		stair_cmd_publisher_ = nh_.advertise<gohi_hw_monitor_msgs::stair_config>("/stair_car_mobile_base/stair_controller/cmd_vel", 1000);
		roll_cmd_publisher_ = nh_.advertise<gohi_hw_monitor_msgs::roll_config>("/stair_car_mobile_base/roll_controller/cmd_vel", 1000);
		idcard_write_config_publisher_ = nh_.advertise<gohi_hw_monitor_msgs::idcard_write_config>("/idcard_write_config/cmd", 1000);
		laser_range_config_publisher_ = nh_.advertise<gohi_hw_monitor_msgs::laser_range_config>("/laser_range_config/cmd", 1000);
		robot_desire_point_config_publisher_ = nh_.advertise<gohi_hw_monitor_msgs::robot_desire_point_config>("/robot_desire_point_config/cmd", 1000);

		idcard_read_config_subscriber_ = nh_.subscribe("/idcard_read_config/cmd", 1,  &HIGO_ROS::idcard_read_config_callback, this);
		sick_range_read_config_subscriber_ = nh_.subscribe("/sick_range", 1,  &HIGO_ROS::laser_range_read_config_callback, this);
		imu_state_subscriber_ = nh_.subscribe("/imu_state_data", 1,  &HIGO_ROS::imu_state_read_callback, this);
		battery_state_subscriber_ = nh_.subscribe("/bms_state_data", 1,  &HIGO_ROS::battery_state_read_callback, this);
		stair_car_state_subscriber_ =nh_.subscribe("/stair_car_robot_state", 1,  &HIGO_ROS::stair_state_read_callback, this);
		power_car_state_subscriber_ =nh_.subscribe("/power_car_robot_state", 1,  &HIGO_ROS::power_state_read_callback, this);
		flat_car_state_subscriber_ =nh_.subscribe("/flat_car_robot_state", 1,  &HIGO_ROS::flat_state_read_callback, this);

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
		ros::NodeHandle cm_nh("mobile_base1");
		cm_nh.setCallbackQueue(&cm_callback_queue);
		controller_manager::ControllerManager cm(this, cm_nh);

		ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
		ros::AsyncSpinner hw_spinner(1, &queue_);
		ros::Rate rate(controller_freq_);
		cm_spinner.start();
		hw_spinner.start();

		int count = 0;
		ros::Time currentTime = ros::Time::now();

        static int send_times=0;

		while (ros::ok())
		{

			

        //    // car1 /car2  speed control    
        //   if(higo_ap_.getRobotAbstract()->receive_package_flag==1)
		//   {
		// 	  	higo_ap_.getRobotAbstract()->receive_package_flag=0;
		// 		// higo_ap_.updateCommand(READ_INTERFACE_CAR1_SPEED_CONTROL, count,3);//回传应答
		// 		flat_twist.linear.x = (float)higo_ap_.getRobotAbstract()->car1_speed_config.x_speed/100.0 ;
		// 		flat_twist.linear.y = 0;
		// 		flat_twist.linear.z = 0;
		// 		flat_twist.angular.x = 0;
		// 		flat_twist.angular.y = 0;
		// 		flat_twist.angular.z = (float)higo_ap_.getRobotAbstract()->car1_speed_config.z_speed/100.0;
        // 		std::cerr <<"car1 interface x_speed  " <<flat_twist.linear.x <<std::endl; 
        // 		std::cerr <<"car1 interface y_speed  " <<flat_twist.linear.y <<std::endl; 
        // 		std::cerr <<"car1 interface z_speed  " <<flat_twist.angular.z<<std::endl; 				
		// 		flat_cmd_publisher_.publish(flat_twist);
		// 		power_twist.linear.x = (float)higo_ap_.getRobotAbstract()->car1_speed_config.x_speed/100.0 ;
		// 		power_twist.linear.y = 0;
		// 		power_twist.linear.z = 0;
		// 		power_twist.angular.x = 0;
		// 		power_twist.angular.y = 0;
		// 		power_twist.angular.z = (float)higo_ap_.getRobotAbstract()->car1_speed_config.z_speed/100.0;
		// 		power_cmd_publisher_.publish(power_twist);
		//   }
		//   //car3 position control
		//   else  if(higo_ap_.getRobotAbstract()->receive_package_flag==3)
		//   {
		// 		higo_ap_.getRobotAbstract()->receive_package_flag=0;
		// 		// higo_ap_.updateCommand(READ_INTERFACE_CAR3_POSITION_CONTROL, count,3);//回传应答				
		// 		stair_vel_cmd_config_.speed = (float)higo_ap_.getRobotAbstract()->car3_position_config.speed/100.0 ;
		// 		stair_vel_cmd_config_.type = (float)higo_ap_.getRobotAbstract()->car3_position_config.type ;
		// 		stair_vel_cmd_config_.position = (float)higo_ap_.getRobotAbstract()->car3_position_config.position ;
		// 		std::cerr <<"car3 interface speed  " <<stair_vel_cmd_config_.speed<<std::endl; 
        // 		std::cerr <<"car3 interface type  " <<stair_vel_cmd_config_.type<<std::endl; 
        // 		std::cerr <<"car3 interface position  " <<stair_vel_cmd_config_.position<<std::endl; 
 
		// 		stair_cmd_publisher_.publish(stair_vel_cmd_config_);			  
		//   }
		//   // car4 -dilong speed_control
		//   else  if(higo_ap_.getRobotAbstract()->receive_package_flag==4)
		//   {
		// 	  	higo_ap_.getRobotAbstract()->receive_package_flag=0;
		// 		// higo_ap_.updateCommand(READ_INTERFACE_CAR4_SINGLE_SPEED_CONTROL, count,3);//回传应答					  
		//    		roll_vel_cmd_config_.m1_speed = (float)higo_ap_.getRobotAbstract()->car4_single_speed_config.m1_speed/100.0 ;
		//    		roll_vel_cmd_config_.m2_speed = (float)higo_ap_.getRobotAbstract()->car4_single_speed_config.m1_speed/100.0 ;
		//    		roll_vel_cmd_config_.m3_speed = (float)higo_ap_.getRobotAbstract()->car4_single_speed_config.m1_speed/100.0 ;
		//    		std::cerr <<"car4 interface m1_speed  " <<roll_vel_cmd_config_.m1_speed<<std::endl; 
        // 		std::cerr <<"car4 interface m2_speed  " <<roll_vel_cmd_config_.m2_speed<<std::endl; 
        // 		std::cerr <<"car4 interface m3_speed  " <<roll_vel_cmd_config_.m3_speed<<std::endl; 

    	//    		roll_cmd_publisher_.publish(roll_vel_cmd_config_);			  
		//   }
		//   //car 1 -position feedback
		//   else  if(higo_ap_.getRobotAbstract()->receive_package_flag==5)
		//   {
		// 	  	higo_ap_.getRobotAbstract()->receive_package_flag=0;
		// 		// higo_ap_.updateCommand(READ_INTERFACE_CAR1_ODOM_CONTROL, count,3);//回传应答	
		//    		robot_desire_point_config_.position_x = (float)higo_ap_.getRobotAbstract()->car_global_position_config.position_X/100.0;
		//    		robot_desire_point_config_.position_y= (float)higo_ap_.getRobotAbstract()->car_global_position_config.position_Y/100.0 ;
		//    		robot_desire_point_config_.position_z = (float)higo_ap_.getRobotAbstract()->car_global_position_config.position_Z/100.0 ;

		// 		std::cerr <<"car1 X Position  " <<robot_desire_point_config_.position_x<<std::endl; 
		// 		std::cerr <<"car1 Y Position  " <<robot_desire_point_config_.position_y<<std::endl; 
		// 		std::cerr <<"car1 Z Position  " <<robot_desire_point_config_.position_z<<std::endl;  


    	//    		robot_desire_point_config_publisher_.publish(robot_desire_point_config_);			  
		//   }		  
		//   // id  write control
		//   else  if(higo_ap_.getRobotAbstract()->receive_package_flag==6)
		//   {
		// 	  	higo_ap_.getRobotAbstract()->receive_package_flag=0;
		// 		// higo_ap_.updateCommand(READ_INTERFACE_ID_CONTROL, count,3);//回传应答		
				
		// 		idcard_write_config_.data[0]=(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data1;
		// 		idcard_write_config_.data[1]=(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data2;
		// 		idcard_write_config_.data[2]=(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data7;
		// 		idcard_write_config_.data[3]=(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data6;
		// 		idcard_write_config_.data[4]=(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data4;
		// 		idcard_write_config_.data[5]=(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data5;
		// 		idcard_write_config_.data[6]=(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data3;
		// 		idcard_write_config_.data[7]=(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data8;

		// 		std::cerr <<"ID1 interface   " <<idcard_write_config_.data[0]<<std::endl; 
		// 		std::cerr <<"ID2 interface   " <<idcard_write_config_.data[1]<<std::endl; 
		// 		std::cerr <<"ID3 interface   " <<idcard_write_config_.data[2]<<std::endl; 
		// 		std::cerr <<"ID4 interface   " <<idcard_write_config_.data[3]<<std::endl; 
		// 		std::cerr <<"ID5 interface   " <<idcard_write_config_.data[4]<<std::endl; 
		// 		std::cerr <<"ID6 interface   " <<idcard_write_config_.data[5]<<std::endl; 
		// 		std::cerr <<"ID7 interface   " <<idcard_write_config_.data[6]<<std::endl; 
		// 		std::cerr <<"ID8 interface   " <<idcard_write_config_.data[7]<<std::endl; 
				
    	//    		idcard_write_config_publisher_.publish(idcard_write_config_);			  
		//   }
		//   else  if(higo_ap_.getRobotAbstract()->receive_package_flag==7)
		//   {
		// 	  	higo_ap_.getRobotAbstract()->receive_package_flag=0;
	    //  		// higo_ap_.updateCommand(READ_INTERFACE_LASER_CONTROL, count,3);//回传应答					  

		// 		laser_range_config_.l_range=(short int)higo_ap_.getRobotAbstract()->laser_range_config.laser_range_L;
		// 		laser_range_config_.r_range=(short int)higo_ap_.getRobotAbstract()->laser_range_config.laser_range_R;

		// 		std::cerr <<"Laser range L  " <<laser_range_config_.l_range<<std::endl; 
		// 		std::cerr <<"Laser range R  " <<laser_range_config_.r_range<<std::endl; 

        //         float temp_range=laser_range_config_.r_range-laser_range_config_.l_range;
        // 		if(temp_range>0)  
        // 		{
        //     		laser_range_config_.laser_scan_num=(unsigned short int)temp_range/0.5;
		// 		}
		// 		else    laser_range_config_.laser_scan_num=0;    
             
        //     	std::cerr <<"Laser scan num  " <<laser_range_config_.laser_scan_num<<std::endl; 
    	//    		laser_range_config_publisher_.publish(laser_range_config_);			  
		//   }
		//   else  if(higo_ap_.getRobotAbstract()->receive_package_flag==8)
		//   {
		// 	  	higo_ap_.getRobotAbstract()->receive_package_flag=0;
        // 	    higo_ap_.updateCommand(WRITE_SENSOR_DATA_TO_PAD_INTERFACE, count,3);//射频传感器读卡  ---ok				  	  
		//   }
		//   else  if(higo_ap_.getRobotAbstract()->receive_package_flag==9)
		//   {
		// 	  	higo_ap_.getRobotAbstract()->receive_package_flag=0;
        // 	    higo_ap_.updateCommand(WRITE_LASER_DATA_TO_PAD_INTERFACE, count,3);//射频传感器读卡  ---ok				  	  
		//   }

        //     if(send_times%10==0)
		// 	 	 higo_ap_.updateCommand(WRITE_LASER_DATA_TO_PAD_INTERFACE_1, count,3);//射频传感器读卡  ---ok
				  
			send_times++;
			higo_ap_.updateCommand(READ_INTERFACE_CAR1_SPEED_CONTROL, count,2);//射频传感器读卡
			//----------------------------------------------------

// 
            // higo_ap_.write_data();
			
			readBufferUpdate();

			cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));


			writeBufferUpdate();
			 
			rate.sleep();
			
			count++;
		   

		}

		cm_spinner.stop();
		hw_spinner.stop();

	}

