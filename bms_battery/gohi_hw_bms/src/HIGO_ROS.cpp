#include <gohi_hw/HIGO_ROS.h>


HIGO_ROS::HIGO_ROS(ros::NodeHandle &nh, std::string url, std::string config_addr) :
		higo_ap_(url, config_addr),
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

		bms_state_publisher_ = nh_.advertise<sensor_msgs::BatteryState>("/bms_state_data", 1000);
		




		
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

		ros::NodeHandle cm_nh("hw_bms");
		cm_nh.setCallbackQueue(&cm_callback_queue);
		controller_manager::ControllerManager cm(this, cm_nh);

		ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
		ros::AsyncSpinner hw_spinner(1, &queue_);
		ros::Rate rate(controller_freq_);
		cm_spinner.start();
		hw_spinner.start();

		int count = 0;
		ros::Time currentTime = ros::Time::now();
		while (ros::ok())
		{	  

	
				// higo_ap_.updateCommand(READ_LAXIAN_POSITION, count,0);//拉线传感器  ---ok  
				// // higo_ap_.updateCommand(READ_EULER_ANGLE, count,0);//38
				// higo_ap_.updateCommand(READ_RFID_REG_DATA, count,0);//射频传感器读卡  ---ok
				// higo_ap_.updateCommand(READ_THERMOMETER_REG_DATA, count,0);	//温度传感器	

			higo_ap_.updateCommand(READ_BMS_REG_DATA, count, 2);
			//  std::cerr << "the publish_data is " <<higo_ap_.publish_data<<std::endl;
	  


			bms_state_.power_supply_health =higo_ap_.bms_battery_.battery_health;
			bms_state_.voltage=higo_ap_.bms_battery_.total_voltage;
			bms_state_.current=higo_ap_.bms_battery_.current_capacity;
			bms_state_.percentage=higo_ap_.bms_battery_.battery_capacity_percentage;
 			bms_state_publisher_.publish(bms_state_);		 
			
				
			//----------------------------------------------------
			readBufferUpdate();

			cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));
            // ROS_INFO("head_servo1_cmd_ = %.4f  head_servo2_cmd_=%.4f" , head_servo1_cmd_ ,head_servo2_cmd_);

			writeBufferUpdate();     
			 
			rate.sleep();
			
			// std::cerr << "spend time is  " << (ros::Time::now() - currentTime).toSec() << std::endl;
			// currentTime = ros::Time::now();
			count++;

		//	std::cerr << "spend time is  " << (ros::Time::now() - currentTime).toSec() <<std::endl;
		//	std::cerr << "count is  " <<  count<<std::endl;
			
			currentTime = ros::Time::now();
		}

		cm_spinner.stop();
		hw_spinner.stop();

	}

