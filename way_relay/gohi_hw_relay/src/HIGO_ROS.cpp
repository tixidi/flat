#include <gohi_hw/HIGO_ROS.h>




 void HIGO_ROS::relay_state_cmd_callback(const gohi_hw_relay_msgs::relay_state& msg)
 {
	// if(msg->brake_config==1)
	{
		// std::cerr<< (uint8_t)msg.relay_state << std::endl;
		switch(msg.relay_state){
			case 0x80:
				 relay_state  |=  (unsigned char)1 << 7;
				//  std::cerr<< "0x80" << std::endl;
				 break;
			case 0x40:
				 relay_state  |= (unsigned char)1 << 6;
				//  std::cerr<< "0x40" << std::endl;
				 break;
			case 0x20:
				relay_state  |= (unsigned char)1 << 5;
				// std::cerr<< "0x20" << std::endl;
				break;
			case 0x60: 
				if(relay_state & 0x80){
		    		relay_state  = relay_state -0x80;
				}
				// std::cerr<< "0x60" << std::endl;
				break;
			case 0xa0:
				if(relay_state & 0x40){
		    		relay_state  = relay_state -0x40;
				}
				// std::cerr<< "0xa0" << std::endl;
				break;
			case 0xc0:
				if(relay_state & 0x20){
		    		relay_state  = relay_state -0x20;
				}
				// std::cerr<< "0xc0" << std::endl;
				break;

		}
	}	
							  
 }

HIGO_ROS::HIGO_ROS(ros::NodeHandle &nh, std::string url, std::string config_addr) :
		higo_ap_(url, config_addr),
		nh_(nh)
	{
		//get the parameter
		nh_.setCallbackQueue(&queue_);
        base_mode_ = "2diff-wheel";
		with_arm_ = false;
		controller_freq_ = 50;  //默认 100
		idcard_write_flag=0;
		nh_.getParam("base_mode", base_mode_);
		nh_.getParam("with_arm", with_arm_);
		nh_.getParam("freq", controller_freq_);

		relay_state_publisher_ = nh_.advertise<gohi_hw_relay_msgs::relay_state>("/relay_state_data", 1000);
		bms_state_publisher_ = nh_.advertise<sensor_msgs::BatteryState>("/bms_state_data", 1000);
		
		relay_cmd_subscribe_ = nh_.subscribe("/relay_state_cmd", 1,  &HIGO_ROS::relay_state_cmd_callback, this);	
		
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

		ros::NodeHandle cm_nh("hw_relay");
		cm_nh.setCallbackQueue(&cm_callback_queue);
		controller_manager::ControllerManager cm(this, cm_nh);

		ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
		ros::AsyncSpinner hw_spinner(1, &queue_);
		ros::Rate rate(controller_freq_);
		cm_spinner.start();
		hw_spinner.start();

		static int command_switch_counts=0;
		int count = 0;
		relay_state =0xe0;  //默认高三位为000   1表示ON   0表示off   每一位对应一个继电器
		ros::Time currentTime = ros::Time::now();
		while (ros::ok())
		{	  

			switch(command_switch_counts%3)
			{
				case 0:
					higo_ap_.updateCommand(SET_RELAY5_STATE, count,0,relay_state);//38
					break;
				case 1:
					higo_ap_.updateCommand(SET_RELAY6_STATE, count,0,relay_state);//38
					break;
				case 2:
					higo_ap_.updateCommand(SET_RELAY7_STATE, count,0,relay_state);//38
					break;
				// case 3:
				// 	higo_ap_.updateCommand(READ_BMS_REG_DATA, count, 2);
				// 	bms_state_.voltage=1;
				// 	bms_state_.current=1;
				// 	bms_state_.percentage=higo_ap_.publish_data;
				// 	bms_state_publisher_.publish(bms_state_);		 
				// 	break;

			}
			command_switch_counts++;
			relay_state_publisher_.publish(relay_state_);			  
				
			//----------------------------------------------------
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

