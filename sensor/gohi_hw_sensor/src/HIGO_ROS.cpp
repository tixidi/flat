#include <gohi_hw/HIGO_ROS.h>



 void HIGO_ROS::idcard_write_config_callback(const gohi_hw_sensor_msgs::idcard_write_config& msg)
 {
	higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data1=msg.data[0];
	higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data2=msg.data[1];
	higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data3=msg.data[2];
	higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data4=msg.data[3];
	higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data5=msg.data[4];
	higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data6=msg.data[5];    
	higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data7=msg.data[6];
	higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data8=msg.data[7];  
	idcard_write_flag=1; 
	  
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
		idcard_write_flag=0;
		nh_.getParam("base_mode", base_mode_);
		nh_.getParam("with_arm", with_arm_);
		nh_.getParam("freq", controller_freq_);

		idcard_read_config_publisher_ = nh_.advertise<gohi_hw_sensor_msgs::idcard_read_config>("/idcard_read_config/cmd", 1000);
		idcard_write_config_subscriber_ = nh_.subscribe("/idcard_write_config/cmd", 1,  &HIGO_ROS::idcard_write_config_callback, this);
		laxian_data_publisher_ =nh_.advertise<gohi_hw_sensor_msgs::laxian_data>("/stair_car_mobile_base/laxian_data", 1000);



		
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

		ros::NodeHandle cm_nh("hw_sensor");
		cm_nh.setCallbackQueue(&cm_callback_queue);
		controller_manager::ControllerManager cm(this, cm_nh);

		ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
		ros::AsyncSpinner hw_spinner(1, &queue_);
		ros::Rate rate(controller_freq_);
		cm_spinner.start();
		hw_spinner.start();

		int count = 0;
		ros::Time currentTime = ros::Time::now();
		static int command_switch_counts=0;
		int i =1;
		while (ros::ok())
		{	  


			std::cerr <<"-------------------------------------------------------------------------------" <<std::endl;

			switch(command_switch_counts%3)
			{					   
				case 0:
						higo_ap_.updateCommand(READ_LAXIAN_POSITION, count,0);//拉线传感器  ---ok 
						laxian_data_.length =(float)higo_ap_.getRobotAbstract()->laxian_length.length_data;
						laxian_data_publisher_.publish(laxian_data_);
						break;
				case 1:		
						 higo_ap_.updateCommand(READ_RFID_REG_DATA, count,0);//射频传感器读卡  ---ok	
				    	break;
				case 2:		
						 higo_ap_.updateCommand(READ_THERMOMETER_REG_DATA, count,0);//射频传感器读卡  ---ok	
				    	break;
						
			}
			command_switch_counts++;


             if(idcard_write_flag==1)
			 {
				std::cerr << "idcard is  " << std::endl;
                idcard_write_flag=0;
											
				std::cerr <<"ID1 interface   " <<(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data1<<std::endl; 
				std::cerr <<"ID2 interface   " <<(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data2<<std::endl; 
				std::cerr <<"ID3 interface   " <<(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data3<<std::endl; 
				std::cerr <<"ID4 interface   " <<(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data4<<std::endl; 
				std::cerr <<"ID5 interface   " <<(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data5<<std::endl; 
				std::cerr <<"ID6 interface   " <<(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data6<<std::endl; 
				std::cerr <<"ID7 interface   " <<(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data7<<std::endl; 
				std::cerr <<"ID8 interface   " <<(short int)higo_ap_.getRobotAbstract()->rfid_write_data.write_to_reg_data8<<std::endl; 

				higo_ap_.updateCommand(SET_RFID_REG_DATA, count,0);
				unsigned char write_flag=higo_ap_.getRobotAbstract()->rfid_read_data.write_state;
				while(write_flag!=0);
			 }
		
			idcard_read_config_.data[0]=(short int)higo_ap_.getRobotAbstract()->Temperature_Data.read_from_reg_data3;
			idcard_read_config_.data[1]=(short int)higo_ap_.getRobotAbstract()->Temperature_Data.read_from_reg_data4;
	
			idcard_read_config_.data[2]=(short int)higo_ap_.getRobotAbstract()->laxian_length.length_data;
			
			idcard_read_config_.data[3]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.write_state;
			idcard_read_config_.data[4]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.read_state;
			idcard_read_config_.data[5]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.singnal_intensity;				
			idcard_read_config_.data[6]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data1;
			idcard_read_config_.data[7]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data2;
			idcard_read_config_.data[8]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data3;
			idcard_read_config_.data[9]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data4;
			idcard_read_config_.data[10]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data5;
			idcard_read_config_.data[11]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data6;
			idcard_read_config_.data[12]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data7;
			idcard_read_config_.data[13]=(short int)higo_ap_.getRobotAbstract()->rfid_read_data.read_from_reg_data8;
			// sleep(10);	
			// idcard_read_config_.data[0] =i++;
			// if(i>37)i =1;
		    idcard_read_config_publisher_.publish(idcard_read_config_);		


		
			readBufferUpdate();

			cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));


			writeBufferUpdate();

			
			std::cerr <<(int)count<< "spend time is  " << (float)(ros::Time::now() - currentTime).toSec() <<std::endl;
			currentTime = ros::Time::now();

			rate.sleep();

			count++;


//--------------------------------------------------------------------------------------------------

		}

		cm_spinner.stop();
		hw_spinner.stop();

	}

