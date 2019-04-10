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

		imu_state_publisher_ = nh_.advertise<gohi_hw_imu_msgs::imu_state>("/imu_state_data", 1000);

		
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

		ros::NodeHandle cm_nh("hw_imu");
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

	
			higo_ap_.updateCommand(READ_EULER_ANGLE, count,0);//38


           std::cerr <<"measure pitch  " <<higo_ap_.getRobotAbstract()->euler_angle.pitch*180.0/32768<<std::endl;  
   		   std::cerr <<"measure roll  " <<higo_ap_.getRobotAbstract()->euler_angle.roll *180.0/32768<<std::endl;   
           std::cerr <<"measure yaw  " <<higo_ap_.getRobotAbstract()->euler_angle.yaw*180.0/32768<<std::endl;   

		    imu_state_.euler_x=higo_ap_.getRobotAbstract()->euler_angle.pitch;
			imu_state_.euler_y=higo_ap_.getRobotAbstract()->euler_angle.roll;				
			imu_state_.euler_z=higo_ap_.getRobotAbstract()->euler_angle.yaw;	

			imu_state_publisher_.publish(imu_state_);			  
				
			//----------------------------------------------------
			readBufferUpdate();

			cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));

			writeBufferUpdate();     
			 
			rate.sleep();
			

			count++;


			
			std::cerr <<(int)count<< "spend time is  " << (float)(ros::Time::now() - currentTime).toSec() <<std::endl;
			currentTime = ros::Time::now();
		}

		cm_spinner.stop();
		hw_spinner.stop();

	}

