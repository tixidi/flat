#include <gohi_hw/HIGO_AP.h>




HIGO_AP::HIGO_AP(std::string url, std::string config_addr)
{

    if (url == "tcp")
    {
       // boost::asio::io_service io_service;
	    tcp::endpoint endpoint(address::from_string("192.168.0.204"), 502);

        // 直接从 new 操作符的返回值构造
	    client_ptr new_session(new client(io_service, endpoint));
   

        client_tcp_=new_session;
        read_time_out_ =15;//default 500
        write_time_out_ =10;
        hflinkmodbus_ = boost::make_shared<HFLink_Modbus>(&my_robot_  , 0x01 , 0x11);
        timer_.reset(new boost::asio::deadline_timer(io_service,boost::posix_time::milliseconds(read_time_out_)));
        timer_.reset(new boost::asio::deadline_timer(io_service,boost::posix_time::milliseconds(write_time_out_)));

	    new_session->start();
	    new_session->initializeTcp();
        

    }else if(url == "udp"){
  	 
        // UdpLinkClient usUdpClient("127.0.0.1",NULL,8888);
        
        udp_client_ptr new_session(new UdpLinkClient("127.0.0.1",NULL,8888,io_service));
        
        client_udp_=new_session;


        read_time_out_ =15;//default 500
        write_time_out_ =10;
        hflinkmodbus_ = boost::make_shared<HFLink_Modbus>(&my_robot_  , 0x01 , 0x11);
        timer_.reset(new boost::asio::deadline_timer(io_service,boost::posix_time::milliseconds(read_time_out_)));
        timer_.reset(new boost::asio::deadline_timer(io_service,boost::posix_time::milliseconds(write_time_out_)));


        std::cerr<<"udp_client is dong "<<std::endl;
        int nRet=new_session->Start();


      	if(nRet != 0)
	    {
		    std::cerr<<"udp_client is error "<<std::endl;
	    }

 

        //阻塞等待，否则就直接退出了程序，io_service无法消息循环处理
        //io_service run也可以不调用，但该进程不能直接退出，需要阻塞。
        // ioService_1.run();
        std::cerr<<"udp_client is ok "<<std::endl;
        // new_session->Stop();
        // io_service.stop();

    }



    

    //process the config file
    file_.open(config_addr.c_str(), std::fstream::in);
    if (file_.is_open())
    {
        for (int i = 0; i < LAST_COMMAND_FLAG_; i++)
        {
            std::string temp;
            file_ >> temp >> hflink_command_set_[i] >> hflink_freq_[i];
            std::cout<< temp << hflink_command_set_[i] << hflink_freq_[i]<<std::endl;
        }
        file_.close();
        // initialize_ok_ = port_->initialize_ok();
    } else
    {
        std::cerr << "config file can't be opened, check your system" <<std::endl;
        // initialize_ok_ = false;
    }
}

void HIGO_AP::timeoutHandler(const boost::system::error_code &ec)
{
    if (!ec)
    {
        std::cerr << "Time Out" <<std::endl;
        boost::mutex::scoped_lock lock(wait_mutex_);
        time_out_flag_ = true;
    }
}






bool HIGO_AP::updateCommand(const MotorModbusCommand &command, int count,int read_or_write)
{
         static int time_out_count=0;

         
         boost::asio::deadline_timer cicle_timer_(io_service);
         cicle_timer_.expires_from_now(boost::posix_time::millisec(read_time_out_));
         
         ros::Time currentTime = ros::Time::now();
         time_out_count++ ;

            
    if(read_or_write==2)
    {
        Buffer data = client_tcp_->readBuffer();
        
        ack_ready_ = false;
        while (!ack_ready_)
        {          
            for (int i = 0; i < data.size(); i++)
            {    
                if (hflinkmodbus_->byteAnalysisCall_R_FromPAD(data[i]))
                {
                    // one package ack arrived  
                    ack_ready_ = true;          
                }

            }
            data = client_tcp_->readBuffer();
            if (cicle_timer_.expires_from_now().is_negative())
            {
                // std::cerr<<"Timeout continue skip this package"<<std::endl;
                return false;
            }
        }
    }
    else if(read_or_write==3)
    {
        // update command set  data from embedded system
        if (hflink_command_set_[command] != 0)
        {
                sendCommandModbus(command);
        }


    }


    return true;
}

