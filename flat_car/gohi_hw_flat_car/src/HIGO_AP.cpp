#include <gohi_hw/HIGO_AP.h>




HIGO_AP::HIGO_AP(std::string url, std::string config_addr)
{
    // std::string transport_method = url.substr(0, url.find("://"));
    // if (transport_method == "serial")
    // {
    //     port_ = boost::make_shared<TransportSerial>(url);
    //     time_out_ = 3.5;//default 500
    //     hflinkmodbus_ = boost::make_shared<HFLink_Modbus>(&my_robot_  , 0x01 , 0x11);
    //     timer_.reset(new boost::asio::deadline_timer(*(port_->getIOinstace()),
    //                                                  boost::posix_time::milliseconds(time_out_)));
    // }else if (transport_method == "udp")
    // {
    // }else if (transport_method == "tcp")
    // {
    // }

    //first modify ************************
    if (url == "tcp")
    {
       // boost::asio::io_service io_service;
	    tcp::endpoint endpoint(address::from_string("192.168.0.207"), 502);

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
        // do nothing 
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






bool HIGO_AP::updateCommand(const MotorModbusCommand &command, int &count,int read_or_write)
{


    if(read_or_write==1)
    {
         sendCommandModbus(command);
         readCommandModbus1(command);
    }
    else if(read_or_write==0)
    {

        sendCommandModbus(command);
        readCommandModbus0(command);
    }

    return true;
}


  void HIGO_AP::updateRobot()
  {
     hflinkmodbus_->datatUpdate();
  }
