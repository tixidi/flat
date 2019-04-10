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
	    tcp::endpoint endpoint(address::from_string("192.168.0.200"), 502);

        // 直接从 new 操作符的返回值构造
	    client_ptr new_session(new client(io_service, endpoint));
   

        client_tcp_=new_session;
        read_time_out_ =15;//default 500
        write_time_out_ =10;
        time_out_ =10;
        stair_position_complete_state =0;
        stair_position_temp =0;
        error_state =0;
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



bool HIGO_AP::updateCommand(const MotorModbusCommand &command, int count,int read_or_write,int relay_on_or_off)
{
    boost::asio::deadline_timer cicle_timer_(io_service);
    cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out_));
    if(read_or_write==0)
    {
        
        sendCommandModbus(command,relay_on_or_off);
        //first modify ************************
        readCommandModbus0();
    }

    return true;
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
bool HIGO_AP::dataAnalysisCall(uint16_t rx_data)
{ 
    static int modbus_receive_state_ =0;
    if(modbus_receive_state_==0)
    {
        if (rx_data == send_data_buf[modbus_receive_state_])//get slave addr
        {
                modbus_receive_state_ = 1;
                std::cerr<<"11"<<std::endl;
        }
    }
    else if(modbus_receive_state_==1)
    {
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 2;
        }
        else 
        {    
              std::cerr<<"22"<<std::endl;
            modbus_receive_state_ = 0;
        }
    } 
    else if(modbus_receive_state_==2)
    {
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 3;
        }
        else{
            std::cerr<<"33"<<std::endl;
            modbus_receive_state_ = 0;
        }
            
    }
    else if(modbus_receive_state_==3)
    {                                  
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 4;
        }
         else{
            std::cerr<<"44"<<std::endl;
            modbus_receive_state_ = 0;
        }
           
    }
    else if(modbus_receive_state_==4)
    {          
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 5;
             
        }
        else{
             std::cerr<<"55"<<std::endl;
            modbus_receive_state_ = 0;
        }
           
    }
    else if(modbus_receive_state_==5)
    {          
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 6;
              std::cerr<<"66"<<std::endl;
        }
        else
        modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==6)
    {          
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 7;
              std::cerr<<"77"<<std::endl;
        }
        else
        modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==7)
    {          
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ =0;
            return 1;
            //  std::cerr<<"55"<<std::endl;
        }
        else
        modbus_receive_state_ = 0;
    }
    return 0;
}

