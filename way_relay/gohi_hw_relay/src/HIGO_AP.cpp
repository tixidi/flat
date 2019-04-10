#include <gohi_hw/HIGO_AP.h>




HIGO_AP::HIGO_AP(std::string url, std::string config_addr)
{
    //first modify****************************
    // std::string transport_method = url.substr(0, url.find("://"));
    // if (transport_method == "serial")
    // {
    //     port_ = boost::make_shared<TransportSerial>(url);
    //     time_out_ =50;//default 500
    //     hflinkmodbus_ = boost::make_shared<HFLink_Modbus>(&my_robot_  , 0x01 , 0x11);
    //     timer_.reset(new boost::asio::deadline_timer(*(port_->getIOinstace()),
    //                                                  boost::posix_time::milliseconds(time_out_)));
    // }else if (transport_method == "udp")
    // {
    // }else if (transport_method == "tcp")
    // {
    // }
    if (url == "tcp")
    {
       // boost::asio::io_service io_service;
	    tcp::endpoint endpoint(address::from_string("192.168.0.202"), 502);

        // 直接从 new 操作符的返回值构造
	    client_ptr new_session(new client(io_service, endpoint));
   

        client_tcp_=new_session;
        time_out_ =100;//default 500
        hflinkmodbus_ = boost::make_shared<HFLink_Modbus>(&my_robot_  , 0x01 , 0x11);
        timer_.reset(new boost::asio::deadline_timer(io_service,boost::posix_time::milliseconds(time_out_)));


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


bool HIGO_AP::updateCommand(const MotorModbusCommand &command, int count,int read_or_write)
{
    boost::asio::deadline_timer cicle_timer_(io_service);
    cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out_));
    if(read_or_write==2)
    {
        sendCommandModbus(command);
    }
    // first modify****************************
    readCommandModbus();
    
    return true;
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




//first modify   new    
bool HIGO_AP::dataAnalysisCall_R(const unsigned char rx_data){
   if(modbus_receive_state_==0)
    {
        if (rx_data == '<')//get slave addr
        {
            modbus_receive_state_ = 1;
        }
    }
    else if(modbus_receive_state_==1)
    {
        if (rx_data == '>')
        {
            modbus_receive_state_ = 0;
            // rx_buffer[1]=rx_data;
             // std::cerr<<"1"<<std::endl;
            return true;
        }
        else 
            modbus_receive_state_ = 0;
    } 
    
    return false;

}

unsigned char HIGO_AP::crc_high_first(unsigned char *ptr, unsigned short len)
{
	unsigned char i = 0; 
	 unsigned short n = 0;
	unsigned char crc=0xCC;
	for(n = 0; n < len; n++)
	{
		crc ^= *(ptr+n);  
		for (i=8; i>0; --i)
		{ 
			if (crc & 0x80)
				crc = (crc << 1)^0x31;
			else
				crc = (crc << 1);
		}
	}

	return (unsigned char)(crc & 0xFF); 
}

int HIGO_AP::calculateCrc(unsigned char data1, unsigned char data2)
{
	int result =0;
    if('0'<=data1 && data1<='9'){
       result =(data1 -48)*16;
    }else if('A'<=data1 && data1<='Z'){
       result =(data1 +10-65)*16;
    }
    if('0'<=data2 && data2<='9'){
       result +=(data2 -48);
    }else if('A'<=data2 && data2<='Z'){
       result +=(data2 +10-65);
    }
	return result;
}

int HIGO_AP::dataAnalysis(const unsigned char *data,int len)
{
	int result =0;
    unsigned char data1 =data[len-25];
    unsigned char data2 =data[len-24];
    if('0'<=data1 && data1<='9'){
       result =(data1 -48)*16;
    }else if('A'<=data1 && data1<='Z'){
       result =(data1 +10-65)*16;
    }
    if('0'<=data2 && data2<='9'){
       result +=(data2 -48);
    }else if('A'<=data2 && data2<='Z'){
       result +=(data2 +10-65);
    }
    std::cerr<<result<<std::endl;
	return result;
}


bool HIGO_AP::dataAnalysisCall(uint16_t rx_data)
{ 
    static int modbus_receive_state_ =0;
    if(modbus_receive_state_==0)
    {
        if (rx_data == send_data_buf[modbus_receive_state_])//get slave addr
        {
                modbus_receive_state_ = 1;
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
            modbus_receive_state_ = 0;
        }
    } 
    else if(modbus_receive_state_==2)
    {
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 3;
        }
        else
            modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==3)
    {                                  
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 4;
        }
        else
            modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==4)
    {          
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 5;
            //  std::cerr<<"55"<<std::endl;
        }
        else
        modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==5)
    {          
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 6;
            //  std::cerr<<"55"<<std::endl;
        }
        else
        modbus_receive_state_ = 0;
    }
    else if(modbus_receive_state_==6)
    {          
        if (rx_data == send_data_buf[modbus_receive_state_])
        {
            modbus_receive_state_ = 7;
            //  std::cerr<<"55"<<std::endl;
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

