#include <gohi_hw/HIGO_AP.h>
#include "string.h"
#include <gohi_hw/HIGO_ROS.h>



HIGO_AP::HIGO_AP(std::string url, std::string config_addr)
{
    //first modify ************************

    // std::string transport_method = url.substr(0, url.find("://"));
    // if (transport_method == "serial")
    // {
    //     port_ = boost::make_shared<TransportSerial>(url);
    //     time_out_ =50;//default 500
    //     hflinkmodbus_ = boost::make_shared<HFLink_Modbus>(&my_robot_  , 0x01 , 0x11);
    //     timer_.reset(new boost::asio::deadline_timer(*(port_->getIOinstace()),
    //                                                  boost::posix_time::milliseconds(time_out_)));
    // }else if (transport_method == "udp")
    // {}

    //first modify ************************
    if (url == "tcp")
    {
       // boost::asio::io_service io_service;
	    tcp::endpoint endpoint(address::from_string("192.168.0.203"), 502);

        // 直接从 new 操作符的返回值构造
	    client_ptr new_session(new client(io_service, endpoint));
   

        client_tcp_=new_session;
        time_out_ =600;//default 500
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
            
    // if(read_or_write==1)
    // {
    //     // update command set  data from embedded system
    //     if (hflink_command_set_[command] != 0)
    //     {
    //         int cnt = count % 100;
    //         if (cnt %  (100 / hflink_freq_[command]) == 0)
    //         {
    //             sendCommandModbus(command);
    //         } else
    //         {
    //             // skip this package
    //             return false;
    //         }
    //     }

    //     Buffer data = port_->readBuffer();
    //     ack_ready_ = false;
    //     while (!ack_ready_)
    //     {
    //         for (int i = 0; i < data.size(); i++)
    //         {
    //             if (hflinkmodbus_->byteAnalysisCall(data[i]))
    //             {
    //                 // one package ack arrived  
    //                 ack_ready_ = true;         
    //             }
    //         }
    //         data = port_->readBuffer();
    //         if (cicle_timer_.expires_from_now().is_negative())
    //         {
    //             std::cerr<<"Timeout continue skip this package"<<std::endl;
    //             return false;
    //         }
    //     }
    // }
    //  first modify   new --data-------------------------------------------------------------------
    //else
     if(read_or_write==2)
    {
        // update command set  data from embedded system
        if (hflink_command_set_[command] != 0)
        {
            int cnt = count % 100;
            if (cnt %  (100 / hflink_freq_[command]) == 0)
            {
                sendCommandModbus(command);
            } else
            {
                // skip this package
                return false;
            }
        }
    // first modify****************************
       readCommandModbus();


    }
    return true;
}


//first modify   new    
bool HIGO_AP::dataAnalysisCall(const unsigned char rx_data){
   if(modbus_receive_state_==0)
    {
        if (rx_data == '<')//get slave addr
        {
            modbus_receive_state_ = 1;
            // CrcRecValue=0;
            // rev_sum_changed=0;
            // rx_buffer[0]=rx_data;
            // std::cerr<<"0"<<std::endl;
        }
    }
    else if(modbus_receive_state_==1)
    {
        if (rx_data == '>')
        {
            modbus_receive_state_ = 2;
            // rx_buffer[1]=rx_data;
             // std::cerr<<"1"<<std::endl;
            return true;
        }
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

int HIGO_AP::getResult(unsigned char data1, unsigned char data2)
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

void HIGO_AP::dataAnalysis(const unsigned char *data,int len, BMSBattery &bms_battery_temp_)
{
	int result =0;
    unsigned char data1 =data[len-25];
    unsigned char data2 =data[len-24];
    result =getResult(data1,data2);
    bms_battery_temp_.battery_capacity_percentage =result;
    // std::cerr<<result<<std::endl;
	data1 =data[len-125];
    data2 =data[len-124];   
    result =getResult(data1,data2);
    bms_battery_temp_.battery_health =result;
    data1 =data[len-123];
    data2 =data[len-122];   
    result =getResult(data1,data2);
    bms_battery_temp_.total_voltage =result;
    data1 =data[len-121];
    data2 =data[len-120];   
    result =getResult(data1,data2);
    bms_battery_temp_.total_voltage = (bms_battery_temp_.total_voltage *256 +result)*2;//单位：mv
    data1 =data[len-23];
    data2 =data[len-22];   
    result =getResult(data1,data2);
    bms_battery_temp_.current_capacity =result;//单位：mv
    data1 =data[len-21];
    data2 =data[len-20];   
    result =getResult(data1,data2);
    bms_battery_temp_.current_capacity =(float)(bms_battery_temp_.current_capacity *256 +result)*0.1;//单位：AH
}
