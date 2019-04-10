#include <gohi_hw/HIGO_AP.h>




HIGO_AP::HIGO_AP(std::string url, std::string config_addr,std::string idConfig_addr)
{
    std::string transport_method = url.substr(0, url.find("://"));
    if (transport_method == "serial")
    {
        port_ = boost::make_shared<TransportSerial>(url);
        time_out_ =50;//default 500
        hflinkmodbus_ = boost::make_shared<HFLink_Modbus>(&my_robot_  , 0x01 , 0x11);
        timer_.reset(new boost::asio::deadline_timer(*(port_->getIOinstace()),
                                                     boost::posix_time::milliseconds(time_out_)));
    }else if (transport_method == "udp")
    {

    }else if (transport_method == "tcp")
    {
        
    }
    read_id_num_temp =0;
    set_car_speed_flag =0;
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
        initialize_ok_ = port_->initialize_ok();
    } else
    {
        std::cerr << "config file can't be opened, check your system" <<std::endl;
        initialize_ok_ = false;
    }

    //     first modify
    file_.open(idConfig_addr.c_str(), std::fstream::in);
    if (file_.is_open())
    {
        for (int i = 0; i < 8; i++)
        {
            std::string temp;
            id_num[i] =i+1;  //ID number
            file_ >> temp >> id_cmd_x_speed[i] >> id_cmd_Rz[i]>> id_cmd_stair_position[i]>>id_cmd_stair_speed[i]>>id_cmd_stair_type[i]>>id_cmd_roll_speed[i];
            std::cerr<< temp <<"   x ="<< id_cmd_x_speed[i] <<",RZ ="<< id_cmd_Rz[i]<<", stair position ="<<id_cmd_stair_position[i]<<", roll speed"<<id_cmd_roll_speed[i] \
                << ", stair speed ="<<id_cmd_stair_speed[i] << ", stair type ="<<id_cmd_stair_type[i] <<std::endl;
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

//first modify
bool HIGO_AP::dataAnalysis(ID_Info &id_info_)
{
     for (int i = 0; i < 8; i++)
    {
        if(id_info_.id_number==id_num[i])
        {
            if(read_id_num_temp == id_num[i]) return true;
            std::cerr<<"id number is exist,id number is"<<id_num[i]<<std::endl;
            id_info_.x_speed =id_cmd_x_speed[i];
            id_info_.Rz =id_cmd_Rz[i];
            id_info_.stair_position = id_cmd_stair_position[i];
            id_info_.stair_speed =id_cmd_stair_speed[i];
            id_info_.stair_type =id_cmd_stair_type[i];
            id_info_.roll_speed = id_cmd_roll_speed[i];
            
            set_car_speed_flag =1;
            return true;
             //function    
        }
    }
    // std::cerr<<"id number "<<id_info_.id_number<<"is not exist"<<id_info_.id_number<<std::endl;
    return false;
}


bool HIGO_AP::updateCommand(const MotorModbusCommand &command, int count,int read_or_write)
{
    boost::asio::deadline_timer cicle_timer_(*(port_->getIOinstace()));
    cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out_));
            
    if(read_or_write==1)
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

        Buffer data = port_->readBuffer();
        ack_ready_ = false;
        while (!ack_ready_)
        {
            for (int i = 0; i < data.size(); i++)
            {
                if (hflinkmodbus_->byteAnalysisCall(data[i]))
                {
                    // one package ack arrived  
                    ack_ready_ = true;         
                }
            }
            data = port_->readBuffer();
            if (cicle_timer_.expires_from_now().is_negative())
            {
                std::cerr<<"Timeout continue skip this package"<<std::endl;
                return false;
            }
        }
    }
    else if(read_or_write==0)
    {
        // update command set  data from embedded system
        if (hflink_command_set_[command] != 0)
        {
            int cnt = count % 100;
            if (cnt %  (100 / hflink_freq_[command]) == 0)
            {
                sendCommandModbus(command);
            } 
            else
            {
                // skip this package
                return false;
            }
        }

        Buffer data = port_->readBuffer();
        ack_ready_ = false;
        while (!ack_ready_)
        {
          
            for (int i = 0; i < data.size(); i++)
            {    

                if (hflinkmodbus_->byteAnalysisCall_R(data[i]))
                {
                    // one package ack arrived  
                    ack_ready_ = true;          
                }

            }
            data = port_->readBuffer();
            if (cicle_timer_.expires_from_now().is_negative())
            {
                return false;
            }
        }
    }

    return true;
}