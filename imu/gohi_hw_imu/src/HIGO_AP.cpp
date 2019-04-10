#include <gohi_hw/HIGO_AP.h>




HIGO_AP::HIGO_AP(std::string url, std::string config_addr)
{
    std::string transport_method = url.substr(0, url.find("://"));
    if (transport_method == "serial")
    {
        port_ = boost::make_shared<TransportSerial>(url);
        time_out_ =20;//default 500
        hflinkmodbus_ = boost::make_shared<HFLink_Modbus>(&my_robot_  , 0x01 , 0x11);
        timer_.reset(new boost::asio::deadline_timer(*(port_->getIOinstace()),
                                                     boost::posix_time::milliseconds(time_out_)));
    }else if (transport_method == "udp")
    {
    }else if (transport_method == "tcp")
    {
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
        initialize_ok_ = port_->initialize_ok();
    } else
    {
        std::cerr << "config file can't be opened, check your system" <<std::endl;
        initialize_ok_ = false;
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


// bool HIGO_AP::updateCommand(const MotorModbusCommand &command, int count,int read_or_write)
// {
//     boost::asio::deadline_timer cicle_timer_(*(port_->getIOinstace()));
//     cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out_));
            
//     if(read_or_write==1)
//     {
//         // update command set  data from embedded system
//         if (hflink_command_set_[command] != 0)
//         {
//             int cnt = count % 100;
//             if (cnt %  (100 / hflink_freq_[command]) == 0)
//             {
//                 sendCommandModbus(command);
//             } else
//             {
//                 // skip this package
//                 return false;
//             }
//         }

//         Buffer data = port_->readBuffer();
//         ack_ready_ = false;
//         while (!ack_ready_)
//         {
//             for (int i = 0; i < data.size(); i++)
//             {
//                 if (hflinkmodbus_->byteAnalysisCall(data[i]))
//                 {
//                     // one package ack arrived  
//                     ack_ready_ = true;         
//                 }
//             }
//             data = port_->readBuffer();
//             if (cicle_timer_.expires_from_now().is_negative())
//             {
//                 std::cerr<<"Timeout continue skip this package"<<std::endl;
//                 return false;
//             }
//         }
//     }
//     else if(read_or_write==0)
//     {
//         // update command set  data from embedded system
//         if (hflink_command_set_[command] != 0)
//         {

//                 sendCommandModbus(command);
   
//         }

//         Buffer data = port_->readBuffer();
//         ack_ready_ = false;
//         while (!ack_ready_)
//         {
          
//             for (int i = 0; i < data.size(); i++)
//             {    

//                 if (hflinkmodbus_->byteAnalysisCall_R(data[i]))
//                 {
//                     // one package ack arrived  
//                     ack_ready_ = true;          
//                 }

//             }
//             data = port_->readBuffer();
//             if (cicle_timer_.expires_from_now().is_negative())
//             {
//                 return false;
//             }
//         }
//     }

//     return true;
// }