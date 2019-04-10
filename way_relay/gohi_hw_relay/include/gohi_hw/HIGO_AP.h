#pragma once

#ifndef HIGO_AP_H_
#define HIGO_AP_H_

#include <fstream>

//first modify
// #include <gohi_hw/transport_serial.h>
#include <gohi_hw/transport_tcp.h>

#include <hf_link_modbus.h>
#include <cstdlib>

class HIGO_AP
{
public:
	 HIGO_AP(std::string url, std::string config_addr);

 //   bool updateCommand(const Command &command, int count);
    bool updateCommand(const MotorModbusCommand &command, int count);
    bool updateCommand(const MotorModbusCommand &command, int count,int read_or_write);
    bool updateCommand(const MotorModbusCommand &command, int count,int read_or_write,int relay_on_or_off);

    //  first modify     new 
    bool dataAnalysisCall_R(const unsigned char rx_data);
    unsigned char crc_high_first(unsigned char *ptr, unsigned short len);
    int calculateCrc(unsigned char data1, unsigned char data2);
    int dataAnalysis(const unsigned char *data,int len);
    bool dataAnalysisCall(uint16_t rx_data);


    inline RobotAbstract* getRobotAbstract()
    {
        return &my_robot_;
    }

    inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
    {
        return client_tcp_->getIOinstace();
    }

    bool reconfig()
    {

    }

    inline bool initialize_ok () const
    {
        return initialize_ok_;
    }

    // inline void checkHandshake()
    // {
    //     if (hflink_->getReceiveRenewFlag(SHAKING_HANDS)==1)
    //     {
    //         sendCommand(SHAKING_HANDS);
    //         std::cout<<"send shake hands"<<std::endl;
    //     }
    // }
    inline void sendCommandModbus(const MotorModbusCommand command_state)
    {
        hflinkmodbus_->masterSendCommand(command_state);
        Buffer data(hflinkmodbus_->getSerializedData(), hflinkmodbus_->getSerializedLength() + hflinkmodbus_->getSerializedData());
        client_tcp_->writeBuffer(data);
    }
    inline void sendCommandModbus(const MotorModbusCommand command_state,int relay_on_or_off)
    {
        hflinkmodbus_->masterSendCommand(command_state,relay_on_or_off);
        Buffer data(hflinkmodbus_->getSerializedData(), hflinkmodbus_->getSerializedLength() + hflinkmodbus_->getSerializedData());
        client_tcp_->writeBuffer(data);
        // for(int i=0;i<data.size();i++){
        //     send_data_buf[i] =(int)data[i];
        //     std::cerr<< send_data_buf[i]<<" ";
        // }
        // std::cerr<<std::endl;
    }

     // first modify****************************
    inline bool readCommandModbus()
    {
         boost::asio::deadline_timer cicle_timer_(io_service);
        cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out_));
        Buffer data = client_tcp_->readBuffer();
        unsigned char data_buff[160] ={0};
        int cnt =0;
        ack_ready_ = false;
        modbus_receive_state_ =0;
        while (!ack_ready_)
        {   
           
            for (int i = 0; i < data.size(); i++)
            {           
       
               // std::cerr<<" the data "<<i<<" is "<<data[i]<<std::endl;             
                data_buff[cnt++] =data[i];
                //data_buff[cnt] ='\0';
                //std::cerr<<" the data is "<<data_buff<<std::endl; 
                if (dataAnalysisCall_R(data[i]))
                {
                    // one package ack arrived 
                    data_buff[cnt] ='\0';
                    unsigned char data_buff_temp[160];
                    for(int i=0; i<strlen((const char *)data_buff)-4;i++){
                        data_buff_temp[i] =data_buff[i+1];
                    }
                    data_buff_temp[strlen((const char *)data_buff)-4] ='\0';
                    int CRC_data=(int)crc_high_first(data_buff_temp,strlen((const char *)data_buff_temp));
                    if(strlen((const char *)data_buff_temp)!=153){
                        std::cerr<<" the data is ERROR!"<<data_buff<<std::endl;
                        return false;
                    }

                    if(CRC_data == calculateCrc(data_buff[strlen((const char *)data_buff)-3],data_buff[strlen((const char *)data_buff)-2])){
                        std::cerr<<" the data is OK"<<data_buff<<std::endl; 
                        publish_data =dataAnalysis(data_buff,strlen((const char *)data_buff));
                    }
                    else{
                        std::cerr<<" the data is ERROR!"<<data_buff<<std::endl;
                        return false;
                    }
                    // std::cerr<<" the data is "<<data_buff<<std::endl; 
                    //  std:: cerr<<" the crc is "<<CRC_data<<std::endl; 
                    
                    ack_ready_ = true;         
                }
            }
            data = client_tcp_->readBuffer();

            //  std::cerr << "rea Out" <<std::endl;
            if (cicle_timer_.expires_from_now().is_negative())
            {
                std::cerr<<"Timeout continue skip this package"<<std::endl;
                return false;
            }
        }
    }
    
   //first modify***************************
    inline void readCommandModbus0()
    {
        boost::asio::deadline_timer cicle_timer_(io_service);
         cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out_));
        Buffer data=client_tcp_->readBuffer();   
        // std::cerr << "read3:"<< data.size()<<std::endl;
        ack_ready_ = false;
        while (!ack_ready_)
        {
            for (int i = 0; i < data.size(); i++)
            {    
                if (dataAnalysisCall((uint16_t)data[i]))
                {
                     std::cerr << "read is ok"<<std::endl;
                    // one package ack arrived  
                    ack_ready_ = true;         
                }
            }
            data = client_tcp_->readBuffer();
            if (cicle_timer_.expires_from_now().is_negative())
            {
                // std::cerr<<"Timeout continue skip this package"<<std::endl;
                return;
            }
        }
    }



    //first modify 
    int publish_data;
private:
    //  first modify************************************  
    // boost::shared_ptr<Transport> port_;
    boost::shared_ptr<client> client_tcp_;
    boost::asio::io_service io_service;


 //   boost::shared_ptr<HFLink> hflink_;
    boost::shared_ptr<HFLink_Modbus> hflinkmodbus_;
    
    boost::shared_ptr<boost::asio::deadline_timer> timer_;

    //for reading config file
    std::fstream file_;
    bool initialize_ok_;

    uint16_t send_data_buf[16];  //发送信息数据的缓存，用于与接受回来的数据进行比较
    //for updating data
    // int hflink_command_set_[LAST_COMMAND_FLAG];
    // int hflink_freq_[LAST_COMMAND_FLAG];
    // int hflink_count_[LAST_COMMAND_FLAG];
    // int hflink_command_set_current_[LAST_COMMAND_FLAG];

    int hflink_command_set_[LAST_COMMAND_FLAG_];
    int hflink_freq_[LAST_COMMAND_FLAG_];
    int hflink_count_[LAST_COMMAND_FLAG_];
    int hflink_command_set_current_[LAST_COMMAND_FLAG_];


    int time_out_;
    bool time_out_flag_;

    
    boost::mutex wait_mutex_;
    bool ack_ready_;
    void timeoutHandler(const boost::system::error_code &ec);

    // inline uint8_t checkUpdate(const Command command_state)
    // {
    //     if (hflink_command_set_current_[command_state] & hflink_->getReceiveRenewFlag(command_state))
    //     {
    //         return 1;
    //     }
    //     if (hflink_command_set_current_[command_state] == 0 ) return 1;
    //     return 0;
    // }




    // a single object for robot
    RobotAbstract my_robot_;

    //first modify 
    int modbus_receive_state_;

};



#endif
