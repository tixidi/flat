#pragma once

#ifndef HIGO_AP_H_
#define HIGO_AP_H_
#include <fstream>
#include <ros/ros.h>
//first modify***************
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
    bool updateCommand( const MotorModbusCommand &command,int &count,int read_or_write);

    

    void updateRobot();

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



private:
//first modify *************************
    // boost::shared_ptr<Transport> port_;
    boost::shared_ptr<client> client_tcp_;
    boost::asio::io_service io_service;

 //   boost::shared_ptr<HFLink> hflink_;
    boost::shared_ptr<HFLink_Modbus> hflinkmodbus_;
    
    boost::shared_ptr<boost::asio::deadline_timer> timer_;

    //for reading config file
    std::fstream file_;
    bool initialize_ok_;


    int hflink_command_set_[LAST_COMMAND_FLAG_];
    int hflink_freq_[LAST_COMMAND_FLAG_];
    int hflink_count_[LAST_COMMAND_FLAG_];
    int hflink_command_set_current_[LAST_COMMAND_FLAG_];


    int write_time_out_;
    int read_time_out_;
    bool time_out_flag_;
    boost::mutex wait_mutex_;
    bool ack_ready_;
    void timeoutHandler(const boost::system::error_code &ec);



    inline void sendCommandModbus(const MotorModbusCommand command_state)
    {
        hflinkmodbus_->masterSendCommand(command_state);
        Buffer data(hflinkmodbus_->getSerializedData(), hflinkmodbus_->getSerializedLength() + hflinkmodbus_->getSerializedData());
        client_tcp_->writeBuffer(data);
        // for(int i=0;i<data.size();i++){
        //     if(command_state == SET_CAR1_LEFT_SPEED_CONTROL){
        //         std::cerr << "write motor1 :"<< (uint16_t)data[i]<<std::endl;
        //     }else if(command_state == SET_CAR1_RIGHT_SPEED_CONTROL)
        //         std::cerr << "write motor2:"<< (uint16_t)data[i]<<std::endl;
        // }
    }


 //first modify**0*************************
    inline void readCommandModbus0(const MotorModbusCommand &command)
    {
         static int time_out_count=0;
    
         boost::asio::deadline_timer cicle_timer_(io_service);
         cicle_timer_.expires_from_now(boost::posix_time::millisec(read_time_out_));
         
         ros::Time currentTime = ros::Time::now();
         time_out_count++ ;

         
        ack_ready_ = false;  
        
        Buffer data=client_tcp_->readBuffer();   
        while (!ack_ready_)
        {
            for (int i = 0; i < data.size(); i++)
            {    
                if (hflinkmodbus_->byteAnalysisCall_R(data[i]))
                {
  
                        if(command == READ_LAXIAN_POSITION)
                        {
                            
                            std::cerr<<"the "<< time_out_count<<" count's "<<"READ_LAXIAN_POSITION   receive ok"<<std::endl;
                            
                        } 
                        else if(command == READ_RFID_REG_DATA)
                        {
                            std::cerr<<"the "<< time_out_count<<" count's "<<"READ_RFID_REG_DATA   receive ok"<<std::endl;
                        
                        }  
                        else if(command == READ_THERMOMETER_REG_DATA)
                        {
                            std::cerr<<"the "<< time_out_count<<" count's "<<"READ_THERMOMETER_REG_DATA   receive ok"<<std::endl;
                        
                        }

                    // one package ack arrived  
                    ack_ready_ = true;    
                    return;     
                }
            }
            data = client_tcp_->readBuffer();
            if (cicle_timer_.expires_from_now().is_negative())
            {
                if(command == READ_LAXIAN_POSITION)
                {
                    std::cerr<<"the "<< time_out_count<<" count's "<<"READ_LAXIAN_POSITION   Timeout  "<< (float)(ros::Time::now() - currentTime).toSec()<<std::endl;
                     
                } 
                else if(command == READ_RFID_REG_DATA)
                {
                    std::cerr<<"the "<< time_out_count<<" count's "<<"READ_RFID_REG_DATA  Timeout  "<< (float)(ros::Time::now() - currentTime).toSec()<<std::endl;
                   
                }  
                else if(command == READ_THERMOMETER_REG_DATA)
                {
                    std::cerr<<"the "<< time_out_count<<" count's "<<"READ_THERMOMETER_REG_DATA  Timeout  "<< (float)(ros::Time::now() - currentTime).toSec()<<std::endl;
                   
                }
   
                ros::Time currentTime = ros::Time::now();
                // time_out_count=0;
               

                return;
            }
        }
    }
    inline void readCommandModbus1(const MotorModbusCommand &command)
    {
          static int time_out_count=0;
        boost::asio::deadline_timer cicle_timer_(io_service);
        cicle_timer_.expires_from_now(boost::posix_time::millisec(write_time_out_));

        ros::Time currentTime = ros::Time::now();                   
        time_out_count++ ;

        ack_ready_ = false;
        Buffer data=client_tcp_->readBuffer();           
        while (!ack_ready_)
        {

            
            for (int i = 0; i < data.size(); i++)
            {     

            
                // std::cerr<<"the "<< i<<" count's "<<(int)data[i]<<"   "<<data.size()<<std::endl;

                if (hflinkmodbus_->byteAnalysisCall(data[i]))
                {
                    if(command == SET_RFID_REG_DATA)
                    {
                           
                    std::cerr<<"the "<< time_out_count<<" count's "<<"SET_RFID_REG_DATA write receive ok  "<<std::endl;
                    }


                    ack_ready_ = true;       
                    return;  
                }
            }
            data = client_tcp_->readBuffer();
           
            if (cicle_timer_.expires_from_now().is_negative())
            {
                if(command == SET_RFID_REG_DATA)
                {
                   std::cerr<<"the "<< time_out_count<<" count's "<<"SET_RFID_REG_DATA write Timeout continue skip this package  "<< (float)(ros::Time::now() - currentTime).toSec()<<std::endl;
                }

                ros::Time currentTime = ros::Time::now();
                // time_out_count=0;
                
                return;
            }
        }
    }  

    // a single object for robot
    RobotAbstract my_robot_;

};



#endif
