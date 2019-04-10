#include <gohi_hw/HIGO_AP.h>




HIGO_AP::HIGO_AP(std::string url, std::string config_addr)
{

    if (url == "tcp")
    {


    }else if(url == "udp"){


    }
    else if(url == "tcp_server"){

        TcpServer_ptr server(new TcpServer(io_service));

        server_tcp_=server;

        server->startWork();




        // io_service.run();
        // // server->initializeTcpServer();

	    // int n = 0;
		// while(true)
		// {

		// 	// pTcpConnectionObject->sendData("111",3,NULL,0,0);
		// 	// pTcpConnectionObject->recvDataByAsync(RecvDataCallbackProcess,(int)pTcpConnectionObject,0);
		//     server->read_data();
 
		// 	io_service.poll();
		// 	// Sleep(200);
		// 	n++;
		// 	if(n > 1000)
		// 	{
		// 		break;
		// 	}
        // }
        // server->stopWork();




/*思路3*/
		// std::cout << "server start." << std::endl;
		// // 建造服务对象
		// boost::asio::io_service ios;
		// // 构建Server实例
		// Server server(ios); 
		// // 启动异步调用事件处理循环
		// ios.run();
 

 
    }

    //process the config file
    // file_.open(config_addr.c_str(), std::fstream::in);
    // if (file_.is_open())
    // {
    //     for (int i = 0; i < LAST_COMMAND_FLAG_; i++)
    //     {
    //         std::string temp;
    //         file_ >> temp >> hflink_command_set_[i] >> hflink_freq_[i];
    //         std::cout<< temp << hflink_command_set_[i] << hflink_freq_[i]<<std::endl;
    //     }
    //     file_.close();
    //     // initialize_ok_ = port_->initialize_ok();
    // } else
    // {
    //     std::cerr << "config file can't be opened, check your system" <<std::endl;
    //     // initialize_ok_ = false;
    // }
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
    	// server_tcp_->read_data();
 
		io_service.poll();
        std::cerr << " read is  " << std::endl;

        //  static int time_out_count=0;

         
        //  boost::asio::deadline_timer cicle_timer_(io_service);
        //  cicle_timer_.expires_from_now(boost::posix_time::millisec(read_time_out_));
         
        //  ros::Time currentTime = ros::Time::now();
        //  time_out_count++ ;
        //  server_tcp_->read_data();
            
    // if(read_or_write==2)
    // {
    //     Buffer data = client_tcp_->readBuffer();
        
    //     ack_ready_ = false;
    //     while (!ack_ready_)
    //     {          
    //         for (int i = 0; i < data.size(); i++)
    //         {    
    //             if (hflinkmodbus_->byteAnalysisCall_R_FromPAD(data[i]))
    //             {
    //                 // one package ack arrived  
    //                 ack_ready_ = true;          
    //             }

    //         }
    //         data = client_tcp_->readBuffer();
    //         if (cicle_timer_.expires_from_now().is_negative())
    //         {
    //             // std::cerr<<"Timeout continue skip this package"<<std::endl;
    //             return false;
    //         }
    //     }
    // }
    // else if(read_or_write==3)
    // {
    //     // update command set  data from embedded system
    //     if (hflink_command_set_[command] != 0)
    //     {
    //             sendCommandModbus(command);
    //     }


            
    //                 // TcpConnection * pTcpConnectionObject = NULL;
    //                 // std::list<TcpConnection *>::iterator iter,iterEnd;
                    
    //                 // int n = 0;
    //                 // while(true)
    //                 // {
    //                 //     //删除无效连接
    //                 //     server->DeleteNoEffectConnection();
            
    //                 //     //遍历
    //                 //     iter = server->g_listConnection.begin();
    //                 //     iterEnd = server->g_listConnection.end();
    //                 //     for(iter; iter!=iterEnd; iter++)
    //                 //     {
    //                 //         pTcpConnectionObject = *iter;
    //                 //         pTcpConnectionObject->sendData("111",3,NULL,0,0);
    //                         // pTcpConnectionObject->recvDataByAsync(NULL,(DWORD)pTcpConnectionObject,0);
    //                     // }
            
                       
    //                     // Sleep(200);
    //                     // n++;
    //                     // if(n > 1000)
    //                     // {
    //                     //     break;
    //                 //     // }
    //                 // }
                    

    // }


    return true;
}

