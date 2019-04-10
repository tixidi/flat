#ifndef TRANSPORT_TCP_SERVER_H_
#define TRANSPORT_TCP_SERVER_H_



#define	TCP_RECV_DATA_PACKAGE_MAX_LENGTH			2048
#define	TCP_SEND_DATA_PACKAGE_MAX_LENGTH			2048
 
#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <list>
#include <boost/array.hpp>
#include <boost/thread.hpp>


#include<boost/asio/io_service.hpp>
#include<boost/asio/ip/tcp.hpp>
#include<boost/bind.hpp>
#include<boost/shared_ptr.hpp>
#include<boost/enable_shared_from_this.hpp>
#include<string>
#include<iostream>
#include<boost/asio/streambuf.hpp>
#include<boost/asio/placeholders.hpp>
#include<boost/asio.hpp>
#include <iostream>
#include <inttypes.h>
#include <vector>
#include <deque>
#include <queue>

#include <stdio.h>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

#include <boost/asio.hpp>
#include <iostream>
 using boost::asio::ip::address;

// // 异步服务器类
// class Server {

// private:

// 	// 服务实例
// 	boost::asio::io_service& ios_;

// 	// 接收器实例
// 	boost::asio::ip::tcp::acceptor acceptor_;

// 	// socket智能指针
// 	typedef boost::shared_ptr<boost::asio::ip::tcp::socket> socket_ptr;

// public:

// 	Server(boost::asio::io_service& _ios) : ios_(_ios),
// 		acceptor_(_ios, boost::asio::ip::tcp::endpoint(address::from_string("127.0.0.1"), 8888)) {
// 		// 默认执行
// 		start();
// 	}

// 	// 启动网络侦听的操作入口
// 	void start(void) {
// 		// 自定义的智能指针
// 		socket_ptr socket(new boost::asio::ip::tcp::socket(ios_));
// 		// 异步侦听，若有服务连接，则自动调用Server::handler_accept函数，并将error, socket传入作为参数
// 		acceptor_.async_accept(*socket,
// 			boost::bind(&Server::accept_handler, this,
// 			boost::asio::placeholders::error/* 此处作为占位符 */, socket));
// 	}

// 	// 请求者响应后触发的处理器
// 	void accept_handler(const boost::system::error_code& _ec, socket_ptr _socket) {
// 		// 错误码检测
// 		if (_ec) {
// 			return;
// 		}
// 		// 打印当前连接进来的客户端
// 		std::cerr << "client: " << _socket->remote_endpoint().address() << std::endl;
// 		// 异步发送 "hello CSND_Ayo" 消息到客户端，发送成功后，自动调用Server::write_handler函数
// 		_socket->async_write_some(boost::asio::buffer("hello CSND_Ayo"),
// 			boost::bind(&Server::write_handler, this,
// 			boost::asio::placeholders::error/* 此处作为占位符 */));
// 		// 启动新的异步监听
// 		start();
// 	}

// 	// 完成异步写操作后的处理器
// 	void write_handler(const boost::system::error_code& _ec) {
// 		std::cerr << "server: send message complete." << std::endl;
// 	}


// };


//---------------------------------------------------------------------
//---------------------------------------------------------------------
//---------------------------------------------------------------------
//---------------------------------------------------------------------
//---------------------------------------------------------------------


using namespace boost::asio;
 typedef unsigned long DWORD;
 using boost::asio::ip::address;
 
 
//发生数据回调函数
// typedef void (CALLBACK *SendDataCallback)(const boost::system::error_code& error,std::size_t bytes_transferred,DWORD dwUserData1,DWORD dwUserData2);
 
//接收数据回调函数
// typedef void (CALLBACK *RecvDataCallback)(const boost::system::error_code& error,char *pData,int nDataSize,DWORD dwUserData1,DWORD dwUserData2);
 

typedef boost::function<void (const boost::system::error_code& error,std::size_t bytes_transferred,DWORD dwUserData1,DWORD dwUserData2)>  SendDataCallback;   

typedef boost::function<void (const boost::system::error_code& error,char *pData,int nDataSize,DWORD dwUserData1,DWORD dwUserData2)>  RecvDataCallback;   


//tcp connection
class TcpConnection
{
public:	
    TcpConnection(io_service& ioService)
	: m_socket(ioService)
	{
		m_bDisconnect = false;
	}

	static TcpConnection * create(io_service& ioService)
	{
		return new TcpConnection(ioService);
	}
 

	virtual ~TcpConnection()
	{
		m_bDisconnect = true;
		m_socket.close();
	}
 
	ip::tcp::socket& socket()
	{
		return m_socket;
	}
 
	//发送数据
	inline int sendData(char *pData,int nDataSize,SendDataCallback fnCallback,DWORD dwUserData1,DWORD dwUserData2)
	{
		if(fnCallback == NULL)
		{
			//同步
			if(!m_socket.is_open())
			{
				return 0;
			}
			std::size_t  nSendedSize = boost::asio::write(m_socket,boost::asio::buffer(pData,nDataSize));
			if(nDataSize == nSendedSize)
			{
				return 0;
			}
			else
			{
				return nSendedSize;
			}
		}
		else
		{
			//异步
			if(!m_socket.is_open())
			{
				return 0;
			}
			memcpy(m_sendBuf.data(),pData,nDataSize);
			boost::asio::async_write(
				m_socket, 
				boost::asio::buffer(m_sendBuf.data(),nDataSize), 
				boost::bind(&TcpConnection::handle_write, this, 
				boost::asio::placeholders::error, 
				boost::asio::placeholders::bytes_transferred
				));
		}
	
		return 0;
	}
	//接收数据（同步）
	inline  int	recvDataBySync()
	{
		if(!m_socket.is_open())
		{
			return 0;
		}
		boost::system::error_code error;
		std::size_t nSize = m_socket.read_some(boost::asio::buffer(m_recvBuf),error);
		if(error != NULL)
		{
			//错误
			return 0;
		}
 
		return nSize;
	}
	//接收数据（异步）
	inline int	recvDataByAsync(RecvDataCallback  fnCallback,DWORD dwUserData1,DWORD dwUserData2)
	{
    
		// std::cerr<<" read connect! " <<std::endl;
		m_socket.async_read_some(boost::asio::buffer(m_recvBuf),
			boost::bind(&TcpConnection::handle_read,this, 
			boost::asio::placeholders::error, 
			boost::asio::placeholders::bytes_transferred
			));
 
		return 0;
	}
 
private:
 	bool	m_bDisconnect;											//是否断开连接
	ip::tcp::socket m_socket;
	boost::array<char,TCP_RECV_DATA_PACKAGE_MAX_LENGTH> m_recvBuf;	//接收数据缓冲区
	boost::array<char,TCP_SEND_DATA_PACKAGE_MAX_LENGTH> m_sendBuf;	//发送数据缓冲区

private:
	void handle_write(const boost::system::error_code& error,size_t bytes_transferred)
	{
	
		if(error != NULL)
		{
			m_bDisconnect = true;
			if(m_socket.is_open())
			{
				m_socket.close();
			}
			if(!m_bDisconnect)
			{
				// printf("close connect \n");
					std::cerr<<" close connect! " <<std::endl;
			}
 
			//发送数据失败
			return;
		}
 
		// printf("write data!!!");
		std::cerr<<" write data! " <<std::endl;
		
	}
 
	void handle_read(const boost::system::error_code& error,size_t bytes_transferred)
	{

		if(error != NULL)
		{
				if (error == boost::asio::error::eof || error == boost::asio::error::connection_reset)
				{
					std::cerr<<" error "<<std::endl;
					//boost::asio::error::eof  --对端方关闭连接（正常关闭套接字）
					//boost::asio::error::connection_reset --对端方关闭连接（暴力关闭套接字）
					//对端方关闭连接
					if(m_socket.is_open())
					{
						m_socket.close();
					}
					// printf("close connect \n");
					std::cerr<<" close connect! " <<std::endl;
					return;
				}
				else
				{
					if(m_socket.is_open())
					{
						m_socket.close();
					}
				}
				return;
		}

		char szMsg[128] = {0};
		memcpy(szMsg,m_recvBuf.data(),bytes_transferred);
		std::cerr<<" rev data is "<<std::endl;
		for(int i=0;i<bytes_transferred;i++)
		    std::cerr<<szMsg[i]<<" ";
		std::cerr<<" rev data end"<<std::endl;
	}
 

};


 
class TcpServer
{
public://address::from_string("127.0.0.1")
	TcpServer(io_service& ioService) : io_service_(ioService),m_acceptor(ioService, ip::tcp::endpoint(ip::tcp::v4(), 8888))
	{
		std::cerr << "Tcp server init " << std::endl;
		// m_funcConnectionHandler = NULL;
		m_nUserData = 0;
		m_bStop = false;
		
	}
 
	// //新建连接回调函数
	// typedef boost::function<void (TcpConnection * new_connection,int nUserData)>	CreateConnnectionCallbackHandler;
 
	// //设置新建连接回调函数
	// void setNewConnectionCallback(CreateConnnectionCallbackHandler fnHandler,int nUserData)
	// {
	// 	m_funcConnectionHandler = NULL;
	// 	m_nUserData = nUserData;
	// }
 
	//开始工作
	void startWork()
	{
		m_bStop = false;
		start_accept();
	}
 
	//停止工作
	void stopWork()
	{
		m_bStop = true;
		m_acceptor.close();
	}


private:
	void start_accept()
	{
		if(m_bStop)
		{
			return;
		}

    	TcpConnection *new_connection= TcpConnection::create(m_acceptor.get_io_service());
	
    
      

		m_acceptor.async_accept(
			new_connection->socket(), 
			boost::bind(&TcpServer::handle_accept, 
			this, 
			new_connection, 
			boost::asio::placeholders::error));
	}
 
	void handle_accept(TcpConnection *new_connection,
		const boost::system::error_code& error)
	{
		if (!error)
		{
			new_connection->sendData("abcdefg",strlen("abcdefg"),NULL,0,0);
			new_connection->recvDataByAsync(NULL,(DWORD)0,0);
			start_accept();
		}
	}
public: 
    void read_data()
	{
		// TcpConnection *new_connection= TcpConnection::create(m_acceptor.get_io_service());
		// std::cerr << "Tcp server read " << std::endl;
		// new_connection->recvDataByAsync(NULL,(DWORD)0,0);
			
	}


public: 
	ip::tcp::acceptor m_acceptor;
	// CreateConnnectionCallbackHandler m_funcConnectionHandler;
	int				  m_nUserData;
	bool			  m_bStop;
	
public:
	// //所有连接
	// std::list<TcpConnection *> g_listConnection;
	// //无效连接
	// std::list<TcpConnection *> g_listNoEffectConnection;

public:
	// void InsertNoEffectConnection(TcpConnection * pConnnection);
    // void DeleteNoEffectConnection();
    // void NewConnectionCallbackProcess(TcpConnection * new_connection,int nUserData);
    // void  RecvDataCallbackProcess(const boost::system::error_code& error,char *pData,int nDataSize,DWORD dwUserData1,DWORD dwUserData2);
	
	bool initializeTcpServer();
	void mainRun();

private:
    boost::asio::io_service &io_service_;
	
	
	boost::thread thread_;
	// locks
	boost::mutex port_mutex_;
	boost::mutex write_mutex_;
	boost::mutex read_mutex_;


};
 
 typedef boost::shared_ptr<TcpServer> TcpServer_ptr;


#endif
