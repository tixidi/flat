#ifndef TRANSPORT_UDP_CLIENT_H_
#define TRANSPORT_UDP_CLIENT_H_


//udp服务
 
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include <iostream>
#include <inttypes.h>
#include <vector>
#include <deque>
#include <queue>
#include <vector>
#include <iostream>
#include <algorithm>
#include <string>

#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::udp;
 
#define  UDP_DATA_PACKAGE_MAX_LENGTH		1024
 
//发送数据回调函数
typedef unsigned long DWORD;

typedef std::vector<char> Buffer;


typedef boost::function<void (const boost::system::error_code& error,std::size_t bytes_transferred,DWORD dwUserData1,DWORD dwUserData2)>  SendDataCallback;   

// typedef void (CALLBACK *SendDataCallback)(const boost::system::error_code& error,std::size_t bytes_transferred,DWORD dwUserData1,DWORD dwUserData2);//设置工作参数


 
class UdpLinkClient
{
public:
	UdpLinkClient(char *pUrl,char *pServiceName,unsigned short usPort,boost::asio::io_service &io_service);
	virtual ~UdpLinkClient(void);
 
	//
 
	//-------------------------------------------------------------------------------------
	// 功能：发送数据处理函数
	// 参数：nReserved1,nReserved2:保留
	// 返回：NULL:创建失败		其他:对象地址
	//-------------------------------------------------------------------------------------
	typedef boost::function<void* (const boost::system::error_code& error,std::size_t bytes_transferred,DWORD dwUserData1,DWORD dwUserData2)>	SendDataCallbackHandler;
 
	//开始
	int Start();
 
    bool initializeTcp();
	void mainRun();
	//停止
	int Stop();
 
	//发送数据
	int SendDataEx(udp::endpoint endpointRemote,char *pBuffer,int nBufferSize,SendDataCallback pfunc,DWORD dwUserData1,DWORD dwUserData2);
 
	//发送数据
	int SendData(char *pBuffer,int nBufferSize,bool bAsync);
 
	//启用接收数据服务(自动)
	int AutoRecvData();
 
	//启用接收数据服务(人工)
	int MmanualRecvData();
 
	//获取服务器端点
	udp::endpoint & GetServerEndPoint();
 
	//当收到对方端数据时，就进入本函数响应处理
	void handleRecvData(const boost::system::error_code& error,std::size_t bytes_transferred);
	//当发送数据给对方端成功之后响应处理
	void handleSendData(char *pBuffer,int nBufferSize,const boost::system::error_code& error,std::size_t bytes_transferred);
	void handleSendDataInner(SendDataCallback pfunc,DWORD dwUserData1,DWORD dwUserData2,const boost::system::error_code& error,std::size_t bytes_transferred);
	//void handleSendData(boost::shared_ptr<std::string> strMessage,const boost::system::error_code& error,std::size_t bytes_transferred);
 
	static void SendDataCallbackOuter(const boost::system::error_code& error,std::size_t bytes_transferred,DWORD dwUserData1,DWORD dwUserData2);

	// virtual Buffer readBuffer();
	void writeBuffer(Buffer &data);


private:
	void handle_write(const boost::system::error_code& error, size_t bytes_transferred) ;
	void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
	void start_a_read();
	void start_a_write();
    void HexToAscii( unsigned  char * pHex,  unsigned char * pAscii, int nLen);
 
protected:
	//接收数据
	void RecvDataProcess();
 
	//是否停止服务
	bool IsStop();
 
public:
	std::queue<Buffer> write_buffer_;
	std::queue<Buffer> read_buffer_;

private:
	udp::socket	*m_sockUdp;										//服务器的SOCKET
	udp::endpoint m_endpointRemote;								//收到数据时的端点信息
	udp::endpoint m_endpointServer;								//服务器的端点信息
	boost::array<char,UDP_DATA_PACKAGE_MAX_LENGTH> m_recvBuf;	//接收数据缓冲区
	bool m_bStop;												//停止服务
	char m_szUrl[256];											//服务器url
	char m_szServiceName[32];									//服务名，比如daytime，http
	unsigned short m_usPort;									//端口
	bool m_bReviceData;											//是否接收过数据	
	bool m_bReviceServerUse;									//是否启用接收服务

	boost::thread thread_;
    boost::asio::io_service &io_service_;
		// locks
	boost::mutex port_mutex_;
	boost::mutex write_mutex_;
	boost::mutex read_mutex_;
};
 
typedef boost::shared_ptr<UdpLinkClient> udp_client_ptr;


#endif /* TRANSPORT_TCP_H_ */
