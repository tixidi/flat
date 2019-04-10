#include <gohi_hw/transport_udp_client.h>

// #include "StdAfx.h"
// #include "UdpLinkClient.h"
#include <boost/exception/all.hpp>
 
 
UdpLinkClient::UdpLinkClient(char *pUrl,char *pServiceName,unsigned short usPort,boost::asio::io_service &io_service):
io_service_(io_service)
{
	m_bStop = false;
	m_usPort = usPort;
	m_sockUdp = NULL;

	
	memset(m_szUrl,0,sizeof(m_szUrl));
	memset(m_szServiceName,0,sizeof(m_szServiceName));
	if(pUrl != NULL)
	{
		strcpy(m_szUrl,pUrl);
	}
	if(pServiceName != NULL)
	{
		strcpy(m_szServiceName,pServiceName);
	}
	m_bReviceData = false;
	m_bReviceServerUse = false;
}
 
UdpLinkClient::~UdpLinkClient(void)
{
	if(m_sockUdp != NULL)
	{
		m_sockUdp->close();
		delete m_sockUdp;
		m_sockUdp = NULL;
	}
}
 
//开始
int UdpLinkClient::Start()
{
	try
	{
		if(strlen(m_szServiceName) > 0)
		{
			//查询连接的IP域名和端口
			udp::resolver resolver(io_service_);
			//udp::resolver::query query("www.google.com","http");
			udp::resolver::query query(m_szUrl,m_szServiceName);
			m_endpointServer = *resolver.resolve(query);
		}
		else
		{
			m_endpointServer = udp::endpoint(boost::asio::ip::address_v4::from_string(m_szUrl), m_usPort);
		}
		m_sockUdp = new udp::socket(io_service_);
		m_sockUdp->open(udp::v4());
        m_sockUdp->set_option(boost::asio::socket_base::reuse_address(true));
        //使用WSAIoctl设置UDP socket的工作模式，让其忽略这个错误(Windows UDP socket recvfrom返回10054错误的解决办法)
        bool bNewBehavior = false;
        DWORD dwBytesReturned = 0;
        //  WSAIoctl(m_sockUdp->native(), SIO_UDP_CONNRESET, &bNewBehavior, sizeof bNewBehavior, NULL, 0, &dwBytesReturned, NULL, NULL);
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
	}
	catch (boost::exception& e)
	{
		//diagnostic_information(e);
		return -1;
	}
 
    m_bStop = false;
	m_bReviceData = false;
	m_bReviceServerUse = false;
    
	return 0;
}


bool UdpLinkClient::initializeTcp()
{

    try
    {
        thread_ = boost::thread(boost::bind(&UdpLinkClient::mainRun, this));
    }
    catch(std::exception &e)
    {
        std::cerr << "Transport Serial thread create failed " << std::endl;
        std::cerr << "Error Info: " << e.what() <<std::endl; 
        return false;
    }

    return true;
}

void UdpLinkClient::mainRun()
{        
    io_service_.run();
}
 
//停止
int UdpLinkClient::Stop()
{
	m_bStop = true;
 
	return 0;
}
 
//是否停止服务
bool UdpLinkClient::IsStop()
{
	return m_bStop;
}
 
//获取服务器端点
udp::endpoint & UdpLinkClient::GetServerEndPoint()
{
	return m_endpointServer;
}
 
void UdpLinkClient::SendDataCallbackOuter(const boost::system::error_code& error,std::size_t bytes_transferred,DWORD dwUserData1,DWORD dwUserData2)
{
	int i = 0;
}
 
//发送数据
int UdpLinkClient::SendDataEx(udp::endpoint endpointRemote,char *pBuffer,int nBufferSize,SendDataCallback pfunc,DWORD dwUserData1,DWORD dwUserData2)
{
	m_sockUdp->async_send_to(boost::asio::buffer(pBuffer,nBufferSize),endpointRemote,boost::bind(&UdpLinkClient::handleSendDataInner,this,pfunc,dwUserData1,dwUserData2,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
 
	return 0;
}
 
//发送数据
int UdpLinkClient::SendData(char *pBuffer,int nBufferSize,bool bAsync)
{
	if(!m_bReviceData)
	{
		if(bAsync)
		{
			//异步发送
			m_sockUdp->async_send_to(boost::asio::buffer(pBuffer,nBufferSize),m_endpointServer,boost::bind(&UdpLinkClient::handleSendData,this,pBuffer,nBufferSize,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
		}
		else
		{
			//同步发送
			std::size_t nSize = m_sockUdp->send_to(boost::asio::buffer(pBuffer,nBufferSize),m_endpointServer);
			if(nSize > 0)
			{
				AutoRecvData();
			}
		}
	}
	else
	{
		//更新了对方端点信息
		if(bAsync)
		{
			//异步发送
			m_sockUdp->async_send_to(boost::asio::buffer(pBuffer,nBufferSize),m_endpointRemote,boost::bind(&UdpLinkClient::handleSendData,this,pBuffer,nBufferSize,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
		}
		else
		{
			//同步发送
			std::size_t nSize = m_sockUdp->send_to(boost::asio::buffer(pBuffer,nBufferSize),m_endpointRemote);
			if(nSize > 0)
			{
				AutoRecvData();
			}
		}
	}
 
	return 0;
}
 
//启用接收数据服务(自动)
int UdpLinkClient::AutoRecvData()
{
	if(!m_bReviceServerUse)
	{
		RecvDataProcess();
		m_bReviceServerUse = true;
	}
 
	return 0;
}
 
//启用接收数据服务(人工)
int UdpLinkClient::MmanualRecvData()
{
	RecvDataProcess();
 
	return 0;
}
 
//接收数据
void UdpLinkClient::RecvDataProcess()
{
	//异步接收数据
	m_sockUdp->async_receive_from(boost::asio::buffer(m_recvBuf),m_endpointRemote,boost::bind(&UdpLinkClient::handleRecvData,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
}
 
//当收到客户端数据时，就进入本函数响应处理  
void UdpLinkClient::handleRecvData(const boost::system::error_code& error,std::size_t bytes_transferred)
{
	if(IsStop())
		return;
 
	//如果没有出错
	if(!error || error==boost::asio::error::message_size)
	{
		if(!m_bReviceData)
		{
			m_bReviceData = true;
		}
 
		if(bytes_transferred > UDP_DATA_PACKAGE_MAX_LENGTH)
		{
			//超过最大数据长度
			return;
		}
 
		//打印接收的内容
		char szTmp[UDP_DATA_PACKAGE_MAX_LENGTH] = {0};
		memcpy(szTmp,m_recvBuf.data(),bytes_transferred);
		printf("%s\n",szTmp);
 
		//boost::shared_ptr<std::string> strMessage(new std::string("aaaaaa"));
		std::string strMessage = "bbbbbbbbb";
		//SendData((char *)strMessage.data(),strMessage.size());
		SendDataEx(m_endpointRemote,(char *)strMessage.data(),strMessage.size(),SendDataCallbackOuter,(long)this,0);
 
		//下一次接收
		RecvDataProcess();
	}
}
 
//当发送数据给客户端成功之后响应。  
void UdpLinkClient::handleSendData(char *pBuffer,int nBufferSize,const boost::system::error_code& error,std::size_t bytes_transferred)
{
	if(error != NULL)
	{
		//打印错误信息
		printf("%s", boost::system::system_error(error).what());
		return;
	}
	AutoRecvData();
 
	int n = 0;
}
void UdpLinkClient::handleSendDataInner(SendDataCallback pfunc,DWORD dwUserData1,DWORD dwUserData2,const boost::system::error_code& error,std::size_t bytes_transferred)
{
	if(error != NULL)
	{
		//打印错误信息
		printf("%s", boost::system::system_error(error).what());
		return;
	}
	if(pfunc != NULL)
	{
		pfunc(error,bytes_transferred,dwUserData1,dwUserData2);
	}
	int n = 0;
}
/*
void UdpLinkClient::handleSendData(boost::shared_ptr<std::string> strMessage,const boost::system::error_code& error,std::size_t bytes_transferred)
{
	 int n = 0;
}
*/
 
 void UdpLinkClient::writeBuffer(Buffer &data)
{
    boost::mutex::scoped_lock lock(write_mutex_);

    write_buffer_.push(data);
    
    start_a_write();
}


 void UdpLinkClient::start_a_write()
 {
    boost::mutex::scoped_lock lock(port_mutex_);

    // typedef std::vector<char> Buffer;
     if (!write_buffer_.empty())
    {
		
        char *str = new char[write_buffer_.front().size()+1];
        copy(write_buffer_.front().begin(),write_buffer_.front().end(), str);
        str[write_buffer_.front().size()]=0;
  
        SendData(str,write_buffer_.front().size(),true);			
        write_buffer_.pop();
		delete []str;

    }

 };
 
 void UdpLinkClient::handle_write(const boost::system::error_code& error, size_t bytes_transferred)
 {
	if (error)
    {
        std::cerr << "Transport Serial write Error "<< std::endl;
        return;
    }

    boost::mutex::scoped_lock lock(write_mutex_);

    if (!write_buffer_.empty())	start_a_write();
}
