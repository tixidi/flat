#ifndef TRANSPORT_TCP_H_
#define TRANSPORT_TCP_H_


// #include <gohi_hw/transport.h>
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


typedef std::vector<char> Buffer;

using boost::asio::ip::tcp;
using boost::asio::ip::address;
 
class client : public boost::enable_shared_from_this<client>{
public:
	client(boost::asio::io_service &io_service, tcp::endpoint &endpoint);
	void start();   

	std::queue<Buffer> write_buffer_;
	std::queue<Buffer> read_buffer_;

	Buffer m_buf;

    boost::shared_ptr<boost::asio::io_service> ios_;
	bool initializeTcp();
	void mainRun();




	virtual Buffer readBuffer();
	virtual void writeBuffer(Buffer &data);

	
	inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
	{
		return ios_;
	}

private:
	void handle_connect(const boost::system::error_code &error);
	void handle_write(const boost::system::error_code& error, size_t bytes_transferred) ;
	void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
	void start_a_read();
	void start_a_write();




private:
	boost::asio::io_service &io_service_;
	tcp::socket socket_;
	tcp::endpoint &endpoint_;
	char buf[1024];
	boost::asio::streambuf sbuf;
		// for async read
	Buffer temp_read_buf_;

	boost::thread thread_;
	// locks
	boost::mutex port_mutex_;
	boost::mutex write_mutex_;
	boost::mutex read_mutex_;
	
};
 
typedef boost::shared_ptr<client> client_ptr;
 



#endif /* TRANSPORT_TCP_H_ */
