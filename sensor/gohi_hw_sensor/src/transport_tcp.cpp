#include <gohi_hw/transport_tcp.h>


client::client(boost::asio::io_service &io_service, tcp::endpoint &endpoint)
		: io_service_(io_service),
         socket_(io_service), 
         endpoint_(endpoint),m_buf(1024.,0)
{
     

}
 

void client::start() 
{
    socket_.async_connect(endpoint_,
        boost::bind(&client::handle_connect,
        shared_from_this(),
        boost::asio::placeholders::error));
}

void client::handle_connect(const boost::system::error_code &error) 
{
		if (error) {
			if (error.value() != boost::system::errc::operation_canceled) {
				std::cerr << boost::system::system_error(error).what() << std::endl;
			}
			socket_.close();
			return;
		}
		static tcp::no_delay option(true);
		socket_.set_option(option); 
     
}




bool client::initializeTcp()
{

    temp_read_buf_.resize(1024, 0);
    try
    {
        thread_ = boost::thread(boost::bind(&client::mainRun, this));
    }
    catch(std::exception &e)
    {
        std::cerr << "Transport Serial thread create failed " << std::endl;
        std::cerr << "Error Info: " << e.what() <<std::endl; 
        return false;
    }

    return true;
}

void client::mainRun()
{
    start_a_read();
    io_service_.run();
}

//------------------------------------------
void client::writeBuffer(Buffer &data)
{
    boost::mutex::scoped_lock lock(write_mutex_);

    write_buffer_.push(data);
    
    start_a_write();
}


 void client::start_a_write()
 {
    boost::mutex::scoped_lock lock(port_mutex_);


     if (!write_buffer_.empty())
    {
        boost::asio::async_write(socket_,
            boost::asio::buffer((write_buffer_.front())),
            boost::bind(&client::handle_write,
            shared_from_this(),
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
        
        write_buffer_.pop();
    }

 };
 
 void client::handle_write(const boost::system::error_code& error, size_t bytes_transferred)
 {
	if (error)
    {
        std::cerr << "Transport Serial write Error "<< std::endl;
        return;
    }

    boost::mutex::scoped_lock lock(write_mutex_);

    if (!write_buffer_.empty())	start_a_write();
}

//------------------------------------------
Buffer client::readBuffer()
{
	boost::mutex::scoped_lock lock(read_mutex_);

	if (!read_buffer_.empty())
	{
		Buffer data(read_buffer_.front());                
		read_buffer_.pop();        
		return data;
	}
	Buffer data;
	return data;
}

void client::start_a_read()
{
    boost::mutex::scoped_lock lock(port_mutex_);

    socket_.async_read_some(boost::asio::buffer(temp_read_buf_),
			boost::bind(&client::handle_read,
			shared_from_this(),
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred)
                 );
}

void client::handle_read(const boost::system::error_code &ec, size_t bytesTransferred)
{
	if (ec)
	{
		std::cerr << "Transport Serial read Error " << std::endl;
		return;
	}
    boost::mutex::scoped_lock lock(read_mutex_);
	Buffer data(temp_read_buf_.begin(), temp_read_buf_.begin() + bytesTransferred);
	read_buffer_.push(data);
	start_a_read();
}



