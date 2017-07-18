#include "TCPClient.h"

TCPClient::TCPClient(std::string addr, int port)
{
	socket.reset(new boost::asio::ip::tcp::socket(io_service));
	boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string(addr), port);

	boost::system::error_code ec;
	socket->connect(endpoint, ec);
	if (ec)
	{
		std::cout << boost::system::system_error(ec).what() << std::endl;
		getchar();
		exit(0);
	}
}

TCPClient::~TCPClient()
{
}

int TCPClient::TCPClientReceive(char buf[], int length)
{
	boost::system::error_code ec;
	socket->read_some(boost::asio::buffer(buf, length), ec);
	if (ec)
	{
		std::cout << boost::system::system_error(ec).what() << std::endl;
		return -1;
	}
	return 0;
}

int TCPClient::TCPClientSend(char buf[], int length)
{
	boost::system::error_code ec;
	socket->write_some(boost::asio::buffer(buf, length), ec);
	if (ec)
	{
		std::cout << boost::system::system_error(ec).what() << std::endl;
		return -1;
	}
	return 0;
}
