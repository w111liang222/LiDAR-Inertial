#include "TCPServer.h"

TCPServer::TCPServer(int port)
{
	acceptor.reset(new boost::asio::ip::tcp::acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)));
}

TCPServer::~TCPServer()
{

}

void TCPServer::waitForConnect(void)
{
	socket.reset(new boost::asio::ip::tcp::socket(io_service));
	acceptor->accept(*socket);
}

int TCPServer::TCPServerReceive(char buf[], int length)
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

int TCPServer::TCPServerSend(char buf[], int length)
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




std::string TCPServer::getClientAddr(void)
{
	return socket->remote_endpoint().address().to_string();
}
