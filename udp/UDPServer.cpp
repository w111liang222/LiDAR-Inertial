#include "UDPServer.h"

/** \brief Constructor
* \param[in] udp receive port
*/
UDPServer::UDPServer(int port)
{
	socket = nullptr;

	namespace ip = boost::asio::ip;
	socket.reset(new ip::udp::socket(io_service, ip::udp::endpoint(ip::udp::v4(), port)));

}
UDPServer::~UDPServer()
{

}

/** \brief UDP server receive
* \param[in] receive buffer
* \param[in] buffer length
*/
int UDPServer::UDPServerReceive(char buf[], int length)
{
	return socket->receive_from(boost::asio::buffer(buf, length), sender_endpoint);
}

/** \brief get sender's address
* \return sender's address (std::string)
*/
std::string UDPServer::getSenderAddr(void)
{
	return sender_endpoint.address().to_string();
}

int UDPServer::getSenderPort(void)
{
	return sender_endpoint.port();
}
