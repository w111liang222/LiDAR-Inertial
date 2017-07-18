#include "UDPClient.h"

/** \brief Constructor
* \param[in] udp receiver's port
*/
UDPClient::UDPClient(int port)
{
	namespace ip = boost::asio::ip;
	socket = nullptr;
	broadcast_endpoint = nullptr;

	socket.reset(new ip::udp::socket(io_service, ip::udp::endpoint(ip::udp::v4(), 0)));
	socket->set_option(boost::asio::socket_base::broadcast(true));

	broadcast_endpoint.reset(new ip::udp::endpoint(ip::address_v4::broadcast(), port));

}

/** \brief Constructor
* \param[in] udp receiver's port
* \param[in] udp receiver's port
*/
UDPClient::UDPClient(std::string addrIn, int port)
{
	namespace ip = boost::asio::ip;
	socket = nullptr;
	broadcast_endpoint = nullptr;

	addr = boost::asio::ip::address::from_string(addrIn.c_str());

	socket.reset(new ip::udp::socket(io_service, ip::udp::endpoint(ip::udp::v4(), 0)));
	broadcast_endpoint.reset(new ip::udp::endpoint(addr, port));
}
UDPClient::~UDPClient()
{

}

/** \brief UDP server receive
* \param[in] receive buffer
* \param[in] buffer length
*/
int UDPClient::UDPClientSendto(char buf[], int length)
{
	return socket->send_to(boost::asio::buffer(buf, length), *broadcast_endpoint);
}