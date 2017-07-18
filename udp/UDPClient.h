//============================================================================================================
//!
//! \file UDPClient.h
//!
//! \brief UDPClient operation file.
//!
//============================================================================================================
#ifndef __UDPCLIENT_H   
#define __UDPCLIENT_H

#include <iostream>
#include <memory>

#include <boost/asio.hpp>

class UDPClient
{
public:
	/** \brief Constructor
	* \param[in] udp receiver's port
	*/
	UDPClient(int port);

	/** \brief Constructor
	* \param[in] udp receiver's port
	* \param[in] udp receiver's port
	*/
	UDPClient(std::string addrIn,int port);
	~UDPClient();

	/** \brief UDP sender receive
	* \param[in] sender buffer
	* \param[in] buffer length
	*/
	int UDPClientSendto(char buf[], int length);

private:
	boost::asio::io_service io_service;
	boost::asio::ip::address addr;
	std::unique_ptr<boost::asio::ip::udp::socket> socket;
	std::unique_ptr<boost::asio::ip::udp::endpoint> broadcast_endpoint;
};

#endif //__UDPSERVER_H