//============================================================================================================
//!
//! \file TCPClient.h
//!
//! \brief TCPClient operation file.
//!
//============================================================================================================
#ifndef __TCPCLIENT_H   
#define __TCPCLIENT_H

#include <iostream>
#include <memory>

#include <boost/asio.hpp>

class TCPClient
{
public:
	/** \brief Constructor
	* \param[in] TCP listen port
	*/
	TCPClient(std::string addr, int port);
	~TCPClient();

	/** \brief UDP server receive
	* \param[in] receive buffer
	* \param[in] buffer length
	*/
	int TCPClientReceive(char buf[], int length);



	int TCPClientSend(char buf[], int length);

private:
	boost::asio::io_service io_service;
	std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor;
	std::unique_ptr<boost::asio::ip::tcp::socket> socket;

	boost::asio::ip::udp::endpoint sender_endpoint;
};

#endif //__UDPSERVER_H