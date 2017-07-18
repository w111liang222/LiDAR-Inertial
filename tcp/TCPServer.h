//============================================================================================================
//!
//! \file TCPServer.h
//!
//! \brief TCPServer operation file.
//!
//============================================================================================================
#ifndef __TCPSERVER_H   
#define __TCPSERVER_H

#include <iostream>
#include <memory>

#include <boost/asio.hpp>

class TCPServer
{
public:
	/** \brief Constructor
	* \param[in] TCP listen port
	*/
	TCPServer(int port);
	~TCPServer();

	void waitForConnect(void);

	/** \brief UDP server receive
	* \param[in] receive buffer
	* \param[in] buffer length
	*/
	int TCPServerReceive(char buf[], int length);



	int TCPServerSend(char buf[], int length);

	/** \brief get sender's address
	* \return sender's address (std::string)
	*/
	std::string getClientAddr(void);

private:
	boost::asio::io_service io_service;
	std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor;
	std::unique_ptr<boost::asio::ip::tcp::socket> socket;
};

#endif //__UDPSERVER_H