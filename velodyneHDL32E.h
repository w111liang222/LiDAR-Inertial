//============================================================================================================
//!
//! \file velodyneHDL32E.h
//!
//! \brief definition of velodyneHDL32E.
//!
//============================================================================================================

#ifndef __VELODYNEHDL32E_H
#define __VELODYNEHDL32E_H

#include <iostream>
#include <memory>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <device/base/LiDAR.h>
#include <udp/UDPServer.h>

using namespace std;

class velodyneHDL32E : public LiDAR
{
private:
	const double  pi = 3.1415926535897932384626433832795;
	const double  Ang2Rad = 0.01745329251994;
	const int	  dataSizeMax = 70000;

	vector<pointType> tempData;
	float xmlData[64];

private:
	unique_ptr<boost::thread>		velodyneHDL32ERunThread;
	unique_ptr<UDPServer>   		velodyneHDL32EUDPServer;

	boost::mutex					velodyneHDL32EMutex;
	bool							threadStopFlag;


public:
	velodyneHDL32E(enum modeType modeIn = online, enum saveType saveIn = unsave);
	~velodyneHDL32E();

	/** \brief set LiDAR data
	* \param[in] data
	*/
	int setData(vector<pointType> &dataIn);

	/** \brief get LiDAR data
	* \param[in] fileIndex (only active in offline), -1 ---> read next file data
	* \param[out] data
	*/
	int getData(vector<pointType> &dataIn, int fileIndex = 0);

	/** \brief get device runing frequence
	* \param[out] get device hz
	*/
	int getDeviceHz(void);

private:
	/** \brief set device runing frequence
	* \param[in] set device hz
	*/
	int setDeviceHz(int hzIn);

public:

private:
	/** \brief set laser angle
	*/
	void xmlCorrection(void);

	/** \brief parse velodyne hdl 32E buffer
	* \param[in] buffer
	*/
	int packageParse(char buf[1206]);

private:
	friend void velodyneHDL32ERun(velodyneHDL32E *dataPtr);
	friend void velodyneHDL32ESave(velodyneHDL32E *dataPtr);

	int readDataFromFile(int fileIndex);
	int readDataFromDevice(vector<pointType> &dataIn);
	int saveDataToFile(int fileIndex);
};







#endif
