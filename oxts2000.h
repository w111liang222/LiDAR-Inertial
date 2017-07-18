//============================================================================================================
//!
//! \file oxts2000.h
//!
//! \brief definition of oxts2000 device.
//!
//============================================================================================================

#ifndef __OXTS2000_H
#define __OXTS2000_H

#ifdef WIN32

//#ifdef _DEBUG
//#pragma comment(lib,"Inertial2Lib_Debug.lib")
//#else 
//#pragma comment(lib,"Inertial2Lib_Release.lib")
//#endif

#else

//#error Require WIN32 Environment

#endif

#include <iostream>
#include <memory>
#include <fstream>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <decoder/NComRxC.h>
#include <udp/UDPServer.h>
#include <udp/UDPClient.h>

#include <device/base/GPS_INS_IMU.h>

using namespace std;

// GPS Mode string
static const char *ComGpsXModeName[31] =
{
	"None",
	"Search",
	"Doppler",
	"SPS",
	"Differential",
	"RTK Float",
	"RTK Integer",
	"WAAS",
	"Omnistar",
	"Omnistar HP",
	"No Data",
	"Blanked",
	"Doppler (PP)",
	"SPS (PP)",
	"Differential (PP)",
	"RTK Float (PP)",
	"RTK Integer (PP)",
	"Omnistar XP",
	"CDGPS",
	"Not Recognised",
	"gxDoppler",
	"gxSPS",
	"gxDifferential",
	"gxFloat",
	"gxInteger",
	"ixDoppler",
	"ixSPS",
	"ixDifferential",
	"ixFloat",
	"ixInteger",
	"Unknown"
};


class oxts2000 : public GPS_INS_IMU
{
private:
	const double pi = 3.1415926535897932384626433832795;

	std::unique_ptr<boost::thread>	oxts2000RunThread;
	std::unique_ptr<boost::thread>	steerRunThread;
	std::unique_ptr<UDPServer>   	oxts2000UDPServer;
	std::unique_ptr<UDPServer>		steerUDPServer;
	std::unique_ptr<UDPClient>		steerUDPClient;
	std::unique_ptr<NComRxC>	    nrx;
	bool							threadStopFlag;

	boost::mutex					oxts2000Mutex;

public:
	/** \brief Constructor
	* \param[in] set device mode
	* \param[in] set save mode
	* \param[in] steer receive port
	*/
	oxts2000(enum modeType modeIn = online, enum saveType saveIn = unsave, int revPort = 9399);

	~oxts2000();


	/** \brief set device runing frequence
	* \param[in] set device hz
	*/
	int setDeviceHz(int hzIn);

	/** \brief get device runing frequence
	* \return[out] get device hz
	*/
	int getDeviceHz(void);

	/** \brief set inertial data
	* \param[in] data
	*/
	int setData(InertialType dataIn);

	/** \brief get inertial data
	* \param[in] fileIndex (only active in offline), -1 ---> read next file data
	* \param[out] data
	*/
	int getData(InertialType &dataIn, int fileIndex = 0);

	/** \brief save inertial data to disk file (save mode should be 'unsave')
	* \param[in] prefix name
	* \param[in] suffix name
	* \param[in] file index
	*/
	int saveData(string filePrefix, string fileSuffix, int fileIndex);

	/** \brief initialize steer control(single send)
	* \param[in] steer control IP address
	* \param[in] steer control port
	*/
	int steerInitialize(std::string addrIn, int port);

	/** \brief initialize steer control(broadcast)
	* \param[in] steer control port
	*/
	int steerInitialize(int port);

	/** \brief set steer wheel
	* \param[in] steer wheel angle  + --->clockwise - --->anti-clockwise
	*/
	int setSteer(double steerAngleIn);

	/** \brief get GPS State
	* \param[in] position mode code
	* \return[out] GPS state string
	*/
	string getGPSStateString(double mPosmode);

	/** \brief wait for GPS Stable(block)
	* \param[in] position accuracy
	*/
	void waitForGPSStable(double posAccuracy);

private:
	friend void oxts2000Run(oxts2000 *dataPtr);
	friend void steerRun(oxts2000 *dataPtr);

	int readDataFromFile(int fileIndex);
	int readDataFromDevice(InertialType &dataIn);
	int saveDataToFile(int fileIndex);
};

#endif
