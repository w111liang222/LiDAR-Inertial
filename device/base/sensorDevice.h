//============================================================================================================
//!
//! \file sensorDevice.h
//!
//! \brief virtual definition of sensorDevice.
//!
//============================================================================================================
#ifndef __SENSORDEVICE_H
#define __SENSORDEVICE_H

#include <iostream>
#include <string>

using namespace std;

class sensorDevice
{
public:
	enum modeType { offline = 1, online };// sensor device mode
	enum saveType { unsave = 1, save };   // whether to save sensor data

protected:
	enum modeType _mode;				// device mode
	enum saveType _save;				// save variable
	int deviceHz;

	string OfflineFilePrefix;
	string OfflineFileSuffix;
	string OnlineFilePrefix;
	string OnlineFileSuffix;

	int    dataFileIndex;
	unsigned char dataValid;

public:
	/** \brief Constructor
	* \param[in] set device mode
	* \param[in] set save mode
	*/
	sensorDevice(enum modeType modeIn = online, enum saveType saveIn = unsave);
	virtual ~sensorDevice();

	/** \brief set file names (online/offline)
	* \param[in] file prefix name
	* \param[in] file suffix name
	*/
	void setFileName(string filePrefix, string fileSuffix);

	/** \brief get file names (online/offline)
	* \param[out] file prefix name
	* \param[out] file suffix name
	*/
	void getFileName(string &filePrefix, string &fileSuffix);

	/** \brief set file index (online/offline)
	* \param[in] file index
	*/
	void setFileIndex(int fileIndex);

	/** \brief set file index (online/offline)
	* \param[out] file index
	*/
	void getFileIndex(int &fileIndex);

	/** \brief set device runing frequence
	* \param[in] set device hz
	*/
	virtual int setDeviceHz(int hzIn) = 0;

	/** \brief get device runing frequence
	* \param[out] get device hz
	*/
	virtual int getDeviceHz(void) = 0;

protected:
	virtual int readDataFromFile(int fileIndex) = 0;
	virtual int saveDataToFile(int fileIndex) = 0;
};



#endif