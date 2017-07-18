//============================================================================================================
//!
//! \file LiDAR.h
//!
//! \brief virtual definition of LiDAR.
//!
//============================================================================================================

#ifndef __LIDAR_H
#define __LIDAR_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include "sensorDevice.h"

using namespace std;

typedef struct
{
	float x;
	float y;
	float z;
	float r;
	int  id;
}pointType;

typedef struct
{
	pointType point;
	float h; //horizon angle
	int age;
}pointAgeType;

typedef struct
{
	double lat;
	double lon;
	float  r;
}LiDARGPSPointType;

class LiDAR : public virtual sensorDevice
{

protected:
	vector<pointType> data;

public:
	/** \brief Constructor
	* \param[in] set device mode
	* \param[in] set save mode
	*/
	LiDAR(enum modeType modeIn = online, enum saveType saveIn = unsave);
	virtual ~LiDAR();

	/** \brief set LiDAR data
	* \param[in] data
	*/
	virtual int setData(vector<pointType> &dataIn) = 0;

	/** \brief get LiDAR data
	* \param[in] fileIndex (only active in offline), -1 ---> read next file data
	* \param[out] data
	*/
	virtual int getData(vector<pointType> &dataIn, int fileIndex = 0) = 0;

protected:
	virtual int readDataFromDevice(vector<pointType> &dataIn) = 0;

};


#endif
