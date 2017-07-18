//============================================================================================================
//!
//! \file GPS_INS_IMU.h
//!
//! \brief virtual definition of GPS, INS, IMU.
//!
//============================================================================================================

#ifndef __GPS_INS_IMU_H
#define __GPS_INS_IMU_H

#include <iostream>
#include <string.h>

#include <device/base/sensorDevice.h>

using namespace std;

typedef struct
{
	double mLat; //!< Latitude. [deg]
	double mLon; //!< Longitude. [deg]
	double mAlt; //!< Altitude. [m]
	double mRoll;//!< Roll. [rad]
	double mPitch;//!< Pitch. [rad]
	double mHeading;//!< Heading. [rad]
	double mVn;//!< North velocity. [m s^(-1)]
	double mVe;//!< East velocity. [m s^(-1)]
	double mVf; //!< Forward velocity. [m s^(-1)]
	double mVl;//!< Lateral velocity. [m s^(-1)]
	double mVu;//!< Downward velocity. [m s^(-1)]
	double mAx;//!< Acceleration along the X axis. [m s^(-2)]
	double mAy;//!< Acceleration along the Y axis. [m s^(-2)]
	double mAz;//!< Acceleration along the Z axis. [m s^(-2)]
	double mAf;//!< Acceleration forward. [m s^(-2)]
	double mAl;//!< Acceleration laterally. [m s^(-2)]
	double mAu;//!< Acceleration downward. [m s^(-2)]
	double mWx;//!< Angular rate about the X axis. [rad s^(-1)]
	double mWy;//!< Angular rate about the Y axis. [rad s^(-1)]
	double mWz;//!< Angular rate about the Z axis. [rad s^(-1)]
	double mWf;//!< Angular rate about the forward axis. [rad s^(-1)]
	double mWl;//!< Angular rate about the lateral axis. [rad s^(-1)]
	double mWu;//!< Angular rate about the down axis. [rad s^(-1)]
	double mPos_accuracy;//velocity accuracy (north/east in m)
	double mVel_accuracy;//velocity accuracy (north/east in m/s)
	double mNavstat;//navigation status (see navstat_to_string)
	double mNumsats;//number of satellites tracked by primary GPS receiver
	double mPosmode;//position mode of primary GPS receiver(see gps_mode_to_string)
	double mVelmode;//velocity mode of primary GPS receiver(see gps_mode_to_string)
	double mOrimode;//orientation mode of primary GPS receiver (see gps_mode_to_string)

	double steerAngle;// steer angle [deg s^(-1)]
}InertialType;

class GPS_INS_IMU : public virtual sensorDevice
{

protected:
	InertialType data;					// inertial data (device data)

public:
	/** \brief Constructor
	* \param[in] set device mode 
	* \param[in] set save mode
	*/
	GPS_INS_IMU(enum modeType modeIn = online, enum saveType saveIn = unsave);
	virtual ~GPS_INS_IMU();

	/** \brief set inertial data
	* \param[in] data
	*/
	virtual int setData(InertialType dataIn) = 0;

	/** \brief get inertial data
	* \param[in] fileIndex (only active in offline), -1 ---> read next file data
	* \param[out] data
	*/
	virtual int getData(InertialType &dataIn, int fileIndex = 0) = 0;
	
	/** \brief get GPS State
	* \param[in] position mode code
	* \param[out] GPS state string
	*/
	virtual string getGPSStateString(double mPosmode) = 0;

protected:
	virtual int readDataFromDevice(InertialType &dataIn) = 0;
};



#endif