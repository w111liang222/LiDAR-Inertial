#include "GPS_INS_IMU.h"

/** \brief Constructor
* \param[in] set device mode
* \param[in] set save mode
*/
GPS_INS_IMU::GPS_INS_IMU(enum modeType modeIn, enum saveType saveIn) : sensorDevice(modeIn, saveIn)
{
	//--------------variable initializer-------------//
	memset(&data, 0, sizeof(data));

}

GPS_INS_IMU::~GPS_INS_IMU()
{

}
