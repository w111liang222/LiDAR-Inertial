#include "LiDAR.h"

/** \brief Constructor
* \param[in] set device mode
* \param[in] set save mode
*/
LiDAR::LiDAR(modeType modeIn, saveType saveIn) : sensorDevice(modeIn,saveIn)
{
	data.resize(0);
	data.reserve(70000);
}

LiDAR::~LiDAR()
{

}
