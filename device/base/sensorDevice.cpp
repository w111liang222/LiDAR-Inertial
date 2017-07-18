#include "sensorDevice.h"

/** \brief Constructor
* \param[in] set device mode
* \param[in] set save mode
*/
sensorDevice::sensorDevice(modeType modeIn, saveType saveIn) : 
	_mode(modeIn), _save(saveIn), deviceHz(0),
	OfflineFilePrefix(""), OfflineFileSuffix(""), OnlineFilePrefix(""), OnlineFileSuffix(""),
	dataFileIndex(0), dataValid(0)
{

}

sensorDevice::~sensorDevice()
{

}

/** \brief set file names (online/offline)
* \param[in] file prefix name
* \param[in] file suffix name
*/
void sensorDevice::setFileName(string filePrefix, string fileSuffix)
{
	if (online == _mode)
	{
		OnlineFilePrefix = filePrefix;
		OnlineFileSuffix = fileSuffix;
	}
	else
	{
		OfflineFilePrefix = filePrefix;
		OfflineFileSuffix = fileSuffix;
	}
}

/** \brief get file names (online/offline)
* \param[out] file prefix name
* \param[out] file suffix name
*/
void sensorDevice::getFileName(string & filePrefix, string & fileSuffix)
{
	if (online == _mode)
	{
		filePrefix = OnlineFilePrefix;
		fileSuffix = OnlineFileSuffix;
	}
	else
	{
		filePrefix = OfflineFilePrefix;
		fileSuffix = OfflineFileSuffix;
	}
}

/** \brief set file index (online/offline)
* \param[in] file index
*/
void sensorDevice::setFileIndex(int fileIndex)
{
	dataFileIndex = fileIndex;
}

/** \brief set file index (online/offline)
* \param[out] file index
*/
void sensorDevice::getFileIndex(int & fileIndex)
{
	fileIndex = dataFileIndex;
}
