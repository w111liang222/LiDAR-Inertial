#include <oxts2000.h>

// oxts2000 receive and decode thread
void oxts2000Run(oxts2000 * dataPtr)
{
	char buf[512];
	int count = 0;

	dataPtr->oxts2000UDPServer.reset(new UDPServer(3000));//udp server setup, port num 3000
	dataPtr->nrx.reset(NComCreateNComRxC());// decoder setup

	while (1)
	{
		if (dataPtr->threadStopFlag)	break;

		dataPtr->oxts2000UDPServer->UDPServerReceive(buf, 256);
		NComNewChars(dataPtr->nrx.get(), (unsigned char *)buf, 512);

		count++;
		int countmax = int(dataPtr->deviceHz / 10);
		if (count >= countmax)
		{
			count = 0;

			dataPtr->oxts2000Mutex.lock();

			dataPtr->data.mLat = dataPtr->nrx->mLat;
			dataPtr->data.mLon = dataPtr->nrx->mLon;
			dataPtr->data.mAlt = dataPtr->nrx->mAlt;
			dataPtr->data.mRoll = (dataPtr->nrx->mRoll) / 180.0*dataPtr->pi;
			dataPtr->data.mPitch = (dataPtr->nrx->mPitch) / 180.0*dataPtr->pi;
			dataPtr->data.mHeading = (dataPtr->nrx->mHeading) / 180.0*dataPtr->pi;
			dataPtr->data.mVn = dataPtr->nrx->mVn;
			dataPtr->data.mVe = dataPtr->nrx->mVe;
			dataPtr->data.mVf = dataPtr->nrx->mVf;
			dataPtr->data.mVl = dataPtr->nrx->mVl;
			dataPtr->data.mVu = dataPtr->nrx->mVd;
			dataPtr->data.mAx = dataPtr->nrx->mAx;
			dataPtr->data.mAy = dataPtr->nrx->mAy;
			dataPtr->data.mAz = dataPtr->nrx->mAz;
			dataPtr->data.mAf = dataPtr->nrx->mAf;
			dataPtr->data.mAl = dataPtr->nrx->mAl;
			dataPtr->data.mAu = dataPtr->nrx->mAd;
			dataPtr->data.mWx = (dataPtr->nrx->mWx) / 180.0*dataPtr->pi;
			dataPtr->data.mWy = (dataPtr->nrx->mWy) / 180.0*dataPtr->pi;
			dataPtr->data.mWz = (dataPtr->nrx->mWz) / 180.0*dataPtr->pi;
			dataPtr->data.mWf = (dataPtr->nrx->mWf) / 180.0*dataPtr->pi;
			dataPtr->data.mWl = (dataPtr->nrx->mWl) / 180.0*dataPtr->pi;
			dataPtr->data.mWu = (dataPtr->nrx->mWd) / 180.0*dataPtr->pi;
			dataPtr->data.mPos_accuracy = sqrt(dataPtr->nrx->mNorthAcc*dataPtr->nrx->mNorthAcc + dataPtr->nrx->mEastAcc*dataPtr->nrx->mEastAcc);
			dataPtr->data.mVel_accuracy = sqrt(dataPtr->nrx->mVnAcc*dataPtr->nrx->mVnAcc + dataPtr->nrx->mVeAcc*dataPtr->nrx->mVeAcc);
			dataPtr->data.mNavstat = dataPtr->nrx->mInsNavMode;
			dataPtr->data.mNumsats = dataPtr->nrx->mGpsNumObs;
			dataPtr->data.mPosmode = dataPtr->nrx->mGpsPosMode;
			dataPtr->data.mVelmode = dataPtr->nrx->mGpsVelMode;
			dataPtr->data.mOrimode = dataPtr->nrx->mGpsAttMode;

			dataPtr->dataValid = 1;

			if (sensorDevice::saveType::save == dataPtr->_save)
			{
				int ret = dataPtr->saveDataToFile(dataPtr->dataFileIndex);
				if (ret == 0)	dataPtr->dataFileIndex++;
			}

			dataPtr->oxts2000Mutex.unlock();
		}

	}

	NComDestroyNComRxC(dataPtr->nrx.get());
}

// steer angle receive and decode thread
void steerRun(oxts2000 * dataPtr)
{
	while (1)
	{
		if (dataPtr->threadStopFlag)	break;
		char buf[256] = "";

		dataPtr->steerUDPServer->UDPServerReceive(buf, 256);

		dataPtr->oxts2000Mutex.lock();

		dataPtr->data.steerAngle = atof(buf);

		dataPtr->oxts2000Mutex.unlock();
	}
}

/** \brief Constructor
* \param[in] set device mode
* \param[in] set save mode
* \param[in] steer receive port
*/
oxts2000::oxts2000(enum modeType modeIn, enum saveType saveIn, int revPort) : GPS_INS_IMU(modeIn, saveIn), sensorDevice(modeIn, saveIn),
	oxts2000RunThread(nullptr), steerRunThread(nullptr), oxts2000UDPServer(nullptr), steerUDPServer(nullptr), steerUDPClient(nullptr), nrx(nullptr),
	threadStopFlag(false)
{
	deviceHz = 10;

	if (online == modeIn)
	{
		steerUDPServer.reset(new UDPServer(revPort));// steer receive port
		if (nullptr == steerUDPServer)
		{
			cout << "Open steer angle receiver UDP failed" << endl;
		}

		oxts2000RunThread.reset(new boost::thread(oxts2000Run, this)); //run oxts2000 receiver thread
		steerRunThread.reset(new boost::thread(steerRun, this)); //run steer angle receiver thread

		if (nullptr == oxts2000RunThread)
		{
			cout << "Open oxts2000 thread failed" << endl;
		}
		if (nullptr == steerRunThread)
		{
			cout << "Open steer thread failed" << endl;
		}

	}
}

oxts2000::~oxts2000()
{

}

/** \brief set device runing frequence
* \param[in] set device hz
*/
int oxts2000::setDeviceHz(int hzIn)
{
	deviceHz = (int)(hzIn / 10) * 10;
	return 0;
}

/** \brief get device runing frequence
* \return[out] get device hz
*/
int oxts2000::getDeviceHz(void)
{
	return deviceHz;
}

/** \brief set inertial data
* \param[in] data
*/
int oxts2000::setData(InertialType dataIn)
{
	oxts2000Mutex.lock();

	data = dataIn;

	oxts2000Mutex.unlock();
	return 0;
}

/** \brief get inertial data
* \param[in] fileIndex (only active in offline), -1 ---> read next file data
* \param[out] data
*/
int oxts2000::getData(InertialType & dataIn, int fileIndex)
{
	int ret = -1;
	if (offline == _mode)
	{
		if (fileIndex >= 0)	dataFileIndex = fileIndex;
		ret = readDataFromFile(dataFileIndex);
		if (0 == ret)
		{
			dataFileIndex++;
			dataIn = data;
		}
	}
	else
	{
		ret = readDataFromDevice(dataIn);	
	}
	return ret;
}

/** \brief save inertial data to disk file (save mode should be 'unsave')
* \param[in] prefix name
* \param[in] suffix name
* \param[in] file index
*/
int oxts2000::saveData(string filePrefix, string fileSuffix, int fileIndex)
{
	if (save == _save)
	{
		return -1;
	}

	OnlineFilePrefix = filePrefix;
	OfflineFileSuffix = fileSuffix;
	return saveDataToFile(fileIndex);
}

/** \brief initialize steer control(single send)
* \param[in] steer control IP address
* \param[in] steer control port
*/
int oxts2000::steerInitialize(std::string addrIn, int port)
{
	steerUDPClient.reset(new UDPClient(addrIn, port));
	if (nullptr == steerUDPClient)
	{
		cout << "Open steer angle control UDP failed" << endl;
		return -1;
	}
	return 0;
}

/** \brief initialize steer control(broadcast)
* \param[in] steer control port
*/
int oxts2000::steerInitialize(int port)
{
	steerUDPClient.reset(new UDPClient(port));
	if (nullptr == steerUDPClient)
	{
		cout << "Open steer angle control UDP failed" << endl;
		return -1;
	}
	return 0;
}

/** \brief set steer wheel
* \param[in] steer wheel angle  + --->clockwise - --->anticlockwise
*/
int oxts2000::setSteer(double steerAngleIn)
{
	char steerString[20] = "";
	sprintf(steerString, "%.6lf", steerAngleIn);

	return steerUDPClient->UDPClientSendto(steerString, 20);
}

/** \brief get GPS State
* \param[in] position mode code
* \param[out] GPS state string
*/
string oxts2000::getGPSStateString(double mPosmode)
{
	string returnString = ComGpsXModeName[(int)(mPosmode)];
	return returnString;
}

void oxts2000::waitForGPSStable(double posAccuracy)
{
	//--------------------------Waiting for GPS_INS_IMU device------------------------//
	std::cout << "Waiting for oxts2000 device stable" << std::endl;
	InertialType oxts2000Data;
	while (1)
	{
		if (-1 == getData(oxts2000Data))
		{
			std::cout << "     Can not receive GPS_INS_IMU data" << std::endl;
			continue;
		}

		if (oxts2000Data.mPos_accuracy < posAccuracy && oxts2000Data.mPos_accuracy >= 0.00001)	break;

		std::string gpsState = getGPSStateString(oxts2000Data.mPosmode);
		std::cout << "     GPS State : " << gpsState << "  Position Accuracy : " << oxts2000Data.mPos_accuracy << " m" << std::endl;
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	std::cout << "oxts2000 device is working correctly" << std::endl;
}

int oxts2000::readDataFromFile(int fileIndex)
{
	if (_mode == online)
	{
		return -1;
	}

	if (0 == OfflineFilePrefix.size() && 0 == OfflineFileSuffix.size())
	{
		cout << "Please set offline file name!" << endl;
		return -1;
	}

	char indexChar[10] = "";
	sprintf(indexChar, "%06d", fileIndex);
	string fileName = OfflineFilePrefix + indexChar + OfflineFileSuffix;

	ifstream fileStream(fileName);
	if (false == fileStream.is_open())
	{
		cout << "Can not open :" << fileName.c_str() << endl;
		return -1;
	}

	string fileString;
	string gpsStr;
	fileString.reserve(600);
	getline(fileStream, fileString);
	fileStream.close();

	int startIndex = 0;
	double tempGPSData[30] = { 0 };
	for (int i = 0; i < 30; i++)
	{
		for (int j = startIndex; j < fileString.length(); j++)
		{
			if (fileString.at(j) == ' ')
			{
				gpsStr = fileString.substr(startIndex, j - startIndex);
				startIndex = j + 1;
				break;
			}
		}
		tempGPSData[i] = atof(gpsStr.c_str());
	}
	if (tempGPSData[0] < 3 || tempGPSData[0]>54)	return -1;
	if (tempGPSData[1] < 73 || tempGPSData[1]>134)	return -1;

	data.mLat = tempGPSData[0];
	data.mLon = tempGPSData[1];
	data.mAlt = tempGPSData[2];
	data.mRoll = tempGPSData[3];
	data.mPitch = tempGPSData[4];
	data.mHeading = tempGPSData[5];
	data.mVn = tempGPSData[6];
	data.mVe = tempGPSData[7];
	data.mVf = tempGPSData[8];
	data.mVl = tempGPSData[9];
	data.mVu = tempGPSData[10];
	data.mAx = tempGPSData[11];
	data.mAy = tempGPSData[12];
	data.mAz = tempGPSData[13];
	data.mAf = tempGPSData[14];
	data.mAl = tempGPSData[15];
	data.mAu = tempGPSData[16];
	data.mWx = tempGPSData[17];
	data.mWy = tempGPSData[18];
	data.mWz = tempGPSData[19];
	data.mWf = tempGPSData[20];
	data.mWl = tempGPSData[21];
	data.mWu = tempGPSData[22];
	data.mPos_accuracy = tempGPSData[23];
	data.mVel_accuracy = tempGPSData[24];
	data.mNavstat = tempGPSData[25];
	data.mNumsats = tempGPSData[26];
	data.mPosmode = tempGPSData[27];
	data.mVelmode = tempGPSData[28];
	data.mOrimode = tempGPSData[29];

	data.steerAngle = 0;
	return 0;
}

int oxts2000::readDataFromDevice(InertialType &dataIn)
{
	if (offline == _mode)
	{
		return -1;
	}
	else
	{
		oxts2000Mutex.lock();
		if (0 == dataValid)
		{
			oxts2000Mutex.unlock();
			return -1;
		}
		else
		{
			dataIn = data;
			oxts2000Mutex.unlock();
			return 0;
		}
	}
}

int oxts2000::saveDataToFile(int fileIndex)
{
	if (offline == _mode)
	{
		return -1;
	}
	if (0 == OnlineFilePrefix.size() && 0 == OnlineFileSuffix.size())
	{
		//cout << "Please set online file name!" << endl;
		return -1;
	}

	double tempGpsData[31];

	tempGpsData[0] = data.mLat;
	tempGpsData[1] = data.mLon;
	tempGpsData[2] = data.mAlt;
	tempGpsData[3] = data.mRoll;
	tempGpsData[4] = data.mPitch;
	tempGpsData[5] = data.mHeading;
	tempGpsData[6] = data.mVn;
	tempGpsData[7] = data.mVe;
	tempGpsData[8] = data.mVf;
	tempGpsData[9] = data.mVl;
	tempGpsData[10] = data.mVu;
	tempGpsData[11] = data.mAx;
	tempGpsData[12] = data.mAy;
	tempGpsData[13] = data.mAz;
	tempGpsData[14] = data.mAf;
	tempGpsData[15] = data.mAl;
	tempGpsData[16] = data.mAu;
	tempGpsData[17] = data.mWx;
	tempGpsData[18] = data.mWy;
	tempGpsData[19] = data.mWz;
	tempGpsData[20] = data.mWf;
	tempGpsData[21] = data.mWl;
	tempGpsData[22] = data.mWu;
	tempGpsData[23] = data.mPos_accuracy;
	tempGpsData[24] = data.mVel_accuracy;
	tempGpsData[25] = data.mNavstat;
	tempGpsData[26] = data.mNumsats;
	tempGpsData[27] = data.mPosmode;
	tempGpsData[28] = data.mVelmode;
	tempGpsData[29] = data.mOrimode;

	tempGpsData[30] = data.steerAngle;

	char fileIndexString[10] = "";
	sprintf(fileIndexString, "%06d", fileIndex);
	string fileName = OnlineFilePrefix + fileIndexString + OnlineFileSuffix;

	ofstream fileStream(fileName);
	if (false == fileStream.is_open())
	{
		std::cout << "Can not open file :" << fileName.c_str() << std::endl;
		return -1;
	}
	string fileString = "";
	fileString.reserve(600);
	char strGps[200];
	for (int i = 0; i < 31; i++)
	{
		sprintf(strGps, "%.13lf ", tempGpsData[i]);
		fileString = fileString + strGps;
	}
	fileStream << fileString;
	fileStream.close();

	return 0;
}
