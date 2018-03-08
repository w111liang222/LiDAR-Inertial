#include "velodyneHDL32E.h"

void velodyneHDL32ESave(velodyneHDL32E * dataPtr)
{
	int ret = dataPtr->saveDataToFile(dataPtr->dataFileIndex);
	if (0 == ret)	dataPtr->dataFileIndex++;
}
void velodyneHDL32ERun(velodyneHDL32E * dataPtr)
{
	unique_ptr<boost::thread>		velodyneHDL32ESaveThread(nullptr);
	char buf[1206] = "";
	dataPtr->velodyneHDL32EUDPServer.reset(new UDPServer(2368));
	while (1)
	{
		if (dataPtr->threadStopFlag)	break;

		dataPtr->velodyneHDL32EUDPServer->UDPServerReceive(buf, 1206);
		int ret = dataPtr->packageParse(buf);
		
		if ((sensorDevice::saveType::save == dataPtr->_save) && (1 == ret))
		{
			if (nullptr != velodyneHDL32ESaveThread)
			{
				velodyneHDL32ESaveThread->join();
			}

			velodyneHDL32ESaveThread.reset(new boost::thread(velodyneHDL32ESave, dataPtr));
			if (nullptr == velodyneHDL32ESaveThread)
			{
				cout << "Open velodyneHDL32E Saving thread failed" << endl;
			}
		}
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}
}

velodyneHDL32E::velodyneHDL32E(modeType modeIn, saveType saveIn) : LiDAR(modeIn, saveIn), sensorDevice(modeIn, saveIn),
	velodyneHDL32ERunThread(nullptr), velodyneHDL32EUDPServer(nullptr),
	threadStopFlag(false)
{
	deviceHz = 100;
	tempData.resize(0);
	tempData.reserve(dataSizeMax);

	xmlCorrection();

	if (online == modeIn)
	{

		velodyneHDL32ERunThread.reset(new boost::thread(velodyneHDL32ERun, this)); //run oxts2000 receiver thread

		if (nullptr == velodyneHDL32ERunThread)
		{
			cout << "Open velodyneHDL32E thread failed" << endl;
		}

	}
}

velodyneHDL32E::~velodyneHDL32E()
{

}

/** \brief set LiDAR data
* \param[in] data
*/
int velodyneHDL32E::setData(vector<pointType>& dataIn)
{
	velodyneHDL32EMutex.lock();

	data = dataIn;

	velodyneHDL32EMutex.unlock();
	return 0;
}

/** \brief get LiDAR data
* \param[in] fileIndex (only active in offline), -1 ---> read next file data
* \param[out] data
*/
int velodyneHDL32E::getData(vector<pointType>& dataIn, int fileIndex)
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

/** \brief get device runing frequence
* \param[out] get device hz
*/
int velodyneHDL32E::getDeviceHz(void)
{
	return deviceHz;
}

/** \brief set device runing frequence
* \param[in] set device hz
*/
int velodyneHDL32E::setDeviceHz(int hzIn)
{
	return -1;
}

/** \brief set laser angle
*/
void velodyneHDL32E::xmlCorrection(void)
{
	xmlData[0] = -30.67f;
	xmlData[1] = -9.33f;
	xmlData[2] = -29.33f;
	xmlData[3] = -8.0f;
	xmlData[4] = -28.0f;
	xmlData[5] = -6.67f;
	xmlData[6] = -26.67f;
	xmlData[7] = -5.33f;
	xmlData[8] = -25.33f;
	xmlData[9] = -4.0f;
	xmlData[10] = -24.0f;
	xmlData[11] = -2.67f;
	xmlData[12] = -22.67f;
	xmlData[13] = -1.33f;
	xmlData[14] = -21.33f;
	xmlData[15] = 0.0f;
	xmlData[16] = -20.0f;
	xmlData[17] = 1.33f;
	xmlData[18] = -18.67f;
	xmlData[19] = 2.67f;
	xmlData[20] = -17.33f;
	xmlData[21] = 4.0f;
	xmlData[22] = -16.0f;
	xmlData[23] = 5.33f;
	xmlData[24] = -14.67f;
	xmlData[25] = 6.67f;
	xmlData[26] = -13.33f;
	xmlData[27] = 8.0f;
	xmlData[28] = -12.0f;
	xmlData[29] = 9.33f;
	xmlData[30] = -10.67f;
	xmlData[31] = 10.67f;
}

/** \brief parse velodyne hdl 32E buffer
* \param[in] buffer
*/
int velodyneHDL32E::packageParse(char buf[1206])
{
	int ret = 0;
	unsigned char  highBit = 0, lowBit = 0;
	static float   rotAngleOld = 359.0;
	static float   rotAngle = 0;

	pointType tempPoint;

	for (int i = 0; i < 12; i++)
	{
		int index1 = 100 * i;
		highBit = buf[index1 + 2];//Rotational Angle
		lowBit = buf[index1 + 3];
		rotAngle = (unsigned short int)((lowBit << 8) + highBit) / 100.0f;
		if (rotAngle < 0) rotAngle = rotAngle + 360;
		else if (rotAngle >= 360) rotAngle = rotAngle - 360;
		//----------check the start of frame ----------//
		if ((rotAngleOld - rotAngle) > 0.1)
		{
			velodyneHDL32EMutex.lock();
			
			data = tempData;
			dataValid = 1;

			velodyneHDL32EMutex.unlock();
			tempData.clear();
			ret = 1;
		}
		rotAngleOld = rotAngle;
		//////////////////////////
		for (int j = 0; j < 32; j++)
		{
			int index2 = 3 * j + index1 + 4;
			highBit = buf[index2];
			lowBit = buf[index2 + 1];//distance 
			float distance = (unsigned short int)((lowBit << 8) + highBit)*0.2f / 100.0f;
			float reflect = (unsigned char)buf[index2 + 2];
			float verticleAngle = xmlData[j];

			if (distance > 1.0 && tempData.size() < dataSizeMax)
			{
				
				tempPoint.x = float(distance*cos(verticleAngle*Ang2Rad)*sin(rotAngle*Ang2Rad));
				tempPoint.y = float(distance*cos(verticleAngle*Ang2Rad)*cos(rotAngle*Ang2Rad));
				tempPoint.z = float(distance*sin(verticleAngle*Ang2Rad));
				tempPoint.r = reflect;
				tempPoint.id = j;
				tempData.push_back(tempPoint);

			}
		}

	}

	return ret;
}

int velodyneHDL32E::readDataFromFile(int fileIndex)
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

	char fileIndexString[10] = "";
	sprintf(fileIndexString, "%06d", fileIndex);
	string fileName = OfflineFilePrefix + fileIndexString + OfflineFileSuffix;

	FILE* veloFp = nullptr;
	veloFp = fopen(fileName.c_str(), "rb");
	if (nullptr == veloFp)
	{
		std::cout << "Can not open :" << fileName.c_str() << std::endl;
		return -1;
	}

	unique_ptr<float[]>tempArrayPtr(new float[5 * dataSizeMax]);
	int arraySize = fread(tempArrayPtr.get(), sizeof(float), 5 * dataSizeMax, veloFp);

	fclose(veloFp);
	if (0 == arraySize)
	{
		cout << "File read error :" << fileName.c_str() << endl;
		return -1;
	}

	data.clear();

	pointType tempPoint;
	for (int i = 0; i < arraySize; i = i + 5)
	{
		tempPoint.x = tempArrayPtr[i];
		tempPoint.y = tempArrayPtr[i + 1];
		tempPoint.z = tempArrayPtr[i + 2];
		tempPoint.r = tempArrayPtr[i + 3];
		tempPoint.id = int(tempArrayPtr[i + 4]);
		data.push_back(tempPoint);
	}

	return 0;
}

int velodyneHDL32E::readDataFromDevice(vector<pointType>& dataIn)
{
	if (offline == _mode)
	{
		return -1;
	}
	else
	{
		bool whileFlag = true;
		while (whileFlag)
		{
			velodyneHDL32EMutex.lock();
			if (0 == dataValid)
			{
				velodyneHDL32EMutex.unlock();
				boost::this_thread::sleep(boost::posix_time::milliseconds(1));
			}
			else
			{
				dataValid = 0;
				dataIn = data;
				velodyneHDL32EMutex.unlock();
				whileFlag = false;
			}
		}
	}
	return 0;
}

int velodyneHDL32E::saveDataToFile(int fileIndex)
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

	char fileIndexString[10] = "";
	sprintf(fileIndexString, "%06d", fileIndex);
	string fileName = OnlineFilePrefix + fileIndexString + OnlineFileSuffix;

	FILE *veloFp = nullptr;
	veloFp = fopen(fileName.c_str(), "wb");
	if (nullptr == veloFp)
	{
		cout << "Can not open file :" << fileName.c_str() << endl;
		return -1;
	}

	unique_ptr<float[]>tempArrayPtr(new float[5 * dataSizeMax]);
	int   tempArrayLen = 0;

	for (auto it = data.begin(); it != data.end(); it++)
	{
		tempArrayPtr[tempArrayLen] = it->x;
		tempArrayPtr[tempArrayLen + 1] = it->y;
		tempArrayPtr[tempArrayLen + 2] = it->z;
		tempArrayPtr[tempArrayLen + 3] = it->r;
		tempArrayPtr[tempArrayLen + 4] = float(it->id);
		tempArrayLen = tempArrayLen + 5;
	}

	fwrite(tempArrayPtr.get(), sizeof(float), tempArrayLen, veloFp);
	fclose(veloFp);

	return 0;
}
