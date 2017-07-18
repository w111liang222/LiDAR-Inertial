# Interface for inertial(oxts2000) and lidar(velodyne hdl-32e)


## Features
* TCP operation interface
* UDP operation interface
* Gaussian projection (GPS coordinate system <-> local Cartesian coordinate system)

## Requirements
* C++ compiler (support C++11)
* boost library (version 1.61)


## Getting started

* Download boost library : contact 15lwang@tongji.edu.cn

#### For Visual Studio 2015

* Create New Project
```
 File -> New -> Project -> Win32 -> Win32 console
```

* Configure Project
```
 Right click project name -> Property -> VC++ directory -> Include directory
 *add boost include directory
 *add inertial_lidar directory
 In Lib Directory
 *add boost lib directory
 Linker -> Input -> dependency files
 *add libboost_thread-vc140-mt-gd-1_61.lib（Debug）
 *add libboost_thread-vc140-mt-1_61.lib（Release）
```

* Add source files
```
 inertial_lidar\decoder\NComRxC.cpp
 inertial_lidar\device\oxts2000.cpp
 inertial_lidar\device\velodyneHDL32E.cpp
 inertial_lidar\device\base\GPS_INS_IMU.cpp
 inertial_lidar\device\base\LiDAR.cpp
 inertial_lidar\device\base\sensorDevice.cpp
 inertial_lidar\projection\gaussianProjection.cpp
 inertial_lidar\udp\UDPClient.cpp
 inertial_lidar\udp\UDPServer.cpp
 inertial_lidar\tcp\TCPClient.cpp
 inertial_lidar\tcp\TCPServer.cpp
```


## Usage examples

#### inertial sensor（Please include oxts2000.h）

* create an instance，online model，auto saving to disk，set receiving port: 9399
```
 oxts2000 oxts2000Device(sensorDevice::modeType::online, sensorDevice::saveType::save, 9399);
```

* set disk file's prefix name and suffix name
```
 oxts2000Device.setFileName(filePrefix, fileSuffix);
```

* get current data
```
 oxts2000Device.getData(data);
```
 
#### LiDAR（Please include velodyneHDL32E.h）

* create an instance，online model，auto saving to disk
```
 velodyneHDL32E velodyneDevice(sensorDevice::modeType::online, sensorDevice::saveType::save);
```

* set disk file's prefix name and suffix name
```
 velodyneDevice.setFileName(filePrefix, fileSuffix);
```

* get current data
```
 oxts2000Device.getData(data);
```

## Help and Support
contact: 15lwang@tongji.edu.cn
