# Matlab interface for inertial(oxts2000) and lidar data file


## Usage examples

#### loadGPSdata.m: 读取保存的oxts2000数据

* 数据格式
```
 oxts{i}(1) -->mLat; 			//!< Latitude. [deg]
 oxts{i}(2) -->mLon; 			//!< Longitude. [deg]
 oxts{i}(3) -->mAlt; 			//!< Altitude. [m]
 oxts{i}(4) -->mRoll;			//!< Roll. [rad]
 oxts{i}(5) -->mPitch;			//!< Pitch. [rad]
 oxts{i}(6) -->mHeading;		//!< Heading. [rad]
 oxts{i}(7) -->mVn;				//!< North velocity. [m s^(-1)]
 oxts{i}(8) -->mVe;				//!< East velocity. [m s^(-1)]
 oxts{i}(9) -->mVf; 			//!< Forward velocity. [m s^(-1)]
 oxts{i}(10)-->mVl;				//!< Lateral velocity. [m s^(-1)]
 oxts{i}(11)-->mVu;				//!< Downward velocity. [m s^(-1)]
 oxts{i}(12)-->mAx;				//!< Acceleration along the X axis. [m s^(-2)]
 oxts{i}(13)-->mAy;				//!< Acceleration along the Y axis. [m s^(-2)]
 oxts{i}(14)-->mAz;				//!< Acceleration along the Z axis. [m s^(-2)]
 oxts{i}(15)-->mAf;				//!< Acceleration forward. [m s^(-2)]
 oxts{i}(16)-->mAl;				//!< Acceleration laterally. [m s^(-2)]
 oxts{i}(17)-->mAu;				//!< Acceleration downward. [m s^(-2)]
 oxts{i}(18)-->mWx;				//!< Angular rate about the X axis. [rad s^(-1)]
 oxts{i}(19)-->mWy;				//!< Angular rate about the Y axis. [rad s^(-1)]
 oxts{i}(20)-->mWz;				//!< Angular rate about the Z axis. [rad s^(-1)]
 oxts{i}(21)-->mWf;				//!< Angular rate about the forward axis. [rad s^(-1)]
 oxts{i}(22)-->mWl;				//!< Angular rate about the lateral axis. [rad s^(-1)]
 oxts{i}(23)-->mWu;				//!< Angular rate about the down axis. [rad s^(-1)]
 oxts{i}(24)-->mPos_accuracy;	//!< velocity accuracy (north/east in m)
 oxts{i}(25)-->mVel_accuracy;	//!< velocity accuracy (north/east in m/s)
 oxts{i}(26)-->mNavstat;		//!< navigation status (see navstat_to_string)
 oxts{i}(27)-->mNumsats;		//!< number of satellites tracked by primary GPS receiver
 oxts{i}(28)-->mPosmode;		//!< position mode of primary GPS receiver(see gps_mode_to_string)
 oxts{i}(29)-->mVelmode;		//!< velocity mode of primary GPS receiver(see gps_mode_to_string)
 oxts{i}(30)-->mOrimode;		//!< orientation mode of primary GPS receiver (see gps_mode_to_string)
 oxts{i}(31)-->steerAngle;		//!< steer angle [deg s^(-1)] (positive ---> clockwise , negative ---> anti-clockwise) 
 其中i表示第i帧数据
```
 
#### loadVelodynedata.m: 读取保存的velodyneHDL32E激光雷达数据

* 数据格式
```
 velo(:,1) --->x				//!< x coordinate. [m]
 velo(:,2) --->y				//!< y coordinate. [m]
 velo(:,3) --->z				//!< z coordinate. [m]
 velo(:,4) --->r				//!< reflect.      
 velo(:,5) --->id				//!< laser ID		
```

## Help and Support
contact: 15lwang@tongji.edu.cn
