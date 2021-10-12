///@file dji_proxy.h
///@author Vinson Sheep (775014077@qq.com)
///@brief 
///@version 1.0
///@date 2021-10-11
///
///@copyright Copyright (c) 2021
///
#ifndef DJI_PROXY_
#define DJI_PROXY

#include "ros/ros.h"
#include "serial/serial.h"
#include <iostream>

#include "std_msgs/String.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_datatypes.h"

#include "radio_proxy/data_type.h"

#include "radio_proxy/FlightData.h"
#include "radio_proxy/Command.h"

namespace dji{


class dji_proxy
{
private:
	// serial port
	serial::Serial _sp;

	// subscriber
	ros::Subscriber _accSub;
	ros::Subscriber _angVelSub;
	ros::Subscriber _altSub;
	ros::Subscriber _batStaSub;
	ros::Subscriber _disMoSub;
	ros::Subscriber _fliStaSub;
	ros::Subscriber _gpsHeaSub;
	ros::Subscriber _gpsPosSub;
	ros::Subscriber _velSub;
	ros::Subscriber _heightSub;
	ros::Subscriber _localPosSub;

	ros::Subscriber _msgSub;

	// variable
	MSG_1 _msg_1;
	MSG_2 _msg_2;

	// publisher
	ros::Publisher _leaderGpsPub;
	ros::Publisher _cmdPub;
	
	// constant
	int _id;
	std::string _port;
	int _baud_rate;

	// buffer
	uint8_t _buffer_write[1024];
	// double buffer design
	uint8_t _buffer_read[2][1024];
	bool _buffer_empty[2];
	bool _buffer_available[2];
	size_t _buffer_len[2];
	size_t _buffer_index;

	bool _flag; // true if serial data is comming

	// loop
	ros::Timer _serialLoopTimer;
	ros::Timer _mainLoopTimer;
	ros::Timer _resendFlightDataTimer;
	ros::Timer _resendStatusTimer;

	// function
	void serialLoopCB(const ros::TimerEvent &event);
	void mainLoopCB(const ros::TimerEvent &event);
	void resentFlightDataCB(const ros::TimerEvent &event);
	void resentStatusCB(const ros::TimerEvent &event);

	void accCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p);
	void angVelCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p);
	void altCB(const geometry_msgs::QuaternionStamped::ConstPtr &msg_p);
	void batStaSub(const sensor_msgs::BatteryState::ConstPtr &msg_p);
	void disMoCB(const std_msgs::UInt8::ConstPtr &msg_p);
	void fliStaCB(const std_msgs::UInt8::ConstPtr &msg_p);
	void gpsHeaCB(const std_msgs::UInt8::ConstPtr &msg_p);
	void gpsPosCB(const sensor_msgs::NavSatFix::ConstPtr &msg_p);
	void velCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p);
	void heightCB(const std_msgs::Float32::ConstPtr &msg_p);
	void localPosCB(const geometry_msgs::PointStamped::ConstPtr &msg_p);

	void msgCB(const std_msgs::String::ConstPtr &msg_p);

public:
	dji_proxy();
	~dji_proxy();
};


}


#endif