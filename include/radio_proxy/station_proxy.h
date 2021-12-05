///@file station_proxy.h
///@author Vinson Sheep (775014077@qq.com)
///@brief 
///@version 1.0
///@date 2021-10-11
///
///@copyright Copyright (c) 2021
///

#ifndef STATION_PROXY_
#define STATION_PROXY_

#include "ros/ros.h"
#include "serial/serial.h"
#include <iostream>
#include "std_msgs/String.h"
#include "string.h"
#include "radio_proxy/data_type.h"
#include "std_msgs/String.h"
#include "sstream"
#include "radio_proxy/FlightData.h"
#include "radio_proxy/Command.h"
#include "radio_proxy/Status.h"

namespace px4{


class station_proxy
{
private:
	// serial port
	serial::Serial _sp;

	// subscriber
	ros::Subscriber _cmdSub;

	// publisher
	ros::Publisher _statusPub[9];
	ros::Publisher _flightDataPub[9];
	ros::Publisher _msgPub[9];

	// constant
	int _uav_num;
	std::string _port;
	int _baud_rate;

	// buffer
	uint8_t _buffer_write[1024];
	// triple buffer design
	uint8_t _buffer_read[3][1024];
	bool _buffer_empty[3];
	bool _buffer_available[3];
	size_t _buffer_len[3];
	size_t _buffer_index;

	bool _flag; // true if serial data is comming

	// loop
	ros::Timer _serialLoopTimer;
	ros::Timer _mainLoopTimer;

	// function
	void serialLoopCB(const ros::TimerEvent &event);
	void mainLoopCB(const ros::TimerEvent &event);
	void cmdCB(const radio_proxy::Command::ConstPtr &msg_p);
	
public:
	station_proxy();
	~station_proxy();

};


}


#endif