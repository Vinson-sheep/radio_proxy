///@file px4_proxy.h
///@author Vinson Sheep (775014077@qq.com)
///@brief 
///@version 1.0
///@date 2021-12-05
///
///@copyright Copyright (c) 2021
///
#ifndef PX4_PROXY_
#define PX4_PROXY_

#include "ros/ros.h"
#include "serial/serial.h"
#include <iostream>
#include <algorithm>

#include "std_msgs/String.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2/utils.h"

#include "mavros_msgs/State.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/BatteryStatus.h"

#include "radio_proxy/data_type.h"
#include "radio_proxy/FlightData.h"
#include "radio_proxy/Command.h"

namespace px4{


class px4_proxy
{
private:
	// serial port
	serial::Serial _sp;

	// subscriber
	ros::Subscriber _stateSub;
	ros::Subscriber _velSub;
	ros::Subscriber _localPoseSub;
	ros::Subscriber _batStaSub;

	ros::Subscriber _msgSub;

	// variable
	MSG_1 _msg_1;
	MSG_2 _msg_2;

	// publisher
	ros::Publisher _leaderFdPub;
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

    void stateCB(const mavros_msgs::State::ConstPtr &msg_p);
	void velCB(const geometry_msgs::TwistStamped::ConstPtr &msg_p);
	void localPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg_p);
	void batStaSub(const mavros_msgs::BatteryStatus::ConstPtr &msg_p);

	void msgCB(const std_msgs::String::ConstPtr &msg_p);

public:
	px4_proxy();
	~px4_proxy();
};


}


#endif