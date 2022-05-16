///@file PX4_proxy.h
///@author Yongsheng Yang (vinsonsheep2020@163.com)
///@brief 
///@version 1.0
///@date 2022-05-14
///
///@copyright Copyright (c) 2022
///

#ifndef PX4_PROXY_H_
#define PX4_PROXY_H_

#include "ros/ros.h"
#include "radio_proxy/uav_proxy.h"
#include "sensor_msgs/BatteryState.h"
#include "mavros_msgs/State.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"

namespace radio_proxy{


class PX4_proxy: public uav_proxy{

private:

    // subscriber
	ros::Subscriber batStaSub;
	ros::Subscriber staSub;
	ros::Subscriber gpsPosSub;
	ros::Subscriber velSub;
	ros::Subscriber localPosSub;

    // callback function
	void batStaCB(const sensor_msgs::BatteryState::ConstPtr &msg_p);
	void staCB(const mavros_msgs::State::ConstPtr &msg_p);
	void gpsPosCB(const sensor_msgs::NavSatFix::ConstPtr &msg_p);
	void velCB(const geometry_msgs::TwistStamped::ConstPtr &msg_p);
	void localPosCB(const geometry_msgs::PoseStamped::ConstPtr &msg_p);


public:
    PX4_proxy();
    ~PX4_proxy();
};

}

#endif