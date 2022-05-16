///@file N3_proxy.h
///@author Yongsheng Yang (vinsonsheep2020@163.com)
///@brief 
///@version 1.0
///@date 2022-05-13
///
///@copyright Copyright (c) 2022
///

#ifndef N3_PROXY_H_
#define N3_PROXY_H_

#include "ros/ros.h"
#include "radio_proxy/uav_proxy.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"


namespace radio_proxy{


class N3_proxy: public uav_proxy{

protected:

    // subscriber
    ros::Subscriber accSub;
	ros::Subscriber angVelSub;
	ros::Subscriber altSub;
	ros::Subscriber batStaSub;
	ros::Subscriber rcStaSub;
	ros::Subscriber disMoSub;
	ros::Subscriber fliStaSub;
	ros::Subscriber gpsHeaSub;
	ros::Subscriber gpsPosSub;
	ros::Subscriber velSub;
	ros::Subscriber heightSub;
	ros::Subscriber localPosSub;
	ros::Subscriber gimbalAngSub;

    // callback function
    void accCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p);
	void angVelCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p);
	void altCB(const geometry_msgs::QuaternionStamped::ConstPtr &msg_p);
	void batStaCB(const sensor_msgs::BatteryState::ConstPtr &msg_p);
	void rcStaCB(const std_msgs::UInt8::ConstPtr &msg_p);
	void disMoCB(const std_msgs::UInt8::ConstPtr &msg_p);
	void fliStaCB(const std_msgs::UInt8::ConstPtr &msg_p);
	void gpsHeaCB(const std_msgs::UInt8::ConstPtr &msg_p);
	void gpsPosCB(const sensor_msgs::NavSatFix::ConstPtr &msg_p);
	void velCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p);
	void heightCB(const std_msgs::Float32::ConstPtr &msg_p);
	void localPosCB(const geometry_msgs::PointStamped::ConstPtr &msg_p);
	void gimbalAngCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p);


public:
    N3_proxy();
    ~N3_proxy();
};

}


#endif