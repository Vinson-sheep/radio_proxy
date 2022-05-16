///@file A3_proxy.h
///@author Yongsheng Yang (vinsonsheep2020@163.com)
///@brief 
///@version 1.0
///@date 2022-05-14
///
///@copyright Copyright (c) 2022
///

#ifndef A3_PROXY_H_
#define A3_PROXY_H_

#include "ros/ros.h"
#include "radio_proxy/N3_proxy.h"

namespace radio_proxy{

class A3_proxy: public N3_proxy{

protected:

    // subscriber
	ros::Subscriber rtkStaSub;
	ros::Subscriber rtkPosSub;
	ros::Subscriber rtkVelSub;

    // callback function
    void rtkStaCB(const std_msgs::UInt8::ConstPtr &msg_p);
    void rtkPosCB(const sensor_msgs::NavSatFix::ConstPtr &msg_p);
    void rtkVelCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p);


public:
    A3_proxy();
    ~A3_proxy();

};

}
#endif