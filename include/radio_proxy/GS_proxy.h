///@file GS_proxy.h
///@author Yongsheng Yang (vinsonsheep2020@163.com)
///@brief 
///@version 1.0
///@date 2022-05-14
///
///@copyright Copyright (c) 2022
///

#ifndef GS_PROXY_H_
#define GS_PROXY_H_

#include "ros/ros.h"
#include "radio_proxy/base_proxy.h"

namespace radio_proxy{


class GS_proxy: public base_proxy{
private:

	// subscriber
    ros::Subscriber cmdSub;

	// publisher
    ros::Publisher msgPub;

	// callback function	
    void cmdCB(const radio_proxy::Command::ConstPtr &msg_p);

	// rewrite function	
	void MSGCB(const MSG_255& msg, uint8_t o_target_id, uint8_t o_local_id);

public:
	GS_proxy();
	~GS_proxy();
};


}


#endif