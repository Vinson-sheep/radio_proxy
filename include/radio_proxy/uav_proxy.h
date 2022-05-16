///@file uav_proxy.h
///@author Yongsheng Yang (vinsonsheep2020@163.com)
///@brief 
///@version 1.0
///@date 2022-05-14
///
///@copyright Copyright (c) 2022
///

#ifndef UAV_PROXY_H_
#define UAV_PROXY_H_

#include "radio_proxy/base_proxy.h"

namespace radio_proxy{

class uav_proxy: public base_proxy{
private:

    // timer
    ros::Timer resendFlightDataTimer;
    ros::Timer resendStatusTimer;

    // subscriber
    ros::Subscriber msgSub;
    ros::Subscriber tarSub;

    // publisher
    ros::Publisher cmdPub;

    // callback function
    void resentFlightDataCB(const ros::TimerEvent &event); // 定时发送自身飞行数据
    void resentStatusCB(const ros::TimerEvent &event); // 定时发送自身状态数据

    void msgCB(const radio_proxy::String::ConstPtr &msg_p);
    void tarCB(const radio_proxy::Target::ConstPtr &msg_p);

    // rewrite function
    void MSGCB(const MSG_101& msg, uint8_t o_target_id, uint8_t o_local_id);

protected:
    string prefix_m;

public:
    uav_proxy();
    ~uav_proxy();

};


}


#endif