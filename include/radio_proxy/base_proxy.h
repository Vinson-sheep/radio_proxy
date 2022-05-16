///@file base_proxy.h
///@author Yongsheng Yang (vinsonsheep2020@163.com)
///@brief 
///@version 1.0
///@date 2022-05-12
///
///@copyright Copyright (c) 2022
///


#ifndef BASE_PROXY_H_
#define BASE_PROXY_H_

#include "ros/ros.h"
#include "radio_proxy/data_type.h"
#include <queue>
#include "serial/serial.h"
#include <string>
#include "std_msgs/String.h"
#include "radio_proxy/Command.h"
#include "radio_proxy/FlightData.h"
#include "radio_proxy/Status.h"
#include "radio_proxy/Target.h"
#include "radio_proxy/String.h"

namespace radio_proxy{

///@brief 基类通信代理
///
class base_proxy{

private:
    // constant
    int baud_rate_m;
    std::string port_m;

    // timer
    ros::Timer serialLoopTimer;
    ros::Timer mainLoopTimer;

    // 回调函数
    void serialLoopCB(const ros::TimerEvent &event); // 检测数据首字符和末字符，获取完整端口数据，并放入缓冲池
    void mainLoopCB(const ros::TimerEvent &event);  // 处理缓冲池数据，根据类型调用回调函数

    virtual void MSGCB(const MSG_1& msg, uint8_t o_target_id, uint8_t o_local_id);
    virtual void MSGCB(const MSG_2& msg, uint8_t o_target_id, uint8_t o_local_id);
    virtual void MSGCB(const MSG_3& msg, uint8_t o_target_id, uint8_t o_local_id);
    virtual void MSGCB(const MSG_101& msg, uint8_t o_target_id, uint8_t o_local_id);
    virtual void MSGCB(const MSG_255& msg, uint8_t o_target_id, uint8_t o_local_id);

protected:

    // constant
    int swarm_num_m;
    int id_m;
    bool broadcast_m;

    const char HEAD1 = 0X5A;
    const char HEAD2 = 0X0B;
    const char END1 = 0X0D;
    const char END2 = 0X0A;

    // variable
    string char_buffer_m;
    uint8_t read_status_m; // 1: 已读HEAD1 2: 已读HEAD2 3: 已读END1 4: 已读END2

    // publisher
    ros::Publisher fdPub;
    ros::Publisher statusPub;
    ros::Publisher tarPub;

    // 信息池
    MSG_1 msg_1_m;
    MSG_2 msg_2_m;

    // 缓冲池
    std::queue<string> buffer_q; 

    // serial port
    serial::Serial sp_m;

    // 工具函数
    void write(const string& payload, uint8_t target_id, uint8_t msg_id); // 自动添加头尾和校验，传入的string只有payload本身
    void write(const string& payload, uint8_t target_id, uint8_t msg_id, uint8_t o_local_id); // 和上面一个函数基本相同，用于转发数据

public:
    base_proxy();
    ~base_proxy();
};


}


#endif