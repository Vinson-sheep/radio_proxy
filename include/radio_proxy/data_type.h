///@file data_type.h
///@author Vinson Sheep (775014077@qq.com)
///@brief 
///@version 1.0
///@date 2021-10-11
///
///@copyright Copyright (c) 2021
///

#ifndef DATA_TPYE_H_
#define DATA_TYPE_H_

#include "ros/ros.h"

namespace px4{

///@brief flight data
///
typedef struct MSG_1{
    float_t x;
    float_t y;
    float_t z;
    float_t vx;
    float_t vy;
    float_t vz;
    float_t ax;
    float_t ay;
    float_t az;
    float_t pitch;
    float_t roll;
    float_t yaw;
    float_t yaw_rate;

    MSG_1(){
        this->x = this->y = this->z = 0;
        this->vx = this->vy = this->vz = 0;
        this->ax = this->ay = this->az = 0;
        this->pitch = this->roll = this->yaw = this->yaw_rate = 0;
    }
} MSG_1;

///@brief uav status
///
typedef struct MSG_2{
    float_t battery_v;
    uint8_t connected;
    uint8_t armed;
    uint8_t manual_input;
    char mode[16];

    MSG_2(){
        this->battery_v = 0;
        this->connected = this->armed = this->manual_input = 0;
        this->mode[0] = '\0';
    }
} MSG_2;

///@brief setpoint GPS (保留)
///
typedef struct MSG_101{
    float_t latitude;
    float_t longitude;
    float_t altitude;
    float_t yaw;
} MSG_101;

///@brief setpoint local
///
typedef struct MSG_102{
    float_t x;
    float_t y;
    float_t z;
    float_t yaw;
} MSG_102;

///@brief auto take-off
///
typedef struct MSG_103{

} MSG_103;

///@brief auto land
///
typedef struct MSG_104{

} MSG_104;

///@brief arm
///
typedef struct MSG_105{

} MSG_105;

///@brief disarm
///
typedef struct MSG_106{

} MSG_106;

///@brief hold
///
typedef struct MSG_107{

} MSG_107;

///@brief message
///
typedef struct MSG_255{
    char data[31];
} MSG_255;


}

#endif