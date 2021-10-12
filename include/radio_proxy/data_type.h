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

namespace dji{

///@brief flight data
///
typedef struct MSG_1{
    float_t latitude;
    float_t longitude;
    float_t altitude;
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
    float_t height_above_takeoff;

    MSG_1(){
        this->latitude = this->longitude = this->altitude = 0;
        this->x = this->y = this->z = 0;
        this->vx = this->vy = this->vz = 0;
        this->ax = this->ay = this->az = 0;
        this->pitch = this->roll = this->yaw = this->yaw_rate = 0;
        this->height_above_takeoff = 0;
    }
} MSG_1;

///@brief dji status
///
typedef struct MSG_2{
    float_t battery_v;
    uint8_t display_mode;
    uint8_t flight_status;
    uint8_t gps_health;
    uint8_t arm_state;
    uint8_t land_state;

    MSG_2(){
        this->battery_v = 0;
        this->display_mode = this->flight_status = 0;
        this->gps_health = 0;
        this->arm_state = this->land_state = 0;
    }
} MSG_2;

///@brief setpoint GPS
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