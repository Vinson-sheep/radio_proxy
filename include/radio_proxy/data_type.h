///@file data_type.h
///@author Yongsheng Yang (vinsonsheep2020@163.com)
///@brief 
///@version 1.0
///@date 2022-05-12
///
///@copyright Copyright (c) 2022
///


#ifndef DATA_TPYE_H_
#define DATA_TYPE_H_

#include <string>
#include <math.h>
using namespace std;

namespace radio_proxy{

///////////////////////////////////////////////////////

// 工具函数

string toStr(const float_t msg); // float_t 转 string
string toStr(const uint8_t msg); // uint8 转 string
float_t stof(const string& msg, int &pos); // string 转float_t
uint8_t stoi(const string& msg, int& pos); // string 转 uint8_t

// 工具结构体
typedef struct vector3f{
    float_t x;
    float_t y;
    float_t z;
} v3f;
typedef v3f v2f;

///////////////////////////////////////////////////////

// 数据基类（任何新增数据类型都应该继承该抽象类）

struct MSG{
public:
    MSG(){};
    ~MSG(){};

    ///@brief 字符串解码 
    virtual bool decode(const string& str)=0;

    ///@brief 字符串编码（统一大端编码） 
    virtual string encode() const=0;

    ///@brief 返回MSG ID
    virtual uint8_t msg_id() const=0;
};

///////////////////////////////////////////////////////

// 数据子类

///@brief flight data
///飞行数据：广播无人机的高频信息
typedef struct MSG_1: public MSG{

    const size_t n = 116;

    float_t latitude;
    float_t longitude;
    float_t altitude;

    float_t latitude_rtk;
    float_t longitude_rtk;
    float_t altitude_rtk;

    v3f position_local;
    v3f position_local_rtk;

    v3f velocity_local;
    v3f velocity_local_rtk;

    v3f accel_local;
    v3f rpy;
    v3f gimbal_angular;

    float_t yaw_rate = 0;
    float_t height_above_takeoff = 0;
    
    MSG_1(){
        latitude = longitude = altitude = 0;
        latitude_rtk = longitude_rtk = altitude_rtk = 0;
        position_local.x = position_local.y = position_local.z = 0;
        position_local_rtk.x = position_local_rtk.y = position_local_rtk.z = 0;
        velocity_local.x = velocity_local.y = velocity_local.z = 0;
        velocity_local_rtk.x = velocity_local_rtk.y = velocity_local_rtk.z = 0;
        accel_local.x = accel_local.y = accel_local.z = 0;
        rpy.x = rpy.y = rpy.z = 0;
        gimbal_angular.x = gimbal_angular.y = gimbal_angular.z = 0;
        yaw_rate = height_above_takeoff = 0;
    }
    ~MSG_1(){}

    bool decode(const string& str){

        if (str.size() != n) return false;

        int pos = 0;

        latitude = stof(str.substr(pos, 4), pos);
        longitude = stof(str.substr(pos, 4), pos);
        altitude = stof(str.substr(pos, 4), pos);

        latitude_rtk = stof(str.substr(pos, 4), pos);
        longitude_rtk = stof(str.substr(pos, 4), pos);
        altitude_rtk = stof(str.substr(pos, 4), pos);
        
        position_local.x = stof(str.substr(pos, 4), pos);
        position_local.y = stof(str.substr(pos, 4), pos);
        position_local.z = stof(str.substr(pos, 4), pos);

        position_local_rtk.x = stof(str.substr(pos, 4), pos);
        position_local_rtk.y = stof(str.substr(pos, 4), pos);
        position_local_rtk.z = stof(str.substr(pos, 4), pos);

        velocity_local.x = stof(str.substr(pos, 4), pos);
        velocity_local.y = stof(str.substr(pos, 4), pos);
        velocity_local.z = stof(str.substr(pos, 4), pos);

        velocity_local_rtk.x = stof(str.substr(pos, 4), pos);
        velocity_local_rtk.y = stof(str.substr(pos, 4), pos);
        velocity_local_rtk.z = stof(str.substr(pos, 4), pos);

        accel_local.x = stof(str.substr(pos, 4), pos);
        accel_local.y = stof(str.substr(pos, 4), pos);
        accel_local.z = stof(str.substr(pos, 4), pos);

        rpy.x = stof(str.substr(pos, 4), pos);
        rpy.y = stof(str.substr(pos, 4), pos);
        rpy.z = stof(str.substr(pos, 4), pos);

        gimbal_angular.x = stof(str.substr(pos, 4), pos);
        gimbal_angular.y = stof(str.substr(pos, 4), pos);
        gimbal_angular.z = stof(str.substr(pos, 4), pos);    

        yaw_rate = stof(str.substr(pos, 4), pos);
        height_above_takeoff = stof(str.substr(pos, 4), pos);

        return true;
    }

    string encode() const{
        
        string res;

        res += toStr(latitude);
        res += toStr(longitude);
        res += toStr(altitude);

        res += toStr(latitude_rtk);
        res += toStr(longitude_rtk);
        res += toStr(altitude_rtk);

        res += toStr(position_local.x);
        res += toStr(position_local.y);
        res += toStr(position_local.z);

        res += toStr(position_local_rtk.x);
        res += toStr(position_local_rtk.y);
        res += toStr(position_local_rtk.z);

        res += toStr(velocity_local.x);
        res += toStr(velocity_local.y);
        res += toStr(velocity_local.z);

        res += toStr(velocity_local_rtk.x);
        res += toStr(velocity_local_rtk.y);
        res += toStr(velocity_local_rtk.z);

        res += toStr(accel_local.x);
        res += toStr(accel_local.y);
        res += toStr(accel_local.z);

        res += toStr(rpy.x);
        res += toStr(rpy.y);
        res += toStr(rpy.z);

        res += toStr(gimbal_angular.x);
        res += toStr(gimbal_angular.y);
        res += toStr(gimbal_angular.z);

        res += toStr(yaw_rate);
        res += toStr(height_above_takeoff);

        return res;
    }

    uint8_t msg_id() const{return 1;}

} MSG_1;


///@brief status
///状态数据：广播无人机的低频信息
typedef struct MSG_2: public MSG{

    const size_t n = 12;

    float_t battery_v = 0;
    uint8_t gps_health = 0;
    uint8_t armed = 0;
    uint8_t flight_state = 0;
    uint8_t rc_connected = 0;
    uint8_t rtk_connected = 0;
    uint8_t mission = 0;
    uint8_t connected = 0;
    uint8_t mode_len = 0;
    string mode;

    MSG_2(){
        battery_v = 0;
        gps_health = 0;
        armed = 0;
        flight_state = 0;
        rc_connected = 0;
        rtk_connected = 0;
        mission = 0;
        connected = 0;
        mode_len = 0;
    }
    ~MSG_2(){}

    bool decode(const string& str){
        
        if (str.size() < n) return false;

        int pos = 0;
        
        battery_v = stof(str.substr(pos, 4), pos);
        gps_health = stoi(str.substr(pos, 1), pos);
        armed = stoi(str.substr(pos, 1), pos);
        flight_state = stoi(str.substr(pos, 1), pos);
        rc_connected = stoi(str.substr(pos, 1), pos);
        rtk_connected = stoi(str.substr(pos, 1), pos);
        mission = stoi(str.substr(pos, 1), pos);
        connected = stoi(str.substr(pos, 1), pos);
        mode_len = stoi(str.substr(pos, 1), pos);
        
        if (str.size() != n+mode_len) return false;

        mode = str.substr(pos, mode_len);

        pos += mode_len;

        return true;
    }

    string encode() const{

        string res;

        res += toStr(battery_v);
        res += toStr(gps_health);
        res += toStr(armed);
        res += toStr(flight_state);
        res += toStr(rc_connected);
        res += toStr(rtk_connected);
        res += toStr(mission);
        res += toStr(connected);
        res += toStr(mode_len);
        res += mode;

        return res;
    }

    uint8_t msg_id() const{return 2;}


} MSG_2;


///@brief tracking_target
///跟踪目标：如果发现了目标则广播位置
typedef struct MSG_3: public MSG{

    const size_t n = 32;

    float_t target_latitude;
    float_t target_longitude;
    float_t target_altitude;

    v3f target_position_body_frame;
    v2f target_position_in_photo;

    MSG_3(){
        target_latitude = target_longitude = target_altitude = 0;
        target_position_body_frame.x = target_position_body_frame.y = target_position_body_frame.z = 0;
        target_position_in_photo.x = target_position_in_photo.y = 0;
    }
    ~MSG_3(){}

    bool decode(const string& str){
        if (str.size() != n) return false;

        int pos = 0;
        
        target_latitude = stof(str.substr(pos, 4), pos);
        target_longitude = stof(str.substr(pos, 4), pos);
        target_altitude = stof(str.substr(pos, 4), pos);

        target_position_body_frame.x = stof(str.substr(pos, 4), pos);
        target_position_body_frame.y = stof(str.substr(pos, 4), pos);
        target_position_body_frame.z = stof(str.substr(pos, 4), pos);

        target_position_in_photo.x = stof(str.substr(pos, 4), pos);
        target_position_in_photo.y = stof(str.substr(pos, 4), pos);

        return true;
    }

    string encode() const{

        string res;

        res += toStr(target_latitude);
        res += toStr(target_longitude);
        res += toStr(target_altitude);

        res += toStr(target_position_body_frame.x);
        res += toStr(target_position_body_frame.y);
        res += toStr(target_position_body_frame.z);

        res += toStr(target_position_in_photo.x);
        res += toStr(target_position_in_photo.y);

        return res;
    }

    uint8_t msg_id() const{return 3;}

} MSG_3;

///////////////////////////////////////////////////////

///@brief setpoint
///控制指令：地面站发送给无人机的控制指令
typedef struct MSG_101{

    const size_t n = 29;

    uint8_t mission;

    float_t target_latitude;
    float_t target_longitude;
    float_t target_altitude;

    v3f target_position;

    float_t yaw;

    MSG_101(){
        mission = 0;
        target_latitude = target_longitude = target_altitude = 0;
        target_position.x = target_position.y = target_position.z = 0;
        yaw = 0;
    }
    ~MSG_101(){}

    bool decode(const string& str){

        if (str.size() != n) return false;

        int pos = 0;
        
        mission = stoi(str.substr(pos, 1), pos);

        target_latitude = stof(str.substr(pos, 4), pos);
        target_longitude = stof(str.substr(pos, 4), pos);
        target_altitude = stof(str.substr(pos, 4), pos);

        target_position.x = stof(str.substr(pos, 4), pos);
        target_position.y = stof(str.substr(pos, 4), pos);
        target_position.z = stof(str.substr(pos, 4), pos);

        yaw = stof(str.substr(pos, 4), pos);

        return true;
    }

    string encode() const{

        string res;

        res += toStr(mission);

        res += toStr(target_latitude);
        res += toStr(target_longitude);
        res += toStr(target_altitude);

        res += toStr(target_position.x);
        res += toStr(target_position.y);
        res += toStr(target_position.z);

        res += toStr(yaw);
        
        return res;
    }

    uint8_t msg_id() const{return 101;}

} MSG_101;


///////////////////////////////////////////////////////

///@brief message
///保留通道：用于消息反馈，通常是无人机发送给地面站
typedef struct MSG_255{

    string data;

    MSG_255(){}
    ~MSG_255(){}

    bool decode(const string& str){

        if (str.empty()) return false; 

        data = str;

        return true;
    }

    string encode() const {
        return data;
    }

    uint8_t msg_id() const{return 255;}

} MSG_255;

///////////////////////////////////////////////////////

// 工具函数区

// 检查大小端
bool isBig(){
    int a = 1;
    int num = (*(char*)&a);//&a 取出a的地址；  (char*)&a 代表a变量地址的第一个字节的地址
    return num==1? true: false;
}


string toStr(const float_t msg){
    string res;
    res.resize(4);
    int flag = isBig();
    char* p = (char*)(&msg);
    for (int i=0; i<4; i++){
        if (flag) res[i] = p[i];
        else res[3-i] = p[i];
    }
    return res;
}

string toStr(const uint8_t msg){
    string res;
    res.resize(1);
    res[0] = msg;
    return res;
}


float_t stof(const string& msg, int &pos){
    float_t res;
    int flag = isBig();
    char* p = (char*)(&res);
    for (int i=0; i<4; i++){
        if (flag) p[i] = msg[i];
        else p[3-i] = msg[i];
    }
    pos += msg.length();
    return res;
}

uint8_t stoi(const string& msg, int& pos){
    pos += 1;
    return (uint8_t)msg[0];
} 

///////////////////////////////////////////////////////

}

#endif