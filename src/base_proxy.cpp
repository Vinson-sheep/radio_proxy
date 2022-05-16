#include "radio_proxy/base_proxy.h"


namespace radio_proxy{

base_proxy::base_proxy(){

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // get param
    private_nh.param("id", id_m, 1);
    private_nh.param("port", port_m, std::string("/dev/ttyUSB0"));
    private_nh.param("baud_rate", baud_rate_m, 230400);
    private_nh.param("swarm_num", swarm_num_m, 1);
    private_nh.param("broadcast", broadcast_m, false);

    // initialize serial
    try 
    { 
        sp_m.setPort(port_m); 
        sp_m.setBaudrate(baud_rate_m); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        sp_m.setTimeout(to); 
        sp_m.open();
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR("Unable to open port %s.", port_m.c_str()); 
    } 

    if(sp_m.isOpen()) 
    { 
        ROS_INFO("Serial Port %s initialized.", port_m.c_str()); 
    } 
    else
    { 
    } 

    // publisher
    fdPub = private_nh.advertise<radio_proxy::FlightData>("flight_data", 1);
    statusPub = private_nh.advertise<radio_proxy::Status>("flight_status", 1);
    tarPub = private_nh.advertise<radio_proxy::Target>("target_all", 1);

    // timer
    serialLoopTimer = nh.createTimer(ros::Duration(0.01), &base_proxy::serialLoopCB, this);
    mainLoopTimer = nh.createTimer(ros::Duration(0.05), &base_proxy::mainLoopCB, this);

    char_buffer_m.clear();
    read_status_m = 4;
}

base_proxy::~base_proxy(){
    sp_m.close();
    printf("Serial port %s closed!\n", port_m.c_str());
}

void base_proxy::serialLoopCB(const ros::TimerEvent &event){
    // 有限状态自动机，检测帧头帧尾
    if (sp_m.available()){
        string data = sp_m.read(sp_m.available());

        for (auto c: data){

            if (c == HEAD1){
                switch(read_status_m){
                    case 1: // 前一个HEAD1非法
                        break;
                    case 2: // 该HEAD1是数据
                        char_buffer_m += c;
                        break;
                    case 3: // 前一个END1是数据
                        char_buffer_m += END1;
                        char_buffer_m += c;
                        read_status_m = 2;
                        break;
                    case 4: // 正常循环
                        read_status_m = 1;
                        break;
                }
            }
            else if (c == HEAD2){
                switch(read_status_m){
                    case 1: // 正常循环
                        read_status_m = 2;
                        break;
                    case 2: // 该HEAD2是数据
                        char_buffer_m += c;
                        break;
                    case 3: // 前一个END1是数据
                        char_buffer_m += END1;
                        char_buffer_m += c;
                        read_status_m = 2;
                        break;
                    case 4: // 该字符非法
                        break;
                }
            }
            else if (c == END1){
                switch(read_status_m){
                    case 1: // 前一个HEAD1非法，且该字符也非法
                        char_buffer_m.clear();
                        read_status_m = 4;
                        break;
                    case 2: // 正常循环
                        read_status_m = 3;
                        break;
                    case 3: // 前一个END1是数据
                        char_buffer_m += END1;
                        break;
                    case 4: // 该字符非法
                        break;
                }
            }
            else if (c == END2){
                switch(read_status_m){
                    case 1: // 前一个HEAD1非法，该字符非法
                        char_buffer_m.clear();
                        read_status_m = 4;
                        break;
                    case 2: // 该END2是数据
                        char_buffer_m += c;
                        break;
                    case 3: // 正常循环
                        buffer_q.push(char_buffer_m);
                        char_buffer_m.clear();
                        read_status_m = 4;
                        break;
                    case 4: // 该END2非法
                        break;
                }
            }
            else {
                switch(read_status_m){
                    case 1: // 前一个HEAD1非法，该字符非法
                        char_buffer_m.clear();
                        read_status_m = 4;
                        break;
                    case 2: // 该字符是数据
                        char_buffer_m += c;
                        break;
                    case 3: // // 前一个END1是数据，该字符是数据
                        char_buffer_m += END1;
                        char_buffer_m += c;
                        read_status_m = 2;
                        break;
                    case 4: // 该END2非法
                        break;
                }
            }
        }
    }
}

void base_proxy::mainLoopCB(const ros::TimerEvent &event){

    // 处理缓冲池数据，根据类型调用回调函数

    MSG_1 a1;
    MSG_2 a2;
    MSG_3 a3;
    MSG_101 a101;
    MSG_255 a255;

    while (!buffer_q.empty()){
        string data = buffer_q.front();
        buffer_q.pop();

        uint8_t msg_id = data[0];
        uint8_t o_target_id = data[1];
        uint8_t o_local_id = data[2];
        string payload = data.substr(3);

        // 第一层过滤
        if (o_local_id == id_m) { // 如果发送方式自己，说明存在多个broardcaster，需要过滤
            continue;
        }
        // 第二层过滤
        if (o_local_id != 254 && o_local_id != 255 && !(o_local_id >= 1 && o_local_id <= swarm_num_m)){
            ROS_ERROR("Invalid LOCALID = %d.\n", o_local_id);
            continue;
        }
        // 第三层
        if (o_target_id != 254 && o_target_id != 255 && !(o_target_id >= 1 && o_target_id <= swarm_num_m)){
            ROS_ERROR("Invalid TARGETID = %d, LOCALID = %d.\n", o_target_id, o_local_id);
            continue;
        }

        switch(msg_id){
            case 1:
                if (a1.decode(payload)) MSGCB(a1, o_target_id, o_local_id);
                else ROS_ERROR("Unable to decode: MSG_ID = %d, targetID = %d, localID = %d, payloadLen = %zu.\n", msg_id, o_target_id, o_local_id, payload.size());
                break;
            case 2:
                if (a2.decode(payload)) MSGCB(a2, o_target_id, o_local_id);
                else ROS_ERROR("Unable to decode: MSG_ID = %d, targetID = %d, localID = %d, payloadLen = %zu.\n", msg_id, o_target_id, o_local_id, payload.size());
                break;
            case 3:
                if (a3.decode(payload)) MSGCB(a3, o_target_id, o_local_id);
                else ROS_ERROR("Unable to decode: MSG_ID = %d, targetID = %d, localID = %d, payloadLen = %zu.\n", msg_id, o_target_id, o_local_id, payload.size());
                break;
            case 101:
                if (a101.decode(payload)) MSGCB(a101, o_target_id, o_local_id);
                else ROS_ERROR("Unable to decode: MSG_ID = %d, targetID = %d, localID = %d, payloadLen = %zu.\n", msg_id, o_target_id, o_local_id, payload.size());
                break;
            case 255:
                if (a255.decode(payload)) MSGCB(a255, o_target_id, o_local_id);
                else ROS_ERROR("Unable to decode: MSG_ID = %d, targetID = %d, localID = %d, payloadLen = %zu.\n", msg_id, o_target_id, o_local_id, payload.size());
                break;
            default: // 不合法
                ROS_ERROR("Invalid MSG_ID = %d.\n", msg_id);
        }

    }
}

void base_proxy::MSGCB(const MSG_1& msg, uint8_t o_target_id, uint8_t o_local_id){
    // 以topic形式发送飞行数据
    // 广播该消息（可选）

    if (o_target_id == id_m || o_target_id == 255){

        radio_proxy::FlightData msg_pub;

        msg_pub.header.stamp = ros::Time::now();

        msg_pub.id = o_local_id;

        msg_pub.latitude = msg.latitude;
        msg_pub.longitude = msg.longitude;
        msg_pub.altitude = msg.altitude;

        msg_pub.latitude_rtk = msg.latitude_rtk;
        msg_pub.longitude_rtk = msg.longitude_rtk;
        msg_pub.altitude_rtk = msg.altitude_rtk;

        msg_pub.position_local.x = msg.position_local.x;
        msg_pub.position_local.y = msg.position_local.y;
        msg_pub.position_local.z = msg.position_local.z;

        msg_pub.position_local_rtk.x = msg.position_local_rtk.x;
        msg_pub.position_local_rtk.y = msg.position_local_rtk.y;
        msg_pub.position_local_rtk.z = msg.position_local_rtk.z;

        msg_pub.velocity_local.x = msg.velocity_local.x;
        msg_pub.velocity_local.y = msg.velocity_local.y;
        msg_pub.velocity_local.z = msg.velocity_local.z;

        msg_pub.velocity_local_rtk.x = msg.velocity_local_rtk.x;
        msg_pub.velocity_local_rtk.y = msg.velocity_local_rtk.y;
        msg_pub.velocity_local_rtk.z = msg.velocity_local_rtk.z;

        msg_pub.accel_local.x = msg.accel_local.x;
        msg_pub.accel_local.y = msg.accel_local.y;
        msg_pub.accel_local.z = msg.accel_local.z;

        msg_pub.rpy.x = msg.rpy.x;
        msg_pub.rpy.y = msg.rpy.y;
        msg_pub.rpy.z = msg.rpy.z;

        msg_pub.gimbal_angular.x = msg.gimbal_angular.x;
        msg_pub.gimbal_angular.y = msg.gimbal_angular.y;
        msg_pub.gimbal_angular.z = msg.gimbal_angular.z;

        msg_pub.yaw_rate = msg.yaw_rate;    
        msg_pub.height_above_takeoff = msg.height_above_takeoff;

        fdPub.publish(msg_pub);
    }

    if (broadcast_m) write(msg.encode(), o_target_id, msg.msg_id(), o_local_id);    
}
void base_proxy::MSGCB(const MSG_2& msg, uint8_t o_target_id, uint8_t o_local_id){
    // 以topic形式发送状态数据
    // 广播该消息（可选）

    if (o_target_id == id_m || o_target_id == 255){

        radio_proxy::Status msg_pub;

        msg_pub.header.stamp = ros::Time::now();

        msg_pub.id = o_local_id;

        msg_pub.battery_v = msg.battery_v;
        msg_pub.gps_health = msg.gps_health;
        msg_pub.armed = msg.armed;
        msg_pub.flight_state = msg.flight_state;
        msg_pub.rc_connected = msg.rc_connected;
        msg_pub.rtk_connected = msg.rtk_connected;
        msg_pub.mission = msg.mission;
        msg_pub.connected = msg.connected;
        msg_pub.mode = msg.mode;

        statusPub.publish(msg_pub);
    }

    if (broadcast_m) write(msg.encode(), o_target_id, msg.msg_id(), o_local_id);    
}
void base_proxy::MSGCB(const MSG_3& msg, uint8_t o_target_id, uint8_t o_local_id){
    // 以topic形式发送状态数据
    // 广播该消息（可选）

    if (o_target_id == id_m || o_target_id == 255){

        radio_proxy::Target msg_pub;

        msg_pub.header.stamp = ros::Time::now();

        msg_pub.id = o_local_id;

        msg_pub.target_latitude = msg.target_latitude;
        msg_pub.target_longitude = msg.target_longitude;
        msg_pub.target_altitude = msg.target_altitude;

        msg_pub.target_position_body_frame.x = msg.target_position_body_frame.x;
        msg_pub.target_position_body_frame.y = msg.target_position_body_frame.y;
        msg_pub.target_position_body_frame.z = msg.target_position_body_frame.z;

        msg_pub.target_position_in_photo_x = msg.target_position_in_photo.x;
        msg_pub.target_position_in_photo_y = msg.target_position_in_photo.y;

        tarPub.publish(msg_pub);
    }

    if (broadcast_m) write(msg.encode(), o_target_id, msg.msg_id(), o_local_id);    
}
void base_proxy::MSGCB(const MSG_101& msg, uint8_t o_target_id, uint8_t o_local_id){
    // 仅广播该消息（可选）
    if (broadcast_m) write(msg.encode(), o_target_id, msg.msg_id(), o_local_id);    
}
void base_proxy::MSGCB(const MSG_255& msg, uint8_t o_target_id, uint8_t o_local_id){
    // 仅广播该消息（可选）    
    if (broadcast_m) write(msg.encode(), o_target_id, msg.msg_id(), o_local_id);    
}

void base_proxy::write(const string& payload, uint8_t target_id, uint8_t msg_id){
    string data;
    char TARGET_ID = target_id;
    char LOCAL_ID = id_m;
    char MSG_ID = msg_id;
    data = data + HEAD1 + HEAD2 + MSG_ID + TARGET_ID + LOCAL_ID + payload + END1 + END2;
    sp_m.write(data);
}

void base_proxy::write(const string& payload, uint8_t target_id, uint8_t msg_id, uint8_t o_local_id){
    string data;
    char TARGET_ID = target_id;
    char LOCAL_ID = o_local_id;
    char MSG_ID = msg_id;
    data = data + HEAD1 + HEAD2 + MSG_ID + TARGET_ID + LOCAL_ID + payload + END1 + END2;
    sp_m.write(data);
}

}

