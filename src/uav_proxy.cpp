#include "radio_proxy/uav_proxy.h"

namespace radio_proxy{

uav_proxy::uav_proxy(){
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("prefix", prefix_m, string("dji_sdk"));

    cmdPub = private_nh.advertise<radio_proxy::Command>("command", 1);        

    msgSub = private_nh.subscribe<radio_proxy::String>("message", 2, &uav_proxy::msgCB, this);
    tarSub = private_nh.subscribe<radio_proxy::Target>("target_self", 2, &uav_proxy::tarCB, this);

    resendFlightDataTimer = nh.createTimer(ros::Duration(0.2), &uav_proxy::resentFlightDataCB, this);
    resendStatusTimer = nh.createTimer(ros::Duration(2), &uav_proxy::resentStatusCB, this);
}

uav_proxy::~uav_proxy(){

}

void uav_proxy::resentFlightDataCB(const ros::TimerEvent &event){
    write(msg_1_m.encode(), 255, msg_1_m.msg_id());
}

void uav_proxy::resentStatusCB(const ros::TimerEvent &event){
    write(msg_2_m.encode(), 255, msg_2_m.msg_id());
}

void uav_proxy::msgCB(const radio_proxy::String::ConstPtr &msg_p){
    // 获取字符串消息，传输给目标(实际只有地面站接收该信息，其他结点要么转发要么抛弃)
    MSG_255 msg_255;
    msg_255.decode(msg_p->data);
    write(msg_255.encode(), msg_p->id, msg_255.msg_id());
}

void uav_proxy::tarCB(const radio_proxy::Target::ConstPtr &msg_p){
    // 将目标数据传输给指定结点
    MSG_3 msg_3;

    msg_3.target_latitude = msg_p->target_latitude;
    msg_3.target_longitude = msg_p->target_longitude;
    msg_3.target_altitude = msg_p->target_altitude;

    msg_3.target_position_body_frame.x = msg_p->target_position_body_frame.x;
    msg_3.target_position_body_frame.y = msg_p->target_position_body_frame.y;
    msg_3.target_position_body_frame.z = msg_p->target_position_body_frame.z;

    msg_3.target_position_in_photo.x = msg_p->target_position_in_photo_x;
    msg_3.target_position_in_photo.y = msg_p->target_position_in_photo_y;
    
    write(msg_3.encode(), msg_p->id, msg_3.msg_id());
}

void uav_proxy::MSGCB(const MSG_101& msg, uint8_t o_target_id, uint8_t o_local_id){
    // 以topic形式发送地面站指令
    // 广播该消息（可选）

    if (o_target_id == id_m || o_target_id == 255){

        radio_proxy::Command msg_pub;

        msg_pub.header.stamp = ros::Time::now();
        
        msg_pub.id = o_local_id;
        
        msg_pub.mission = msg.mission;

        msg_pub.latitude = msg.target_latitude;
        msg_pub.longitude = msg.target_longitude;
        msg_pub.altitude = msg.target_altitude;

        msg_pub.x = msg.target_position.x;
        msg_pub.y = msg.target_position.y;
        msg_pub.z = msg.target_position.z;

        msg_pub.yaw = msg.yaw;

        cmdPub.publish(msg_pub);
    }
    // 改变mission标志位，让定时器回传
    msg_2_m.mission = msg.mission;

    if (broadcast_m) write(msg.encode(), o_target_id, msg.msg_id(), o_local_id);    
}

}