#include "radio_proxy/GS_proxy.h"

namespace radio_proxy{

GS_proxy::GS_proxy(){
    ros::NodeHandle private_nh("~");
	msgPub = private_nh.advertise<radio_proxy::String>("message", 1);
	cmdSub = private_nh.subscribe<radio_proxy::Command>("command", 2, &GS_proxy::cmdCB, this);
}

GS_proxy::~GS_proxy(){}


void GS_proxy::cmdCB(const radio_proxy::Command::ConstPtr &msg_p){
    // 将地面站指令传输给指定结点
    MSG_101 msg_101;

    msg_101.mission = msg_p->mission;
    msg_101.target_latitude = msg_p->latitude;
    msg_101.target_longitude = msg_p->longitude;
    msg_101.target_altitude = msg_p->altitude;
    msg_101.target_position.x = msg_p->x;
    msg_101.target_position.y = msg_p->y;
    msg_101.target_position.z = msg_p->z;
    msg_101.yaw = msg_p->yaw;

    write(msg_101.encode(), msg_p->id, msg_101.msg_id());
}

void GS_proxy::MSGCB(const MSG_255& msg, uint8_t o_target_id, uint8_t o_local_id){
    // 以topic形式发送无人机消息
    radio_proxy::String msg_pub;

    msg_pub.header.stamp = ros::Time::now();

    msg_pub.id = o_local_id;
    msg_pub.data = msg.data;

    msgPub.publish(msg_pub);
}

}