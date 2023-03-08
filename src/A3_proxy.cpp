#include "radio_proxy/A3_proxy.h"

namespace radio_proxy{

A3_proxy::A3_proxy(){

    ros::NodeHandle nh;

	rtkStaSub = nh.subscribe<std_msgs::UInt8>(prefix_m + "/rtk_connection_status", 2, &A3_proxy::rtkStaCB, this);
	rtkPosSub = nh.subscribe<sensor_msgs::NavSatFix>(prefix_m + "/rtk_position", 2, &A3_proxy::rtkPosCB, this);
	rtkVelSub = nh.subscribe<geometry_msgs::Vector3Stamped>(prefix_m + "/rtk_velocity", 2, &A3_proxy::rtkVelCB, this);
    
}


A3_proxy::~A3_proxy(){
    
}

void A3_proxy::rtkStaCB(const std_msgs::UInt8::ConstPtr &msg_p){
    msg_2_m.rtk_connected = msg_p->data;
}
void A3_proxy::rtkPosCB(const sensor_msgs::NavSatFix::ConstPtr &msg_p){
    msg_1_m.latitude_rtk = msg_p->latitude;
    msg_1_m.longitude_rtk = msg_p->longitude;
    msg_1_m.altitude_rtk = msg_p->altitude;
}
void A3_proxy::rtkVelCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p){
    // msg_1_m.velocity_local_rtk.x = msg_p->vector.x;
    // msg_1_m.velocity_local_rtk.y = msg_p->vector.y;
    // msg_1_m.velocity_local_rtk.z = msg_p->vector.z;
}

}

