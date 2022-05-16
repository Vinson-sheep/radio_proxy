#include "radio_proxy/PX4_proxy.h"
#include "tf/transform_datatypes.h"

namespace radio_proxy{

PX4_proxy::PX4_proxy(){

    ros::NodeHandle nh;

	batStaSub = nh.subscribe<sensor_msgs::BatteryState>(prefix_m + "/battery", 2, &PX4_proxy::batStaCB, this);
	staSub = nh.subscribe<mavros_msgs::State>(prefix_m +"/state", 2, &PX4_proxy::staCB, this);
	gpsPosSub = nh.subscribe<sensor_msgs::NavSatFix>(prefix_m +"/global_position/global", 2, &PX4_proxy::gpsPosCB, this);
	velSub = nh.subscribe<geometry_msgs::TwistStamped>(prefix_m +"/local_position/velocity", 2, &PX4_proxy::velCB, this);
	localPosSub = nh.subscribe<geometry_msgs::PoseStamped>(prefix_m +"/local_position/pose", 2, &PX4_proxy::localPosCB, this);

}

PX4_proxy::~PX4_proxy(){

}


void PX4_proxy::batStaCB(const sensor_msgs::BatteryState::ConstPtr &msg_p){
    msg_2_m.battery_v = msg_p->voltage;
}
void PX4_proxy::staCB(const mavros_msgs::State::ConstPtr &msg_p){
    msg_2_m.connected = msg_p->connected;
    msg_2_m.armed = msg_p->armed;
    msg_2_m.rc_connected = msg_p->manual_input;
    msg_2_m.mode_len = msg_p->mode.size();
    msg_2_m.mode = msg_p->mode;
}
void PX4_proxy::gpsPosCB(const sensor_msgs::NavSatFix::ConstPtr &msg_p){
    msg_1_m.latitude = msg_p->latitude;
    msg_1_m.longitude = msg_p->longitude;
    msg_1_m.altitude = msg_p->altitude;
}
void PX4_proxy::velCB(const geometry_msgs::TwistStamped::ConstPtr &msg_p){
    msg_1_m.velocity_local.x = msg_p->twist.linear.x;
    msg_1_m.velocity_local.y = msg_p->twist.linear.y;
    msg_1_m.velocity_local.z = msg_p->twist.linear.z;
    msg_1_m.yaw_rate = msg_p->twist.angular.z;
}
void PX4_proxy::localPosCB(const geometry_msgs::PoseStamped::ConstPtr &msg_p){
    msg_1_m.position_local.x = msg_p->pose.position.x;
    msg_1_m.position_local.y = msg_p->pose.position.y;
    msg_1_m.position_local.z = msg_p->pose.position.z;
    // gei RPY
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg_p->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    msg_1_m.rpy.x = roll;
    msg_1_m.rpy.y = pitch;
    msg_1_m.rpy.z = yaw;
}

}