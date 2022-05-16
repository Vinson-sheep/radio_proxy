#include "radio_proxy/N3_proxy.h"
#include "tf/transform_datatypes.h"

namespace radio_proxy{

N3_proxy::N3_proxy(){

    ros::NodeHandle nh;

    accSub = nh.subscribe<geometry_msgs::Vector3Stamped>(prefix_m + "/acceleration_ground_fused", 2, &N3_proxy::accCB, this);
	angVelSub = nh.subscribe<geometry_msgs::Vector3Stamped>(prefix_m + "/angular_velocity_fused", 2, &N3_proxy::angVelCB, this);
	altSub = nh.subscribe<geometry_msgs::QuaternionStamped>(prefix_m + "/attitude", 2, &N3_proxy::altCB, this);
	batStaSub = nh.subscribe<sensor_msgs::BatteryState>(prefix_m + "/battery_state", 2, &N3_proxy::batStaCB, this);
	rcStaSub = nh.subscribe<std_msgs::UInt8>(prefix_m + "/rc_connection_status", 2, &N3_proxy::rcStaCB, this);
	disMoSub = nh.subscribe<std_msgs::UInt8>(prefix_m + "/display_mode", 2, &N3_proxy::disMoCB, this);
	fliStaSub = nh.subscribe<std_msgs::UInt8>(prefix_m + "/flight_status", 2, &N3_proxy::fliStaCB, this);
	gpsHeaSub = nh.subscribe<std_msgs::UInt8>(prefix_m + "/gps_health", 2, &N3_proxy::gpsHeaCB, this);
	gpsPosSub = nh.subscribe<sensor_msgs::NavSatFix>(prefix_m + "/gps_position", 2, &N3_proxy::gpsPosCB, this);
	velSub = nh.subscribe<geometry_msgs::Vector3Stamped>(prefix_m + "/velocity", 2, &N3_proxy::velCB, this);
	heightSub = nh.subscribe<std_msgs::Float32>(prefix_m + "/height_above_takeoff", 2, &N3_proxy::heightCB, this);
	localPosSub = nh.subscribe<geometry_msgs::PointStamped>(prefix_m + "/local_position", 2, &N3_proxy::localPosCB, this);
    gimbalAngSub = nh.subscribe<geometry_msgs::Vector3Stamped>(prefix_m + "/gimbal_angle", 2, &N3_proxy::gimbalAngCB, this);

}

N3_proxy::~N3_proxy(){

}

void N3_proxy::accCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p){
    msg_1_m.accel_local.x = msg_p->vector.x;
    msg_1_m.accel_local.y = msg_p->vector.y;
    msg_1_m.accel_local.z = msg_p->vector.z;
}

void N3_proxy::angVelCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p){
    msg_1_m.yaw_rate = msg_p->vector.z;
}
void N3_proxy::altCB(const geometry_msgs::QuaternionStamped::ConstPtr &msg_p){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg_p->quaternion, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    msg_1_m.rpy.x = roll;
    msg_1_m.rpy.y = pitch;
    msg_1_m.rpy.z = yaw;

}
void N3_proxy::batStaCB(const sensor_msgs::BatteryState::ConstPtr &msg_p){
    msg_2_m.battery_v = msg_p->voltage/1000.0;
}
void N3_proxy::rcStaCB(const std_msgs::UInt8::ConstPtr &msg_p){
    msg_2_m.rc_connected = msg_p->data;
}
void N3_proxy::disMoCB(const std_msgs::UInt8::ConstPtr &msg_p){
    msg_2_m.connected = true;
}
void N3_proxy::fliStaCB(const std_msgs::UInt8::ConstPtr &msg_p){
    msg_2_m.armed = (msg_p->data == 1 || msg_p->data == 2);
    msg_2_m.flight_state = msg_p->data;
    msg_2_m.mode = "NONE";
    msg_2_m.mode_len = msg_2_m.mode.size();
}
void N3_proxy::gpsHeaCB(const std_msgs::UInt8::ConstPtr &msg_p){
    msg_2_m.gps_health = msg_p->data;
}
void N3_proxy::gpsPosCB(const sensor_msgs::NavSatFix::ConstPtr &msg_p){
    msg_1_m.latitude = msg_p->latitude;
    msg_1_m.longitude = msg_p->longitude;
    msg_1_m.altitude = msg_p->altitude;
}
void N3_proxy::velCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p){
    msg_1_m.velocity_local.x = msg_p->vector.x;
    msg_1_m.velocity_local.y = msg_p->vector.y;
    msg_1_m.velocity_local.z = msg_p->vector.z;
}
void N3_proxy::heightCB(const std_msgs::Float32::ConstPtr &msg_p){
    msg_1_m.height_above_takeoff = msg_p->data;
}
void N3_proxy::localPosCB(const geometry_msgs::PointStamped::ConstPtr &msg_p){
    msg_1_m.position_local.x = msg_p->point.x;
    msg_1_m.position_local.y = msg_p->point.y;
    msg_1_m.position_local.z = msg_p->point.z;
}
void N3_proxy::gimbalAngCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg_p){
    msg_1_m.gimbal_angular.x = msg_p->vector.x;
    msg_1_m.gimbal_angular.y = msg_p->vector.y;
    msg_1_m.gimbal_angular.z = msg_p->vector.z;
}

}