#include "radio_proxy/px4_proxy.h"

namespace px4{

px4_proxy::px4_proxy(){

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // get param
    private_nh.param("id", _id, 1);
    private_nh.param("port", _port, std::string("/dev/ttyUSB0"));
    private_nh.param("baud_rate", _baud_rate, 230400);

    // initialize serial
    try 
    { 
        _sp.setPort(_port); 
        _sp.setBaudrate(_baud_rate); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        _sp.setTimeout(to); 
        _sp.open();
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR("Unable to open port %s.", _port.c_str()); 
    } 

    if(_sp.isOpen()) 
    { 
        ROS_INFO("Serial Port %s initialized.", _port.c_str()); 
    } 
    else
    { 
    } 

    // publisher
    _leaderFdPub = private_nh.advertise<radio_proxy::FlightData>("leader/flight_data", 1);
    _cmdPub = private_nh.advertise<radio_proxy::Command>("command", 1);

    // subscriber
    _stateSub = nh.subscribe<>("/mavros/state", 2, &px4_proxy::stateCB, this);
	_velSub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 2, &px4_proxy::velCB, this);
	_localPoseSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 2, &px4_proxy::localPoseCB, this);
	_batStaSub = nh.subscribe<mavros_msgs::BatteryStatus>("/mavros/battery", 2, &px4_proxy::batStaSub, this);


	_msgSub = private_nh.subscribe<std_msgs::String>("message", 2, &px4_proxy::msgCB, this);


    // timer
    _serialLoopTimer = nh.createTimer(ros::Duration(0.0005), &px4_proxy::serialLoopCB, this);
    _mainLoopTimer = nh.createTimer(ros::Duration(0.005), &px4_proxy::mainLoopCB, this);
    _resendFlightDataTimer = nh.createTimer(ros::Duration(0.1), &px4_proxy::resentFlightDataCB, this);
    _resendStatusTimer = nh.createTimer(ros::Duration(1), &px4_proxy::resentStatusCB, this);

    // buffer
    for (int i=0; i<2; i++){
        _buffer_empty[i] = true;
        _buffer_len[i] = 0;
        _buffer_available[i] = false;
    }
    _buffer_index = 0;
    _flag = false;

}

px4_proxy::~px4_proxy(){
    _sp.close();
}

///@brief get one byte from serial and store in proper buffer
///
///@param event 
void px4_proxy::serialLoopCB(const ros::TimerEvent &event){
    // read serial data and store
    if (_sp.available()==0){
        return;
    }

    uint8_t ch;

    _sp.read(&ch, 1);

    // check header
    if (ch == 0X5A){
        _flag = true;
        _buffer_empty[_buffer_index] = false;
    }

    if (_flag == true){
        _buffer_read[_buffer_index][_buffer_len[_buffer_index]++] = ch;
        // check tail
        if (_buffer_read[_buffer_index][_buffer_len[_buffer_index]-2] == 0X0D &&
            _buffer_read[_buffer_index][_buffer_len[_buffer_index]-1] == 0X0A){
            _buffer_available[_buffer_index] = true;
            // change buffer
            for (int i = 0; i<2; i++){
                if (_buffer_empty[i] == true){
                    _buffer_index = i;
                    break;
                }
            }
            _flag = false;
        }
    }
}

///@brief get command and send by serial
///
///@param msg_p 
void px4_proxy::mainLoopCB(const ros::TimerEvent &event){
    for (int i=0; i<2; i++){
        if (_buffer_available[i] == true){
            // copy data and release buffer
            uint8_t buffer[1024];
            memcpy(buffer, _buffer_read[i], _buffer_len[i]);
            _buffer_available[i] = false;
            _buffer_empty[i] = true;
            _buffer_len[i] = 0;
            // get data
            uint8_t msg_id = buffer[1];
            uint8_t target_id = buffer[2];
            uint8_t local_id = buffer[3];

            if (msg_id == 1 && local_id == 1 && target_id == 255){ // flight data from leader
                
                MSG_1 msg_1;
                memcpy(&msg_1, &buffer[4], sizeof(msg_1));
                
                radio_proxy::FlightData fd;
                fd.header.stamp = ros::Time::now();
                fd.id = local_id;
                
                // copy data
                fd.x = msg_1.x;
                fd.y = msg_1.y;
                fd.z = msg_1.z;
                fd.vx = msg_1.vx;
                fd.vy = msg_1.vy;
                fd.vz = msg_1.vz;
                fd.ax = msg_1.ax;
                fd.ay = msg_1.ay;
                fd.az = msg_1.az;
                fd.pitch = msg_1.pitch;
                fd.roll = msg_1.roll;
                fd.yaw = msg_1.yaw;
                fd.yaw_rate = msg_1.yaw_rate;

                // publish
                _leaderFdPub.publish(fd);
            }
            else if (local_id == 254 && msg_id >= 101 && msg_id <= 107 
                        && (target_id == _id || target_id == 255)){ // command from station
                radio_proxy::Command cmd;
                cmd.header.stamp = ros::Time::now();
                cmd.target_id = _id;

                if (msg_id == 101){
                    MSG_101 msg_101;
                    memcpy(&msg_101, &buffer[4], sizeof(msg_101));

                    cmd.latitude = msg_101.latitude;
                    cmd.longitude = msg_101.longitude;
                    cmd.altitude = msg_101.altitude;
                    cmd.yaw = msg_101.yaw;

                    cmd.command = msg_id;
                }
                else if (msg_id == 102){
                    MSG_102 msg_102;
                    memcpy(&msg_102, &buffer[4], sizeof(msg_102));

                    cmd.x = msg_102.x;
                    cmd.y = msg_102.y;
                    cmd.z = msg_102.z;
                    cmd.yaw = msg_102.yaw;

                    cmd.command = msg_id;
                }
                else if (msg_id == 103){
                    cmd.command = msg_id;
                }
                else if (msg_id == 104){
                    cmd.command = msg_id;
                }
                else if (msg_id == 105){
                    cmd.command = msg_id;
                }
                else if (msg_id == 106){
                    cmd.command = msg_id;
                }
                else if (msg_id == 107){
                    cmd.command = msg_id;
                }
                // publish
                _cmdPub.publish(cmd);
            }
        }
    }
        
}

///@brief send flight data of this vechile to station
///
///@param event 
void px4_proxy::resentFlightDataCB(const ros::TimerEvent &event){

    // header
    _buffer_write[0] = 0X5A;
    _buffer_write[1] = 1;
    if (_id == 1){
        _buffer_write[2] = 255;
    }
    else{
        _buffer_write[2] = 254;
    }
    _buffer_write[3] = _id;
    size_t n = 4;

    // payload
    memcpy(_buffer_write+n, &_msg_1, sizeof(_msg_1));
    n += sizeof(_msg_1);

    // tail
    _buffer_write[n++] = 0X0D;
    _buffer_write[n++] = 0X0A;
    _sp.write(_buffer_write, n);

}

///@brief send status of this vechile to station
///
///@param event 
void px4_proxy::resentStatusCB(const ros::TimerEvent &event){

    // header
    _buffer_write[0] = 0X5A;
    _buffer_write[1] = 2;
    _buffer_write[2] = 254;
    _buffer_write[3] = _id;
    size_t n = 4;

    // payload
    memcpy(_buffer_write+n, &_msg_2, sizeof(_msg_2));
    n += sizeof(_msg_2);

    // tail
    _buffer_write[n++] = 0X0D;
    _buffer_write[n++] = 0X0A;
    _sp.write(_buffer_write, n);

}

void px4_proxy::stateCB(const mavros_msgs::State::ConstPtr &msg_p){
    _msg_2.connected = msg_p->connected;
    _msg_2.armed = msg_p->armed;
    _msg_2.manual_input = msg_p->manual_input;

    int i;
    for (i=0; i<msg_p->mode.length(); i++){
        _msg_2.mode[i] = msg_p->mode[i];
    }
    _msg_2.mode[i] = '\0';
    
}

void px4_proxy::velCB(const geometry_msgs::TwistStamped::ConstPtr &msg_p){
    _msg_1.vx = msg_p->twist.linear.x;
    _msg_1.vy = msg_p->twist.linear.y;
    _msg_1.vz = msg_p->twist.linear.z;
    _msg_1.yaw_rate = msg_p->twist.angular.z;
}

void px4_proxy::localPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg_p){
    _msg_1.x = msg_p->pose.position.x;
    _msg_1.y = msg_p->pose.position.y;
    _msg_1.z = msg_p->pose.position.z;
    // gei RPY
    double r,p,y;
    tf2::getEulerYPR(msg_p->pose.orientation, y, p, r);
    _msg_1.yaw = y;
    _msg_1.pitch = p;
    _msg_1.roll = r;
}

void px4_proxy::batStaSub(const mavros_msgs::BatteryStatus::ConstPtr &msg_p){
    _msg_2.battery_v = msg_p->voltage;
}

void px4_proxy::msgCB(const std_msgs::String::ConstPtr &msg_p){
    MSG_255 msg_255;
    int len = std::min(sizeof(msg_255), msg_p->data.length());
    for (int i = 0; i<len; i++){
        msg_255.data[i] = msg_p->data[i];
    }
    len = len ==  sizeof(msg_255) ? len-1: len;  
    msg_255.data[len] = '\0';

    _buffer_write[0] = 0X5A;
    _buffer_write[1] = 255;
    _buffer_write[2] = 254;
    _buffer_write[3] = _id;
    size_t n = 4;
    memcpy(_buffer_write+4, &msg_255, sizeof(msg_255));
    n += sizeof(msg_255);
    _buffer_write[n++] = 0X0D;
    _buffer_write[n++] = 0X0A;
    _sp.write(_buffer_write, n);
}


}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"px4_proxy");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // wait for px4_sdk node setting up
    ros::Duration(3).sleep();

    px4::px4_proxy pp;

    ros::spin();

    return 0;
}