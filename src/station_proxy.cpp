#include "radio_proxy/station_proxy.h"

namespace px4{

station_proxy::station_proxy()
{
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // get param
    private_nh.param("uav_num", _uav_num, 1);
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
    for (int i=1 ;i<=_uav_num; i++){
        std::stringstream ss_s;
        std::stringstream ss_f;
        std::stringstream ss_m;
        ss_s << "/px4_" << i << "/status";
        ss_f << "/px4_" << i << "/flight_data";
        ss_m << "/px4_" << i << "/message";

        _statusPub[i] = nh.advertise<radio_proxy::Status>(ss_s.str().c_str(), 1);
        _flightDataPub[i] = nh.advertise<radio_proxy::FlightData>(ss_f.str().c_str(), 1);
        _msgPub[i] = nh.advertise<std_msgs::String>(ss_m.str().c_str(), 1);
    }

    // subscriber
    _cmdSub = private_nh.subscribe<radio_proxy::Command>("command", 2, &station_proxy::cmdCB, this);

    // timer
    _serialLoopTimer = nh.createTimer(ros::Duration(0.0002), &station_proxy::serialLoopCB, this);
    _mainLoopTimer = nh.createTimer(ros::Duration(0.005), &station_proxy::mainLoopCB, this);

    // buffer
    for (int i=0; i<3; i++){
        _buffer_empty[i] = true;
        _buffer_len[i] = 0;
        _buffer_available[i] = false;
    }
    _buffer_index = 0;
    _flag = false;

}


station_proxy::~station_proxy()
{
    _sp.close();
}

///@brief get one byte from serial and store in proper buffer
///
///@param event 
void station_proxy::serialLoopCB(const ros::TimerEvent &event){
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
            for (int i = 0; i<3; i++){
                if (_buffer_empty[i] == true){
                    _buffer_index = i;
                    break;
                }
            }
            _flag = false;
        }
    }
}

///@brief do something when data in buffer is available
///
/// TODO
///@param event 
void station_proxy::mainLoopCB(const ros::TimerEvent &event){
    for (int i=0; i<3; i++){
        if (_buffer_available[i] == true){
            // copy data and release buffer
            uint8_t buffer[1024];
            size_t len = _buffer_len[i];
            memcpy(buffer, _buffer_read[i], len);
            _buffer_available[i] = false;
            _buffer_empty[i] = true;
            _buffer_len[i] = 0;
            // get data
            uint8_t msg_id = buffer[1];
            uint8_t target_id = buffer[2];
            uint8_t local_id = buffer[3];

            if (local_id < 1 || local_id > _uav_num){
                ROS_ERROR("unvalid local id.");
                ROS_ERROR("local id: %X", local_id);
            }

            if (msg_id == 1){ // flight data
                
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
                _flightDataPub[local_id].publish(fd);

                // resend if data is from leader
                if (local_id == 1){
                    buffer[2] = 255;
                    _sp.write(buffer, len);
                }
            }
            else if (msg_id == 2){

                MSG_2 msg_2;
                memcpy(&msg_2, &buffer[4], sizeof(msg_2));

                radio_proxy::Status st;

                st.header.stamp = ros::Time::now();
                st.id = local_id;

                // copy data
                st.battery_v = msg_2.battery_v;
                st.connected = msg_2.connected == 0 ? false: true;
                st.armed = msg_2.armed == 0 ? false: true;
                st.manual_input = msg_2.manual_input == 0 ? false: true;
                st.mode = msg_2.mode;

                // publish
                _statusPub[local_id].publish(st);
            }
            else if (msg_id == 255){

                MSG_255 msg_255;
                memcpy(&msg_255, &buffer[4], sizeof(msg_255));
                
                std_msgs::String ss;
                ss.data = msg_255.data;
                
                // publish
                _msgPub[local_id].publish(ss);
            }
        }
    }
}

///@brief get command and send by serial
///
///@param msg_p 
void station_proxy::cmdCB(const radio_proxy::Command::ConstPtr &msg_p){

    uint8_t MSG_ID = msg_p->command;
    uint8_t TARGET_ID = msg_p->target_id;
    uint8_t LOCAL_ID = 254;

    // check
    if ((TARGET_ID < 1 || TARGET_ID > _uav_num) && TARGET_ID != 255){
        ROS_ERROR("unvalid command.");
        return;
    }

    _buffer_write[0] = 0X5A;
    _buffer_write[1] = MSG_ID;
    _buffer_write[2] = TARGET_ID;
    _buffer_write[3] = LOCAL_ID;

    size_t n = 4;

    if (MSG_ID == 101){
        MSG_101 msg_101;
        msg_101.altitude = msg_p->altitude;
        msg_101.latitude = msg_p->latitude;
        msg_101.longitude = msg_p->longitude;
        msg_101.yaw = msg_p->yaw;
        memcpy(_buffer_write+n, &msg_101, sizeof(msg_101));
        n += sizeof(MSG_101);
    }
    else if (MSG_ID == 102){
        MSG_102 msg_102;
        msg_102.x = msg_p->x;
        msg_102.y = msg_p->y;
        msg_102.z = msg_p->z;
        msg_102.yaw = msg_p->yaw;
        memcpy(_buffer_write+n, &msg_102, sizeof(msg_102));
        n += sizeof(MSG_102);
    }
    else if (MSG_ID == 103){

    }
    else if (MSG_ID == 104){

    }
    else if (MSG_ID == 105){

    }
    else if (MSG_ID == 106){

    }
    else if (MSG_ID == 107){
        
    }
    else{
        ROS_ERROR("unvalid command.");
    }

    // send data by serial
    _buffer_write[n++] = 0X0D;
    _buffer_write[n++] = 0X0A;
    _sp.write(_buffer_write, n);

}

} // end namespace px4




int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"station_proxy");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    px4::station_proxy proxy;

    ros::spin();

    return 0;
}
