///@file distance_test.cpp
///@author Vinson Sheep (775014077@qq.com)
///@brief 
///@version 1.0
///@date 2021-10-12
///
///@copyright Copyright (c) 2021
///

///@brief 拉距测试，打印地面站数据的频率
///

#include <ros/ros.h>
#include <iostream>
#include "radio_proxy/FlightData.h"
#include "radio_proxy/Status.h"
#include "sstream"
#include "boost/bind.hpp"

// subscriber
ros::Subscriber statusSub[9];
ros::Subscriber flightDataSub[9];

int status_count[9] = {0};
int flight_data_count[9] = {0};

int uav_num;

void statusCB(const radio_proxy::Status::ConstPtr &msg_p, int index){
    ++status_count[index];
}

void flightDataCB(const radio_proxy::FlightData::ConstPtr &msg_p, int index){
    ++flight_data_count[index];
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    ros::Rate r(1);

    // get param
    private_nh.param("uav_num", uav_num, 1);

    // initialization
    for (int i=1; i<=uav_num; i++){
        
        std::stringstream ss_s;
        std::stringstream ss_f;
 
        ss_s << "/dji_" << i << "/status";
        ss_f << "/dji_" << i << "/flight_data";

        statusSub[i] = nh.subscribe<radio_proxy::Status>(ss_s.str().c_str(), 1, boost::bind(&statusCB, _1, i));
        flightDataSub[i] = nh.subscribe<radio_proxy::FlightData>(ss_f.str().c_str(), 1, boost::bind(&flightDataCB, _1, i));
    }

    ros::Time begin = ros::Time::now();

    while(ros::ok())
    {
        ros::Time now = ros::Time::now();
        double dur = (now-begin).toSec();

        std::stringstream ss;

        ss << std::fixed << std::setprecision(2) << std::endl;
        
        for (int i=1; i<=uav_num; i++){

            double f_hz, s_hz;
            if (flight_data_count[i] == 0){
                f_hz = 0;
            }
            else{
                f_hz = (now - begin).toSec()/flight_data_count[i];
            }

            if (status_count[i] == 0){
                s_hz = 0;
            }
            else{
                s_hz = (now - begin).toSec()/status_count[i];
            }

            ss << "dji_" << i << ": (flight_data_hz: " << f_hz << ", status_hz: " << s_hz << ")." << std::endl;
        }

        ROS_INFO("%s", ss.str().c_str());

        ros::spinOnce();
        r.sleep();
    }
    
 
    return 0;
}
