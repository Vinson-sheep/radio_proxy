//serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "string.h"
#include "radio_proxy/data_type.h"
#include "string.h"


int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
    
    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB1");
    //设置串口通信的波特率
    sp.setBaudrate(230400);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB1 is opened.");
    }
    else
    {
        return -1;
    }
    
    uint8_t count = 0;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {

        std::string str = "hello";
        uint8_t buffer[1024];

        buffer[0] = 0X5A;
        buffer[1] = 1;
        buffer[2] = 2;
        buffer[3] = 3;

        dji::MSG_1 msg_1;
        dji::MSG_2 msg_2;

        memcpy(buffer+4, &msg_1, sizeof(msg_1));
        
        buffer[4+sizeof(msg_1)] = 0X0D;
        buffer[4+sizeof(msg_1) + 1] = 0X0A;

        for (uint8_t i=0; i<(4+sizeof(msg_1) + 2); i++){
            printf("%X ", buffer[i]);
        }
        printf("\n");

        sp.write(buffer, 4+sizeof(msg_1) + 2);

        loop_rate.sleep();
        ros::spinOnce();


    }
    
    //关闭串口
    sp.close();
 
    return 0;
}
