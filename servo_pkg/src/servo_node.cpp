#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Int16MultiArray.h>
#include <SCServo.h>
#include <signal.h>

SMS_STS ServoDriver;

void key_callback(const std_msgs::Int16MultiArray& msg){
    ROS_INFO("Servo_node:recv %d %d %d %d and send to servo",msg.data[0],msg.data[1],msg.data[2],msg.data[3]);
    //ServoDriver.WritePosEx(1,msg.data[0],msg.data[1],254);
	//ServoDriver.WritePosEx(2,msg.data[2],msg.data[3],254);
}

void signalHandler(int sig)
{
    // 在这里编写你的中断处理代码
    ServoDriver.WritePosEx(1,0,1000,254);
	ServoDriver.WritePosEx(2,0,1000,254);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "servo_node");
    ros::NodeHandle nh;
    //读取其他节点发布的servowrite话题调用write_callback
    ros::Subscriber servo_sub = nh.subscribe("servo/write", 10, key_callback);
    //向其它节点发布servoread信息
    ros::Publisher read_pub = nh.advertise<std_msgs::Int16MultiArray>("servo/read", 10);

    ROS_INFO("Intializing Servo...");
    if(!ServoDriver.begin(115200, "/dev/ttyUSB1"))
	{
        ROS_ERROR("Failed to init servo!");
        return -1;
    }
    else
    {
        ROS_INFO("Servo initialized");
    }
    // 100hz频率执行
    std::string read_str;

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        //待添加舵机闭环控制？
        //if (ServoDriver.available()) {
        //    read_str=ServoDriver.read(ServoDriver.available());
        //    ROS_INFO("Servo Read: %s",read_str.c_str());
        //}
        loop_rate.sleep();
    }
    ServoDriver.end();
}