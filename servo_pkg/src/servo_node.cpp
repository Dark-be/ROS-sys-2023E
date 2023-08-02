#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Int16MultiArray.h>
#include <SCServo.h>

SMS_STS ServoDriver;

void write_callback(const std_msgs::Int16MultiArray& msg){
    ROS_INFO("Servo Target: %d %d %d %d",msg.data[0],msg.data[1],msg.data[2],msg.data[3]);
    //ServoDriver.WritePosEx(1,msg.data[0],msg.data[1],254);
	//ServoDriver.WritePosEx(2,msg.data[2],msg.data[3],254);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "servo_node");
    ros::NodeHandle nh;
    ros::Subscriber write_sub = nh.subscribe("servo/write", 10, write_callback);//写串口话题
    ROS_INFO("Intializing Servo...")
    if(!ServoDriver.begin(115200, "/dev/ttyUSB0"))
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

    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        //if (ServoDriver.available()) {
        //    read_str=ServoDriver.read(ServoDriver.available());
        //    ROS_INFO("Servo Read: %s",read_str.c_str());
        //}
        loop_rate.sleep();
    }
    ServoDriver.end();
}